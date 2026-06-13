/**
 * @file mpu6050.c
 * @brief MPU-6050 IMU driver implementation (I2C via TWI hardware)
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Uses ATmega328P hardware TWI (I²C) peripheral.
 * SCL = PC5 (A5), SDA = PC4 (A4).
 * I2C clock: 400 kHz (fast mode).
 *
 * Crash detection: |acceleration| > CRASH_ACCEL_THRESHOLD_G
 * Rollover detection: |gyro roll rate| > ROLLOVER_GYRO_THRESHOLD
 */

#include "mpu6050.h"
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdint.h>

/* ════════════════════════════════════════════════════════════════════════════
 *  Internal I2C (TWI) helpers
 * ════════════════════════════════════════════════════════════════════════════ */

/* TWI bit-rate: F_I2C = F_CPU / (16 + 2×TWBR×4^TWPS)
 * For F_I2C=400kHz, F_CPU=16MHz, TWPS=0: TWBR = (16e6/400e3 - 16) / 2 = 12 */
#define TWI_TWBR_VAL     12U

static void twi_init(void)
{
    TWSR = 0;             /* Prescaler = 1 (TWPS = 0) */
    TWBR = TWI_TWBR_VAL; /* Set bit rate             */
    TWCR = (1U << TWEN);  /* Enable TWI               */
}

static uint8_t twi_start(uint8_t address)
{
    /* Send START condition */
    TWCR = (1U << TWINT) | (1U << TWSTA) | (1U << TWEN);
    while (!(TWCR & (1U << TWINT))) {}

    /* Load address+R/W and send */
    TWDR = address;
    TWCR = (1U << TWINT) | (1U << TWEN);
    while (!(TWCR & (1U << TWINT))) {}

    uint8_t status = (TWSR & 0xF8U);
    /* ACK received for write: 0x18; for read: 0x40 */
    return (status == 0x18U || status == 0x40U) ? 1U : 0U;
}

static void twi_stop(void)
{
    TWCR = (1U << TWINT) | (1U << TWSTO) | (1U << TWEN);
    while (TWCR & (1U << TWSTO)) {}
}

static uint8_t twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1U << TWINT) | (1U << TWEN);
    while (!(TWCR & (1U << TWINT))) {}
    return ((TWSR & 0xF8U) == 0x28U) ? 1U : 0U;  /* 0x28 = data ACK */
}

static uint8_t twi_read_ack(void)
{
    TWCR = (1U << TWINT) | (1U << TWEN) | (1U << TWEA);
    while (!(TWCR & (1U << TWINT))) {}
    return TWDR;
}

static uint8_t twi_read_nack(void)
{
    TWCR = (1U << TWINT) | (1U << TWEN);
    while (!(TWCR & (1U << TWINT))) {}
    return TWDR;
}

/* ════════════════════════════════════════════════════════════════════════════
 *  MPU-6050 register read/write helpers
 * ════════════════════════════════════════════════════════════════════════════ */

static uint8_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    if (!twi_start((MPU6050_ADDR << 1U) | 0U)) { twi_stop(); return 0; }
    if (!twi_write(reg))                        { twi_stop(); return 0; }
    if (!twi_write(val))                        { twi_stop(); return 0; }
    twi_stop();
    return 1U;
}

static uint8_t mpu_read_byte(uint8_t reg)
{
    uint8_t val;
    twi_start((MPU6050_ADDR << 1U) | 0U);
    twi_write(reg);
    twi_start((MPU6050_ADDR << 1U) | 1U);
    val = twi_read_nack();
    twi_stop();
    return val;
}

/**
 * @brief Read N consecutive bytes starting at reg into buf.
 */
static void mpu_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    twi_start((MPU6050_ADDR << 1U) | 0U);
    twi_write(reg);
    twi_start((MPU6050_ADDR << 1U) | 1U);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = (i < len - 1U) ? twi_read_ack() : twi_read_nack();
    }
    twi_stop();
}

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise MPU-6050.
 *        Wakes from sleep, sets sample rate, DLPF, and full-scale ranges.
 * @return MPU6050_OK on success.
 */
MPU6050_Status_t mpu6050_init(void)
{
    twi_init();
    _delay_ms(100);  /* Device startup delay */

    /* Check WHO_AM_I register */
    uint8_t who = mpu_read_byte(MPU6050_REG_WHO_AM_I);
    if (who != MPU6050_WHO_AM_I_VAL) {
        return MPU6050_ERR_WHO_AM_I;
    }

    /* Wake up device (clear sleep bit) */
    if (!mpu_write_reg(MPU6050_REG_PWR_MGMT1, 0x00U)) {
        return MPU6050_ERR_NACK;
    }
    _delay_ms(10);

    /* Sample rate divider: SMPLRT_DIV = 7 → sample rate = 1000/(1+7) = 125 Hz */
    mpu_write_reg(MPU6050_REG_SMPLRT_DIV, 0x07U);

    /* DLPF config: ~44 Hz bandwidth */
    mpu_write_reg(MPU6050_REG_CONFIG, 0x03U);

    /* Gyroscope full scale: ±250 °/s */
    mpu_write_reg(MPU6050_REG_GYRO_CFG, 0x00U);

    /* Accelerometer full scale: ±2g */
    mpu_write_reg(MPU6050_REG_ACCEL_CFG, 0x00U);

    return MPU6050_OK;
}

/**
 * @brief Read accelerometer and gyroscope data from MPU-6050.
 * @param data  Pointer to MPU6050_Data_t structure to populate.
 * @return MPU6050_OK on success.
 */
MPU6050_Status_t mpu6050_read(MPU6050_Data_t *data)
{
    uint8_t buf[14];

    /* Read 14 bytes: 6 accel + 2 temp + 6 gyro */
    mpu_read_burst(MPU6050_REG_ACCEL_XOUT_H, buf, 14U);

    /* Accelerometer: signed 16-bit big-endian */
    data->ax_raw = (int16_t)((buf[0]  << 8U) | buf[1]);
    data->ay_raw = (int16_t)((buf[2]  << 8U) | buf[3]);
    data->az_raw = (int16_t)((buf[4]  << 8U) | buf[5]);
    /* Skip temperature bytes: buf[6], buf[7] */
    data->gx_raw = (int16_t)((buf[8]  << 8U) | buf[9]);
    data->gy_raw = (int16_t)((buf[10] << 8U) | buf[11]);
    data->gz_raw = (int16_t)((buf[12] << 8U) | buf[13]);

    /* Convert to physical units */
    data->ax_g  = (float)data->ax_raw / MPU6050_ACCEL_SENS;
    data->ay_g  = (float)data->ay_raw / MPU6050_ACCEL_SENS;
    data->az_g  = (float)data->az_raw / MPU6050_ACCEL_SENS;
    data->gx_ds = (float)data->gx_raw / MPU6050_GYRO_SENS;
    data->gy_ds = (float)data->gy_raw / MPU6050_GYRO_SENS;
    data->gz_ds = (float)data->gz_raw / MPU6050_GYRO_SENS;

    /* Compute vector magnitude of acceleration */
    data->accel_magnitude_g = sqrtf(
        data->ax_g * data->ax_g +
        data->ay_g * data->ay_g +
        data->az_g * data->az_g
    );

    return MPU6050_OK;
}

/**
 * @brief Check if current IMU data indicates a crash event.
 * @param data  Pointer to filled MPU6050_Data_t.
 * @return 1 if crash detected, 0 otherwise.
 */
uint8_t mpu6050_crash_detected(const MPU6050_Data_t *data)
{
    /* A crash produces very high acceleration magnitude (deceleration spike) */
    return (data->accel_magnitude_g > CRASH_ACCEL_THRESHOLD_G) ? 1U : 0U;
}

/**
 * @brief Check if current IMU data indicates a vehicle rollover.
 * @param data  Pointer to filled MPU6050_Data_t.
 * @return 1 if rollover detected, 0 otherwise.
 */
uint8_t mpu6050_rollover_detected(const MPU6050_Data_t *data)
{
    /* Rollover: high roll rate about X or Y axis */
    float roll_rate = (fabsf(data->gx_ds) > fabsf(data->gy_ds))
                      ? fabsf(data->gx_ds)
                      : fabsf(data->gy_ds);
    return (roll_rate > ROLLOVER_GYRO_THRESHOLD) ? 1U : 0U;
}
