/**
 * @file mpu6050.h
 * @brief MPU-6050 6-axis IMU driver header (I2C)
 * @project Integrated Intelligent Vehicle Safety and Accident Prevention System
 *
 * Reads accelerometer (±2g) and gyroscope (±250°/s) data.
 * Used for crash/rollover/hard-braking detection.
 * I2C address: 0x68 (AD0 = GND).
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

/* ── I2C address ─────────────────────────────────────────────────────────── */
#define MPU6050_ADDR            0x68U

/* ── Register map (subset used) ──────────────────────────────────────────── */
#define MPU6050_REG_PWR_MGMT1   0x6BU
#define MPU6050_REG_SMPLRT_DIV  0x19U
#define MPU6050_REG_CONFIG      0x1AU
#define MPU6050_REG_GYRO_CFG    0x1BU
#define MPU6050_REG_ACCEL_CFG   0x1CU
#define MPU6050_REG_ACCEL_XOUT_H 0x3BU
#define MPU6050_REG_GYRO_XOUT_H  0x43U
#define MPU6050_REG_WHO_AM_I    0x75U
#define MPU6050_WHO_AM_I_VAL    0x68U

/* ── Sensitivity (LSB/g at ±2g; LSB/°s at ±250°/s) ─────────────────────── */
#define MPU6050_ACCEL_SENS      16384.0f  /* LSB/g   */
#define MPU6050_GYRO_SENS       131.0f    /* LSB/°/s */

/* ── Crash detection thresholds ──────────────────────────────────────────── */
#define CRASH_ACCEL_THRESHOLD_G    3.5f  /**< >3.5g = crash event            */
#define ROLLOVER_GYRO_THRESHOLD    180.0f/**< >180°/s roll-rate = rollover   */

/* ── Raw data structure ──────────────────────────────────────────────────── */
typedef struct {
    int16_t ax_raw, ay_raw, az_raw;   /**< Accelerometer raw (LSB) */
    int16_t gx_raw, gy_raw, gz_raw;   /**< Gyroscope raw (LSB)     */
    float   ax_g,  ay_g,  az_g;       /**< Acceleration in g       */
    float   gx_ds, gy_ds, gz_ds;      /**< Angular velocity °/s    */
    float   accel_magnitude_g;         /**< |a| = sqrt(ax²+ay²+az²) */
} MPU6050_Data_t;

/* ── Return codes ────────────────────────────────────────────────────────── */
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_NACK,
    MPU6050_ERR_WHO_AM_I
} MPU6050_Status_t;

/* ── API ─────────────────────────────────────────────────────────────────── */
MPU6050_Status_t mpu6050_init(void);
MPU6050_Status_t mpu6050_read(MPU6050_Data_t *data);
uint8_t          mpu6050_crash_detected(const MPU6050_Data_t *data);
uint8_t          mpu6050_rollover_detected(const MPU6050_Data_t *data);

#endif /* MPU6050_H */
