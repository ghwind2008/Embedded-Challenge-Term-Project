#ifndef LSM6DSL_H
#define LSM6DSL_H

#include "mbed.h"
#include <cstdint>

/**
 * LSM6DSL Accelerometer + Gyroscope Driver for mbed (I2C)
 * 
 * Enhanced version with full gyroscope support for FOG detection
 * Board: B-L475E-IOT01A (LSM6DSL on I2C2: PB_11=SDA, PB_10=SCL)
 */

#ifdef __cplusplus
extern "C" {
#endif

// ==================== Initialization ====================

/**
 * Initialize LSM6DSL sensor
 * Configures both accelerometer and gyroscope to 52Hz, enables auto-increment
 * @param i2c Pointer to configured I2C object
 * @return 0 on success, negative on error
 */
int LSM6DSL_Init(I2C *i2c);

/**
 * Read WHO_AM_I register (should return 0x6A for LSM6DSL)
 * @param whoami Pointer to store WHO_AM_I value
 * @return 0 on success, negative on error
 */
int LSM6DSL_ReadWhoAmI(uint8_t *whoami);

// ==================== Accelerometer Functions ====================

/**
 * Read raw accelerometer data (X, Y, Z)
 * @param x Pointer to store X-axis value (int16_t)
 * @param y Pointer to store Y-axis value (int16_t)
 * @param z Pointer to store Z-axis value (int16_t)
 * @return 0 on success, negative on error
 * 
 * Conversion for ±2g scale: value / 16384.0 = acceleration in g
 */
int LSM6DSL_ReadAccelXYZ(int16_t *x, int16_t *y, int16_t *z);

/**
 * Set accelerometer output data rate
 * @param odr_hz Output data rate in Hz (common: 12.5, 26, 52, 104, 208, 416)
 * @return 0 on success, negative on error
 */
int LSM6DSL_SetAccelODR(float odr_hz);

/**
 * Set accelerometer full scale range
 * @param g Full scale in g (2, 4, 8, or 16)
 * @return 0 on success, negative on error
 * 
 * Scale conversion factors:
 * ±2g:  16384 LSB/g
 * ±4g:  8192 LSB/g
 * ±8g:  4096 LSB/g
 * ±16g: 2048 LSB/g
 */
int LSM6DSL_SetAccelFullScale(int g);

// ==================== Gyroscope Functions ====================

/**
 * Read raw gyroscope data (X, Y, Z)
 * @param x Pointer to store X-axis value (int16_t)
 * @param y Pointer to store Y-axis value (int16_t)
 * @param z Pointer to store Z-axis value (int16_t)
 * @return 0 on success, negative on error
 * 
 * Conversion for ±245dps scale: value * 8.75 / 1000 = angular rate in dps
 */
int LSM6DSL_ReadGyroXYZ(int16_t *x, int16_t *y, int16_t *z);

/**
 * Set gyroscope output data rate
 * @param odr_hz Output data rate in Hz (common: 12.5, 26, 52, 104, 208, 416)
 * @return 0 on success, negative on error
 */
int LSM6DSL_SetGyroODR(float odr_hz);

/**
 * Set gyroscope full scale range
 * @param dps Full scale in degrees per second (245, 500, 1000, or 2000)
 * @return 0 on success, negative on error
 * 
 * Scale conversion factors:
 * ±245 dps:  8.75 mdps/LSB
 * ±500 dps:  17.50 mdps/LSB
 * ±1000 dps: 35.00 mdps/LSB
 * ±2000 dps: 70.00 mdps/LSB
 */
int LSM6DSL_SetGyroFullScale(int dps);

// ==================== Status Functions ====================

/**
 * Check if new data is available
 * @param accel_ready Pointer to store accelerometer data ready flag
 * @param gyro_ready Pointer to store gyroscope data ready flag
 * @return 0 on success, negative on error
 */
int LSM6DSL_IsDataReady(bool *accel_ready, bool *gyro_ready);

#ifdef __cplusplus
}
#endif

#endif // LSM6DSL_H