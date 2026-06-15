#include "lsm6dsl.h"

// -------------------- LSM6DSL I2C + Registers --------------------
static constexpr uint8_t LSM6DSL_ADDR_0 = 0x6A;
static constexpr uint8_t LSM6DSL_ADDR_1 = 0x6B;

// Registers
static constexpr uint8_t REG_WHO_AM_I   = 0x0F;
static constexpr uint8_t WHO_AM_I_VALUE = 0x6A;

static constexpr uint8_t REG_CTRL1_XL   = 0x10; // accel ODR/FS
static constexpr uint8_t REG_CTRL2_G    = 0x11; // gyro ODR/FS
static constexpr uint8_t REG_CTRL3_C    = 0x12; // BDU, IF_INC
static constexpr uint8_t REG_STATUS_REG = 0x1E; // Data ready status

static constexpr uint8_t REG_OUTX_L_G   = 0x22; // gyro data start (6 bytes)
static constexpr uint8_t REG_OUTX_L_XL  = 0x28; // accel data start (6 bytes)

// CTRL3_C bits
static constexpr uint8_t CTRL3_IF_INC = 0x04; // auto-increment address
static constexpr uint8_t CTRL3_BDU    = 0x40; // block data update

// STATUS_REG bits
static constexpr uint8_t STATUS_XLDA  = 0x01; // accel data available
static constexpr uint8_t STATUS_GDA   = 0x02; // gyro data available

// ODR encoding for accelerometer (CTRL1_XL[7:4])
static inline uint8_t odr_to_bits(float odr_hz) {
    if (odr_hz <= 0.0f)   return 0x00;
    if (odr_hz <= 12.5f)  return 0x10;
    if (odr_hz <= 26.0f)  return 0x20;
    if (odr_hz <= 52.0f)  return 0x30;
    if (odr_hz <= 104.0f) return 0x40;
    if (odr_hz <= 208.0f) return 0x50;
    if (odr_hz <= 416.0f) return 0x60;
    if (odr_hz <= 833.0f) return 0x70;
    if (odr_hz <= 1660.0f) return 0x80;
    if (odr_hz <= 3330.0f) return 0x90;
    return 0xA0; // 6660 Hz
}

// FS encoding for accelerometer (CTRL1_XL[3:2])
static inline uint8_t accel_fs_to_bits(int g) {
    switch (g) {
        case 2:  return 0x00; // ±2g
        case 16: return 0x04; // ±16g
        case 4:  return 0x08; // ±4g
        case 8:  return 0x0C; // ±8g
        default: return 0x00;
    }
}

// FS encoding for gyroscope (CTRL2_G[3:2])
static inline uint8_t gyro_fs_to_bits(int dps) {
    switch (dps) {
        case 245:  return 0x00; // ±245 dps
        case 500:  return 0x04; // ±500 dps
        case 1000: return 0x08; // ±1000 dps
        case 2000: return 0x0C; // ±2000 dps
        default:   return 0x00;
    }
}

// -------------------- Driver State --------------------
static I2C *g_i2c = nullptr;
static uint8_t g_addr = 0;

// I2C helper functions
static inline int i2c_write_reg(uint8_t reg, uint8_t val) {
    if (!g_i2c || g_addr == 0) return -1;
    char data[2] = {static_cast<char>(reg), static_cast<char>(val)};
    int rc = g_i2c->write(static_cast<int>(g_addr << 1), data, 2, false);
    return (rc == 0) ? 0 : -2;
}

static inline int i2c_read_regs(uint8_t reg, uint8_t *buf, int len) {
    if (!g_i2c || g_addr == 0) return -1;
    char r = static_cast<char>(reg);
    int rc = g_i2c->write(static_cast<int>(g_addr << 1), &r, 1, true);
    if (rc != 0) return -2;
    rc = g_i2c->read(static_cast<int>(g_addr << 1), reinterpret_cast<char*>(buf), len, false);
    return (rc == 0) ? 0 : -3;
}

static int detect_address(uint8_t *whoami_out) {
    uint8_t who = 0;

    g_addr = LSM6DSL_ADDR_0;
    if (i2c_read_regs(REG_WHO_AM_I, &who, 1) == 0 && who == WHO_AM_I_VALUE) {
        if (whoami_out) *whoami_out = who;
        return 0;
    }

    g_addr = LSM6DSL_ADDR_1;
    if (i2c_read_regs(REG_WHO_AM_I, &who, 1) == 0 && who == WHO_AM_I_VALUE) {
        if (whoami_out) *whoami_out = who;
        return 0;
    }

    g_addr = 0;
    return -10;
}

// -------------------- Public API --------------------
int LSM6DSL_Init(I2C *i2c) {
    g_i2c = i2c;
    if (!g_i2c) return -1;

    // Set I2C to 400kHz (fast mode)
    g_i2c->frequency(400000);

    // Detect sensor address
    uint8_t who = 0;
    int rc = detect_address(&who);
    if (rc != 0) return rc;

    // Enable auto-increment + BDU for stable multi-byte reads
    rc = i2c_write_reg(REG_CTRL3_C, static_cast<uint8_t>(CTRL3_IF_INC | CTRL3_BDU));
    if (rc != 0) return rc;

    // Configure accelerometer: 52Hz, ±2g
    uint8_t ctrl1 = static_cast<uint8_t>(odr_to_bits(52.0f) | accel_fs_to_bits(2));
    rc = i2c_write_reg(REG_CTRL1_XL, ctrl1);
    if (rc != 0) return rc;

    // Configure gyroscope: 52Hz, ±245dps
    uint8_t ctrl2 = static_cast<uint8_t>(odr_to_bits(52.0f) | gyro_fs_to_bits(245));
    rc = i2c_write_reg(REG_CTRL2_G, ctrl2);
    if (rc != 0) return rc;

    // Wait for sensor to stabilize
    wait_us(20000);
    return 0;
}

int LSM6DSL_ReadWhoAmI(uint8_t *whoami) {
    if (!whoami) return -1;
    uint8_t who = 0;
    int rc = i2c_read_regs(REG_WHO_AM_I, &who, 1);
    if (rc != 0) return rc;
    *whoami = who;
    return 0;
}

int LSM6DSL_SetAccelODR(float odr_hz) {
    uint8_t ctrl1 = 0;
    int rc = i2c_read_regs(REG_CTRL1_XL, &ctrl1, 1);
    if (rc != 0) return rc;

    ctrl1 = static_cast<uint8_t>((ctrl1 & 0x0F) | odr_to_bits(odr_hz));
    return i2c_write_reg(REG_CTRL1_XL, ctrl1);
}

int LSM6DSL_SetAccelFullScale(int g) {
    uint8_t ctrl1 = 0;
    int rc = i2c_read_regs(REG_CTRL1_XL, &ctrl1, 1);
    if (rc != 0) return rc;

    ctrl1 = static_cast<uint8_t>((ctrl1 & 0xF3) | accel_fs_to_bits(g));
    return i2c_write_reg(REG_CTRL1_XL, ctrl1);
}

int LSM6DSL_SetGyroODR(float odr_hz) {
    uint8_t ctrl2 = 0;
    int rc = i2c_read_regs(REG_CTRL2_G, &ctrl2, 1);
    if (rc != 0) return rc;

    ctrl2 = static_cast<uint8_t>((ctrl2 & 0x0F) | odr_to_bits(odr_hz));
    return i2c_write_reg(REG_CTRL2_G, ctrl2);
}

int LSM6DSL_SetGyroFullScale(int dps) {
    uint8_t ctrl2 = 0;
    int rc = i2c_read_regs(REG_CTRL2_G, &ctrl2, 1);
    if (rc != 0) return rc;

    ctrl2 = static_cast<uint8_t>((ctrl2 & 0xF3) | gyro_fs_to_bits(dps));
    return i2c_write_reg(REG_CTRL2_G, ctrl2);
}

int LSM6DSL_ReadAccelXYZ(int16_t *x, int16_t *y, int16_t *z) {
    if (!x || !y || !z) return -1;

    uint8_t buf[6] = {0};
    int rc = i2c_read_regs(REG_OUTX_L_XL, buf, 6);
    if (rc != 0) return rc;

    // Little-endian: L then H
    *x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
    *y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
    *z = static_cast<int16_t>((buf[5] << 8) | buf[4]);

    return 0;
}

int LSM6DSL_ReadGyroXYZ(int16_t *x, int16_t *y, int16_t *z) {
    if (!x || !y || !z) return -1;

    uint8_t buf[6] = {0};
    int rc = i2c_read_regs(REG_OUTX_L_G, buf, 6);
    if (rc != 0) return rc;

    // Little-endian: L then H
    *x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
    *y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
    *z = static_cast<int16_t>((buf[5] << 8) | buf[4]);

    return 0;
}

int LSM6DSL_IsDataReady(bool *accel_ready, bool *gyro_ready) {
    if (!accel_ready || !gyro_ready) return -1;

    uint8_t status = 0;
    int rc = i2c_read_regs(REG_STATUS_REG, &status, 1);
    if (rc != 0) return rc;

    *accel_ready = (status & STATUS_XLDA) != 0;
    *gyro_ready = (status & STATUS_GDA) != 0;

    return 0;
}