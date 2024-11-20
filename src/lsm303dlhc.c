/// \file lsm303dlhc.c
/// \brief This file is part of LSM303DLHC Library for STM32 Nucleo L4
/// \copyright &copy; https://github.com/Ilushenko Oleksandr Ilushenko
///	\author Oleksandr Ilushenko
///	\date 2024
#include "lsm303dlhc.h"
#include "log.h"

enum {
    LSM303_LA_SAD = 0b00110010,  // Linear accelerometer address SAD+W
    LSM303_MF_SAD = 0b00111100   // Magnetic field address SAD+W
};

// Accelerometer registers
enum {
	LSM303_CTRL_REG1_A     = 0x20,
	LSM303_CTRL_REG2_A     = 0x21,
	LSM303_CTRL_REG3_A     = 0x22,
	LSM303_CTRL_REG4_A     = 0x23,
	LSM303_CTRL_REG5_A     = 0x24,
	LSM303_CTRL_REG6_A     = 0x25,
	LSM303_REFERENCE_A     = 0x26,
	LSM303_STATUS_REG_A    = 0x27,
	LSM303_OUT_X_L_A       = 0x28,
	LSM303_OUT_X_H_A       = 0x29,
	LSM303_OUT_Y_L_A       = 0x2a,
	LSM303_OUT_Y_H_A       = 0x2b,
	LSM303_OUT_Z_L_A       = 0x2c,
	LSM303_OUT_Z_H_A       = 0x2d,
	LSM303_FIFO_CTRL_REG_A = 0x2e,
	LSM303_FIFO_SRC_REG_A  = 0x2f,
	LSM303_INT1_CFG_A      = 0x30,
	LSM303_INT1_SRC_A      = 0x31,
	LSM303_INT1_THS_A      = 0x32,
	LSM303_INT1_DURATION_A = 0x33,
	LSM303_INT2_CFG_A      = 0x34,
	LSM303_INT2_SRE_A      = 0x35,
	LSM303_INT2_THS_A      = 0x36,
	LSM303_INT2_DURATION_A = 0x37,
	LSM303_CLICK_CFG_A     = 0x38,
	LSM303_CLICK_SRC_A     = 0x39,
	LSM303_CLICK_THS_A     = 0x3a,
	LSM303_TIME_LIMIT_A    = 0x3b,
	LSM303_TIME_LATENCY_A  = 0x3c,
	LSM303_TIME_WINDOW_A   = 0x3d
};

// Magnetometer registers
enum {
    LSM303_CRA_REG_M   = 0x00,
    LSM303_CRB_REG_M   = 0x01,
    LSM303_MR_REG_M    = 0x02,
    LSM303_OUT_X_H_M   = 0x03,
    LSM303_OUT_X_L_M   = 0x04,
    LSM303_OUT_Z_H_M   = 0x05,
    LSM303_OUT_Z_L_M   = 0x06,
    LSM303_OUT_Y_H_M   = 0x07,
    LSM303_OUT_Y_L_M   = 0x08,
    LSM303_SR_REG_M    = 0x09,
    LSM303_IRA_REG_M   = 0x0A,
    LSM303_IRB_REG_M   = 0x0B,
    LSM303_IRC_REG_M   = 0x0C,
};

// CTRL_REG1_A
typedef union {
    struct __attribute__((__packed__)) {
        uint8_t x           : 1; // (0: X-axis disabled, 1: X-axis enabled)
        uint8_t y           : 1; // (0: Y-axis disabled, 1: Y-axis enabled)
        uint8_t z           : 1; // (0: Z-axis disabled, 1: Z-axis enabled)
        uint8_t lowPower    : 1; // (0: normal mode, 1: low-power mode)
        uint8_t dataRate    : 4; // lsm303_la_datarate_t
    };
    uint8_t reg;
} lsm303_reg_ctrl_a1_t;

// CTRL_REG2_A
typedef union {
    struct __attribute__((__packed__)) {
        uint8_t is1     : 1; // High-pass filter enabled for AOI function on Interrupt 1 (0: filter bypassed, 1: filter enabled)
        uint8_t is2     : 1; // High-pass filter enabled for AOI function on Interrupt 2 (0: filter bypassed, 1: filter enabled)
        uint8_t click   : 1; // High-pass filter enabled for click function (0: filter bypassed, 1: filter enabled)
        uint8_t fds     : 1; // Filtered data selection. Default value: 0 (0: internal filter bypassed, 1: data from internal filter sent to output register and FIFO)
        uint8_t cutoff  : 2; // High-pass filter cutoff frequency selection
        uint8_t mode    : 2; // lsm303_la_hp_t. High-pass filter mode selection. Default value: 00
    };
    uint8_t reg;
} lsm303_reg_ctrl_a2_t;

// CTRL_REG3_A
typedef union {
    struct __attribute__((__packed__)) {
        uint8_t reserverd   : 1;
        uint8_t overrun     : 1; // FIFO overrun interrupt on INT1. Default value 0 (0: disable, 1: enable)
        uint8_t wtm         : 1; // FIFO watermark interrupt on INT1. Default value 0 (0: disable, 1: enable)
        uint8_t drdy2       : 1; // DRDY2 interrupt on INT1. Default value 0 (0: disable, 1: enable)
        uint8_t drdy1       : 1; // DRDY1 interrupt on INT1. Default value 0 (0: disable, 1: enable)
        uint8_t aoi2        : 1; // AOI2 interrupt on INT1. Default value 0 (0: disable, 1: enable)
        uint8_t aoi1        : 1; // AOI1 interrupt on INT1. Default value 0 (0: disable, 1: enable)
        uint8_t click       : 1; // CLICK interrupt on INT1. Default value 0 (0: disable, 1: enable)
    };
    uint8_t reg;
} lsm303_reg_ctrl_a3_t;

// CTRL_REG4_A
typedef union {
    struct __attribute__((__packed__)) {
        uint8_t sim         : 1; // SPI serial interface mode selection. Default value: 0 (0: 4-wire interface, 1: 3-wire interface).
        uint8_t reserved    : 2;
        uint8_t hr          : 1; // High-resolution output mode: Default value: 0 (0: high-resolution disable, 1: high-resolution enable)
        uint8_t fs          : 2; // lsm303_la_fs_t. Full-scale selection. Default value: 00
        uint8_t ble         : 1; // Big/little endian data selection. Default value 0. (0: data LSB @ lower address, 1: data MSB @ lower address)
        uint8_t bdu         : 1; // Block data update. Default value: 0 (0: continuous update, 1: output registers not updated until MSB and LSB have been read
    };
    uint8_t reg;
} lsm303_reg_ctrl_a4_t;

typedef union {
    struct __attribute__((__packed__)) {
        uint8_t reserved1   : 1;
        uint8_t h_lactive   : 1; // Interrupt active high, low (0: active high, 1: active low)
        uint8_t reserved2   : 1;
        uint8_t p2_act      : 1; // Active function status on PAD2 (0: disable, 1: enable)
        uint8_t boot_l1     : 1; // Reboot memory content on PAD2 (0: disable, 1: enable)
        uint8_t i2_int2     : 1; // Interrupt 2 on PAD2 (0: disable, 1: enable)
        uint8_t i2_int1     : 1; // Interrupt 1 on PAD2 (0: disable, 1: enable)
        uint8_t i2_clicken  : 1; // CLICK interrupt on PAD2 (0: disable, 1: enable)
    };
    uint8_t reg;    
} lsm303_reg_ctrl_a6_t;

// CRA_REG_M
typedef union {
    struct __attribute__((__packed__)) {
        uint8_t reserved0   : 2;
        uint8_t dataRate    : 3; // lsm303_mf_do_t
        uint8_t reserved1   : 2;
        uint8_t temperature : 1; // 0: temperature sensor disabled (default), 1: temperature sensor enabled
    };
    uint8_t reg;
} lsm303_reg_cra_t;

// CRB_REG_M
typedef union {
    struct __attribute__((__packed__)) {
        uint8_t reserverd   : 5;
        uint8_t gain        : 3; // lsm303_mf_gain_t
    };
    uint8_t reg;
} lsm303_reg_crb_t;

// MR_REG_M
typedef union {
    struct __attribute__((__packed__)) {
        uint8_t mode        : 2; // lsm303_mf_md_t
        uint8_t reserverd   : 6;
    };
    uint8_t reg;
} lsm303_reg_mr_t;

uint8_t lsm303_buf[6] = { 0 };  // buffer for read lsm303 data
uint8_t lsm303_ashift = 0;      // accelerometer bit shift amount
float lsm303_alsb = 0.0F;       // accelerometer sensitivity mg/LSB
float lsm303_mlsb_xy = 0.0F;    // magnetometer LSB/Gauss for X, Y
float lsm303_mlsb_z = 0.0F;     // magnetometer LSB/Gauss for Z

uint8_t lsm303_la_setup(I2C_HandleTypeDef *i2c, const lsm303_la_datarate_t odr, const uint8_t lpe, const uint8_t hr, const lsm303_la_fs_t fs)
{
    if (0 == i2c) return HAL_ERROR;

    uint8_t data[2] = { 0 };
    lsm303_reg_ctrl_a1_t a1 = { 0 };
    lsm303_reg_ctrl_a4_t a4 = { 0 };

    a1.dataRate = odr;
    a1.lowPower = lpe == 0U ? 0U : 1U;
    a1.x = 1U;
    a1.y = 1U;
    a1.z = 1U;

    a4.hr = hr == 0U ? 0U : 1U;
    a4.fs = fs;
   
    data[0] = LSM303_CTRL_REG1_A;
    data[1] = a1.reg;
    uint8_t ret = HAL_I2C_Master_Transmit(i2c, LSM303_LA_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("CTRL_REG1_A: 0x%02x %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    data[0] = LSM303_CTRL_REG4_A;
    data[1] = a4.reg;
    ret = HAL_I2C_Master_Transmit(i2c, LSM303_LA_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("CTRL_REG4_A: 0x%02x %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    if (hr == 0U) {
        // Normal : Low-power mode
        lsm303_ashift = lpe == 0U ? 6U : 8U;
        switch (fs) {
        case LSM303_AFS_2G:
            lsm303_alsb = lpe == 0U ? 0.0039 : 0.01563;
            break;
        case LSM303_AFS_4G:
            lsm303_alsb = lpe == 0U ? 0.00782 : 0.03126;
            break;
        case LSM303_AFS_8G:
            lsm303_alsb = lpe == 0U ? 0.01563 : 0.06252;
            break;
        case LSM303_AFS_16G:
            lsm303_alsb = lpe == 0U ? 0.0469 : 0.18758;
            break;
        }
    }
    else {
        // High-resolution
        lsm303_ashift = 4U;
        switch (fs) {
        case LSM303_AFS_2G:
            lsm303_alsb = 0.00098;
            break;
        case LSM303_AFS_4G:
            lsm303_alsb = 0.00195;
            break;
        case LSM303_AFS_8G:
            lsm303_alsb = 0.0039;
            break;
        case LSM303_AFS_16G:
            lsm303_alsb = 0.01172;
            break;
        }
    }
    return HAL_OK;
}

uint8_t lsm303_la_int1(I2C_HandleTypeDef *i2c, const uint8_t cfg, uint8_t threshould, uint8_t duration)
{
    if (0 == i2c) return HAL_ERROR;

    uint8_t data[2] = { 0 };
    lsm303_reg_ctrl_a3_t r = { 0 };

    if (cfg == 0U) {
        threshould = 0U;
        duration = 0U;
    }
    else {
        if (threshould > 0x7F) threshould = 0x7F;
        if (duration > 0x7F) duration = 0x7F;
        r.aoi1 = 1U;
    }

    // Configure INT1
    data[0] = LSM303_INT1_CFG_A;
    data[1] = cfg;
    uint8_t ret = HAL_I2C_Master_Transmit(i2c, LSM303_LA_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("INT1_CFG_A: 0x%02X %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    // Threshould
    data[0] = LSM303_INT1_THS_A;
    data[1] = threshould;
    ret = HAL_I2C_Master_Transmit(i2c, LSM303_LA_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("INT1_THS_A: 0x%02X %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    // Duration
    data[0] = LSM303_INT1_DURATION_A;
    data[1] = duration;
    ret = HAL_I2C_Master_Transmit(i2c, LSM303_LA_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("INT1_DURATION_A: 0x%02X %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    // Activate IRQ to INT1 output
    data[0] = LSM303_CTRL_REG3_A;
    data[1] = r.reg;
    ret = HAL_I2C_Master_Transmit(i2c, LSM303_LA_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("CTRL_REG3_A: 0x%02X %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );
    return HAL_OK;
}

uint8_t lsm303_la_src1(I2C_HandleTypeDef *i2c, uint8_t *src)
{
    return HAL_I2C_Mem_Read(i2c, LSM303_LA_SAD, LSM303_INT1_SRC_A, I2C_MEMADD_SIZE_8BIT, src, sizeof(uint8_t), HAL_MAX_DELAY);
}

uint8_t lsm303_la_raw(I2C_HandleTypeDef *i2c, int16_t *x, int16_t *y, int16_t *z)
{
    if (0 == i2c) return HAL_ERROR;
    // Read status
    uint8_t ret = HAL_I2C_Mem_Read(i2c, LSM303_LA_SAD, LSM303_STATUS_REG_A, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(uint8_t), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("STATUS_REG_A Read Error!\n");
        return ret;
    }
    // Check data available
    if ((lsm303_buf[0] >> 3 & 1) == 0U) {
        xWarning("Accelerometer data unavailable!\n");
        return HAL_BUSY;
    }
    // Read
    ret = HAL_I2C_Mem_Read(i2c, LSM303_LA_SAD, LSM303_OUT_X_L_A | 0b10000000, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(lsm303_buf), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("OUT_X_L_A Read Error!\n");
        return ret;
    }
    // Conversion
    *x = (int16_t)(lsm303_buf[1] << 8 | lsm303_buf[0]) >> lsm303_ashift;
    *y = (int16_t)(lsm303_buf[3] << 8 | lsm303_buf[2]) >> lsm303_ashift;
    *z = (int16_t)(lsm303_buf[5] << 8 | lsm303_buf[4]) >> lsm303_ashift;
    return HAL_OK;
}

uint8_t lsm303_la_read(I2C_HandleTypeDef *i2c, float* x, float* y, float* z)
{
    if (0 == i2c) return HAL_ERROR;
    // Read status
    uint8_t ret = HAL_I2C_Mem_Read(i2c, LSM303_LA_SAD, LSM303_STATUS_REG_A, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(uint8_t), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("STATUS_REG_A Read Error!\n");
        return ret;
    }
    // Check data available
    if ((lsm303_buf[0] >> 3 & 1) == 0U) {
        //xWarning("Accelerometer data unavailable!\n");
        return HAL_BUSY;
    }
    // Read
    ret = HAL_I2C_Mem_Read(i2c, LSM303_LA_SAD, LSM303_OUT_X_L_A | 0b10000000, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(lsm303_buf), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("OUT_X_L_A Read Error!\n");
        return ret;
    }
    // Conversion
    *x = (float)((int16_t)(lsm303_buf[1] << 8 | lsm303_buf[0]) >> lsm303_ashift) * lsm303_alsb;
    *y = (float)((int16_t)(lsm303_buf[3] << 8 | lsm303_buf[2]) >> lsm303_ashift) * lsm303_alsb;
    *z = (float)((int16_t)(lsm303_buf[5] << 8 | lsm303_buf[4]) >> lsm303_ashift) * lsm303_alsb;
    return HAL_OK;
}

uint8_t lsm303_mf_setup(I2C_HandleTypeDef *i2c, const uint8_t ten, const lsm303_mf_do_t odr, const lsm303_mf_gain_t gn, const lsm303_mf_md_t md)
{
    if (0 == i2c) return HAL_ERROR;

    uint8_t data[2] = { 0 };
    lsm303_reg_cra_t a = { 0 };
    lsm303_reg_crb_t b = { 0 };
    lsm303_reg_mr_t r = { 0 };

    a.temperature = ten == 0U ? 0U : 1U;
    a.dataRate = odr;
    b.gain = gn;
    r.mode = md;

    data[0] = LSM303_CRA_REG_M;
    data[1] = a.reg;
    uint8_t ret = HAL_I2C_Master_Transmit(i2c, LSM303_MF_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("CRA_REG_M: 0x%02x %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    data[0] = LSM303_CRB_REG_M;
    data[1] = b.reg;
    ret = HAL_I2C_Master_Transmit(i2c, LSM303_MF_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("CRB_REG_M: 0x%02x %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    data[0] = LSM303_MR_REG_M;
    data[1] = r.reg;
    ret = HAL_I2C_Master_Transmit(i2c, LSM303_MF_SAD, &data[0], sizeof(data), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    xDebug("MR_REG_M: 0x%02x %u%u%u%u%u%u%u%u\n",
        data[1],
        data[1] >> 7 & 1,
        data[1] >> 6 & 1,
        data[1] >> 5 & 1,
        data[1] >> 4 & 1,
        data[1] >> 3 & 1,
        data[1] >> 2 & 1,
        data[1] >> 1 & 1,
        data[1] & 1
    );

    switch (gn) {
    case LSM303_MGAIN_1_3:
        lsm303_mlsb_xy = 1100.0F;
        lsm303_mlsb_z = 980.0F;
        break;
    case LSM303_MGAIN_1_9:
        lsm303_mlsb_xy = 855.0F;
        lsm303_mlsb_z = 760.0F;
        break;
    case LSM303_MGAIN_2_5:
        lsm303_mlsb_xy = 670.0F;
        lsm303_mlsb_z = 600.0F;
        break;
    case LSM303_MGAIN_4_0:
        lsm303_mlsb_xy = 450.0F;
        lsm303_mlsb_z = 400.0F;
        break;
    case LSM303_MGAIN_4_7:
        lsm303_mlsb_xy = 400.0F;
        lsm303_mlsb_z = 355.0F;
        break;
    case LSM303_MGAIN_5_6:
        lsm303_mlsb_xy = 330.0F;
        lsm303_mlsb_z = 295.0F;
        break;
    case LSM303_MGAIN_8_1:
        lsm303_mlsb_xy = 230.0F;
        lsm303_mlsb_z = 205.0F;
        break;
    }
    return HAL_OK;
}

uint8_t lsm303_mf_raw(I2C_HandleTypeDef* i2c, int16_t* x, int16_t* y, int16_t* z)
{
    if (0 == i2c) return HAL_ERROR;
    // Read status
    uint8_t ret = HAL_I2C_Mem_Read(i2c, LSM303_MF_SAD, LSM303_SR_REG_M, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(uint8_t), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("SR_REG_M Read Error!\n");
        return ret;
    }
    // Check is data ready (0 bit in register)
    if ((lsm303_buf[0] & 1) == 0) {
        xWarning("Magnetometer data not ready\n");
        return HAL_BUSY;
    }
    // Read data
    ret = HAL_I2C_Mem_Read(i2c, LSM303_MF_SAD, LSM303_OUT_X_H_M, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(lsm303_buf), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("OUT_X_H_M Read Error!\n");
        return ret;
    }
    // Conversion
    *x = (int16_t)(lsm303_buf[0] << 8 | lsm303_buf[1]);
    *y = (int16_t)(lsm303_buf[4] << 8 | lsm303_buf[5]);
    *z = (int16_t)(lsm303_buf[2] << 8 | lsm303_buf[3]);
    return HAL_OK;
}

uint8_t lsm303_mf_read(I2C_HandleTypeDef *i2c, float* x, float* y, float* z)
{
    if (0 == i2c) return HAL_ERROR;
    // Read status
    uint8_t ret = HAL_I2C_Mem_Read(i2c, LSM303_MF_SAD, LSM303_SR_REG_M, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(uint8_t), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("SR_REG_M Read Error!\n");
        return ret;
    }
    // Check is data ready (0 bit in register)
    if ((lsm303_buf[0] & 1) == 0) {
        //xWarning("Magnetometer data not ready\n");
        return HAL_BUSY;
    }
    // Read data
    ret = HAL_I2C_Mem_Read(i2c, LSM303_MF_SAD, LSM303_OUT_X_H_M, I2C_MEMADD_SIZE_8BIT, &lsm303_buf[0], sizeof(lsm303_buf), HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        xWarning("OUT_X_H_M Read Error!\n");
        return ret;
    }
    // Conversion
    *x = (float)((int16_t)(lsm303_buf[0] << 8 | lsm303_buf[1])) / lsm303_mlsb_xy * 100.0F;
    *y = (float)((int16_t)(lsm303_buf[4] << 8 | lsm303_buf[5])) / lsm303_mlsb_xy * 100.0F;
    *z = (float)((int16_t)(lsm303_buf[2] << 8 | lsm303_buf[3])) / lsm303_mlsb_z * 100.0F;
    return HAL_OK;
}

// float motionLP(float x, float y, float z, const float alpha, const float delta, const uint8_t sample)
// {
//     static uint8_t setup = 0;
//     static uint8_t smpl = 0;
//     // preview data
//     static float pX = 0.0;
//     static float pY = 0.0;
//     static float pZ = 0.0;
//     // filtered data
//     static float fX = 0.0;
//     static float fY = 0.0;
//     static float fZ = 0.0;
//     // first iteration
//     if (setup == 0) {
//         fX = x;
//         fY = y;
//         fZ = z;
//         setup++;
//         return 0.0F;
//     }
//     // Low-pass filter
//     fX = alpha * x + (1.0F - alpha) * fX;
//     fY = alpha * y + (1.0F - alpha) * fY;
//     fZ = alpha * z + (1.0F - alpha) * fZ;
//     // Accumulation
//     if (setup < CNTSETUP) {
//         pX = fX;
//         pY = fY;
//         pZ = fZ;
//         setup++;
//         return 0.0F;
//     }
//     // Samples
//     if (smpl++ < sample) return 0.0;
//     smpl = 0;
//     // Delta
//     const float dX = fabsf(fX - pX);
//     const float dY = fabsf(fY - pY);
//     const float dZ = fabsf(fZ - pZ);
//     // Magnitude
//     const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
//     // if (dX > delta) xDebug("ACC %f, %f, %f\tX: %f M: %f\n", x, y, z, dX, M);
//     // if (dY > delta) xDebug("ACC %f, %f, %f\tY: %f M: %f\n", x, y, z, dY, M);
//     // if (dZ > delta) xDebug("ACC %f, %f, %f\tZ: %f M: %f\n", x, y, z, dZ, M);
//     if (m > delta) {
//         setup = 0;
//         xDebug("ACC %f, %f, %f\tD: %f\n", x, y, z, m);
//         return m;
//     }
//     return 0.0F;
// }

// float motionK(float x, float y, float z, const float Q, const float R, const float E, const float delta, const uint8_t sample)
// {
//     static uint8_t setup = 0;
//     static uint8_t smpl = 0;
//     // Prediction estimate (filtered data)
//     static float fX = 0.0;
//     static float fY = 0.0;
//     static float fZ = 0.0;
//     // Prediction error
//     static float eX = 0.0;
//     static float eY = 0.0;
//     static float eZ = 0.0;
//     // preview data
//     static float pX = 0.0;
//     static float pY = 0.0;
//     static float pZ = 0.0;
//     // first iteration
//     if (setup == 0) {
//         // Prediction estimate
//         fX = x;
//         fY = y;
//         fZ = z;
//         // Prediction error
//         eX = eY = eZ = E;
//         setup++;
//         //return 0.0F;
//     }
//     // Kalman filter X
//     eX += Q;                // Prediction of a new error
//     float K = eX / (eX + R);// Calculation of the Kalman coefficient
//     fX += K * (x - fX);     // Estimate update
//     eX *= (1.0 - K);        // Error update
//     // Kalman filter Y
//     eY += Q;                // Prediction of a new error
//     K = eY / (eY + R);      // Calculation of the Kalman coefficient
//     fY += K * (y - fY);     // Estimate update
//     eY *= (1.0 - K);        // Error update
//     // Kalman filter Z
//     eZ += Q;                // Prediction of a new error
//     K = eZ / (eZ + R);      // Calculation of the Kalman coefficient
//     fZ += K * (z - fZ);     // Estimate update
//     eZ *= (1.0 - K);        // Error update
//     //xTrace("%f, %f, %f, %f, %f, %f\n", x, y, z, fX + 10.0, fY + 10.0, fZ + 10.0);
//     // Accumulation
//     if (setup < CNTSETUP) {
//         pX = fX;
//         pY = fY;
//         pZ = fZ;
//         setup++;
//         return 0.0F;
//     }    
//     // Samples
//     if (smpl++ < sample) return 0.0;
//     smpl = 0;
//     // Delta
//     const float dX = fabsf(fX - pX);
//     const float dY = fabsf(fY - pY);
//     const float dZ = fabsf(fZ - pZ);
//     // Magnitude
//     const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
//     if (m > delta && m < delta * 1.5) {
//         setup = 0;
//         xDebug("ACC %f, %f, %f\tD: %f\n", x, y, z, m);
//         return m;
//     }
//     return 0.0F;



//     // // !
//     // // difference
//     // const float dX = x - fX;
//     // const float dY = y - fY;
//     // const float dZ = z - fZ;
//     // const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
//     // // check
//     // if (m > delta) {
//     //     xDebug("ACC %f, %f, %f\tD: %f\n", x, y, z, m);
//     //     setup = 0;
//     //     return m;
//     // }
//     // return 0.0F;
// }

// float distortionHP(float x, float y, float z, const float alpha, const float delta)
// {
//     static uint8_t setup = 0;
//     // prev input
//     static float iX = 0.0F;
//     static float iY = 0.0F;
//     static float iZ = 0.0F;
//     // prev output
//     static float oX = 0.0F;
//     static float oY = 0.0F;
//     static float oZ = 0.0F;
//     // prev magnitude
//     static float M = 0.0F;
//     // high-pass filter
//     oX = alpha * (oX + x - iX);
//     oY = alpha * (oY + y - iY);
//     oZ = alpha * (oZ + z - iZ);
//     iX = x;
//     iY = y;
//     iZ = z;
//     // difference
//     const float dX = iX - oX;
//     const float dY = iY - oY;
//     const float dZ = iZ - oZ;
//     // magnitude
//     const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
//     // average magnitude
//     if (setup == 0U) {
//         M = m;
//         setup++;
// 		return 0.0F;
//     }
//     // magnitude low-pass filter
//     if (setup < CNTSETUP) {
//         M = alpha * m + (1.0F - alpha) * M;
//         setup++;
//         return 0.0F;
//     }
//     // if (fabsf(dX) > delta) xDebug("MAG %f, %f, %f\tX: %f\n", x, y, z, dX);
//     // if (fabsf(dY) > delta) xDebug("MAG %f, %f, %f\tY: %f\n", x, y, z, dY);
//     // if (fabsf(dZ) > delta) xDebug("MAG %f, %f, %f\tZ: %f\n", x, y, z, dZ);
//     // check
//     const float d = fabsf(M - m);
//     if (d > delta) {
//         xDebug("MAG %f, %f, %f\tM: %f m: %f D: %f\n", x, y, z, M, m, d);
//         setup = 0;
//         return d;
//     }
//     return 0.0F;
// }

// float distortionLP(float x, float y, float z, const float alpha, const float delta)
// {
//     static uint8_t setup = 0;
//     // average
//     static float aX = 0.0F;
//     static float aY = 0.0F;
//     static float aZ = 0.0F;
//     // first iteration
//     if (setup == 0) {
//         aX = x;
//         aY = y;
//         aZ = z;
//         setup++;
//         return 0.0F;
//     }
//     // difference
//     const float dX = x - aX;
//     const float dY = y - aY;
//     const float dZ = z - aZ;
//     // low-pass
//     aX = alpha * x + (1.0F - alpha) * aX;
//     aY = alpha * y + (1.0F - alpha) * aY;
//     aZ = alpha * z + (1.0F - alpha) * aZ;
//     // aX += (x - aX) * alpha;
//     // aY += (y - aY) * alpha;
//     // aZ += (z - aZ) * alpha;
//     // accumulate
//     if (setup < CNTSETUP) {
//         setup++;
//         return 0.0F;
//     }
//     // magnitude
//     const float m = sqrtf(dX * dX + dY * dY + dZ * dZ);
//     if (m > delta) {
//         xDebug("MAG %f, %f, %f\tD: %f\n", x, y, z, m);
//         setup = 0;
//         return m;
//     }
//     return 0.0F;
// }

// uint8_t orientLP(const float a[3], const float m[3], const float alpha, float* pitch, float* roll, float* yaw)
// {
//     enum { X, Y, Z };
//     static uint8_t setup = 0U;
//     // filtered data
//     static float aX = 0.0;
//     static float aY = 0.0;
//     static float aZ = 0.0;
//     static float mX = 0.0;
//     static float mY = 0.0;
//     static float mZ = 0.0;
//     // first iteration
//     if (setup == 0U) {
//         aX = a[X];
//         aY = a[Y];
//         aZ = a[Z];
//         mX = m[X];
//         mY = m[Y];
//         mZ = m[Z];
//         setup++;
//         return 1U;
//     }
//     // Low-pass filter
//     aX = alpha * a[X] + (1.0F - alpha) * aX;
//     aY = alpha * a[Y] + (1.0F - alpha) * aY;
//     aZ = alpha * a[Z] + (1.0F - alpha) * aZ;
//     mX = alpha * m[X] + (1.0F - alpha) * mX;
//     mY = alpha * m[Y] + (1.0F - alpha) * mY;
//     mZ = alpha * m[Z] + (1.0F - alpha) * mZ;
//     // accumulate
//     if (setup < CNTSETUP) {
//         setup++;
//         return 1U;
//     }
//     // pitch & roll
//     *pitch = atan2f(aX, sqrtf(aY * aY + aZ * aZ)) * RAD2DEG;
//     *roll = atan2f(aY, sqrtf(aX * aX + aZ * aZ)) * RAD2DEG;
//     // normalize accelerometer
//     float N = sqrtf(aX * aX + aY * aY + aZ * aZ);
//     aX /= N;
//     aY /= N;
//     aZ /= N;
//     // normalize magnetometer
//     N = sqrtf(mX * mX + mY * mY + mZ * mZ);
//     mX /= N;
//     mY /= N;
//     mZ /= N;
//     // magnetic field horizontal projection
//     const float Mx = mX * aZ - mZ * aX;
//     const float My = mY * aZ - mZ * aY;
//     // yaw
//     *yaw = atan2f(My, Mx) * RAD2DEG;
//     xDebug("Pitch: %f, Roll: %f, Yaw: %f\n", *pitch, *roll, *yaw);
//     return 0U;
// }

// float inclineLP(float x, float y, float z, const float alpha, const float delta)
// {
//     static uint8_t setup = 0U;
//     // filtered data
//     static float aX = 0.0;
//     static float aY = 0.0;
//     static float aZ = 0.0;
//     // first iteration
//     if (setup == 0U) {
//         aX = x;
//         aY = y;
//         aZ = z;
//         setup++;
//         return 0.0F;
//     }
//     // Low-pass filter
//     aX = alpha * x + (1.0F - alpha) * aX;
//     aY = alpha * y + (1.0F - alpha) * aY;
//     aZ = alpha * z + (1.0F - alpha) * aZ;
//     // accumulate
//     if (setup < CNTSETUP) {
//         setup++;
//         return 0.0;
//     }
//     // angle
//     const float theta = acosf(aZ / sqrtf(aX * aX + aY *aY + aZ * aZ)) * RAD2DEG;
//     if (theta > fabsf(delta)) {
//         xDebug("INCL %f, %f, %f\tD: %f\n", x, y, z, theta);
//         setup = 0;
//         return theta;
//     }
//     return 0.0F;
// }

// float getAlpha(const float rate, const float cutoff)
// {
//     const float rc = 1.0 / (2.0 * M_PI * cutoff);
//     const float dt = 1.0 / rate;
//     return dt / (rc + dt);
// }
