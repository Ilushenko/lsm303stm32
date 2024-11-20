/// \file lsm303dlhc.h
/// \brief This file is part of LSM303DLHC Library for STM32 Nucleo L4
/// \copyright &copy; https://github.com/Ilushenko Oleksandr Ilushenko
///	\author Oleksandr Ilushenko
///	\date 2024
#ifndef __LSM303_H__
#define __LSM303_H__

#include "stm32l4xx_hal.h"

/// \defgroup lsm303data 1. LSM303 Parameters
/// \brief LSM303 Enumerators, variables, structures

/// \defgroup lsm303func 2. LSM303 Functions
/// \brief LSM303 Setup and read sensor functions

/// \brief Linear accelerometer sensitivity \a mg/LSB
/// \details This variable is initialized in the function \b lsm303_la_setup
/// \details Depends on the parameters: full-scale selection, enable/disable low-power mode, enable/disable high-resolution output mode
/// \ingroup lsm303data
extern float lsm303_alsb;

/// \brief Magnetic field LSB/Gauss for \c X, \c Y axis \a LSB/Gauss
/// \details This variable is initialized in the function \b lsm303_mf_setup
/// \details Depends on the gain setting
/// \ingroup lsm303data
extern float lsm303_mlsb_xy;

/// \brief Magnetic field LSB/Gauss for \c Z axis \a LSB/Gauss
/// \details This variable is initialized in the function \b lsm303_mf_setup
/// \details Depends on the gain setting
/// \ingroup lsm303data
extern float lsm303_mlsb_z;

/// \brief Linear accelerometer data rate
/// \details \c CTRL_REG1_A register field
/// \ingroup lsm303data
typedef enum {
    LSM303_ADATARATE_DOWN  = 0b0000, ///< Power-down mode
    LSM303_ADATARATE_1     = 0b0001, ///< Normal / low-power mode (1 Hz)
    LSM303_ADATARATE_10    = 0b0010, ///< Normal / low-power mode (10 Hz)
    LSM303_ADATARATE_25    = 0b0011, ///< Normal / low-power mode (25 Hz)
    LSM303_ADATARATE_50    = 0b0100, ///< Normal / low-power mode (50 Hz)
    LSM303_ADATARATE_100   = 0b0101, ///< Normal / low-power mode (100 Hz)
    LSM303_ADATARATE_200   = 0b0110, ///< Normal / low-power mode (200 Hz)
    LSM303_ADATARATE_400   = 0b0111, ///< Normal / low-power mode (400 Hz)
    LSM303_ADATARATE_LOW   = 0b1000, ///< Low-power mode (1.620 kHz)
    LSM303_ADATARATE_SPEC  = 0b1001  ///< Normal (1.344 kHz) / low-power mode (5.376 kHz)
} lsm303_la_datarate_t;

/// \brief Linear accelerometer high-pass filter mode
/// \details \c CTRL_REG2_A register field
/// \ingroup lsm303data
typedef enum {
    LSM303_AHP_RESET   = 0b00, ///< Normal mode (reset reading HP_RESET_FILTER)
    LSM303_AHP_RSIGNAL = 0b01, ///< Reference signal for filtering
    LSM303_AHP_NORMAL  = 0b10, ///< Normal mode
    LSM303_AHP_IEVENT  = 0b11  ///< Autoreset on interrupt event
} lsm303_la_hp_t;

/// \brief Linear accelerometer full-scale selection
/// \details \c CTRL_REG4_A register field
/// \ingroup lsm303data
typedef enum {
    LSM303_AFS_2G     = 0b00, ///< ±2 g
    LSM303_AFS_4G     = 0b01, ///< ±4 g
    LSM303_AFS_8G     = 0b10, ///< ±8 g
    LSM303_AFS_16G    = 0b11  ///< ±16 g
} lsm303_la_fs_t;

/// \brief Linear accelerometer interrupt mode
/// \details \c INT1_CFG_A and \c INT2_CFG_A registers field
/// \ingroup lsm303data
typedef enum {
    LSM303_AOR      = 0b00, ///< OR combination of interrupt events
    LSM303_AOR_6D   = 0b01, ///< 6-direction movement recognition (when the orientation moves from an unknown zone to a known zone)
    LSM303_AAND     = 0b10, ///< AND combination of interrupt events
    LSM303_AAND_6D  = 0b11  ///< 6-direction position recognition (when the orientation is inside a known zone)
} lsm303_la_irq_mode_t;

/// \brief Magnetic field data rate
/// \details \c CRA_REG_M register field
/// \ingroup lsm303data
typedef enum {
    LSM303_MDATARATE_0_75  = 0b000, ///< 0.75 Hz
    LSM303_MDATARATE_1_5   = 0b001, ///< 1.5 Hz
    LSM303_MDATARATE_3_0   = 0b010, ///< 3.0 Hz
    LSM303_MDATARATE_7_5   = 0b011, ///< 7.5 Hz
    LSM303_MDATARATE_15    = 0b100, ///< 15 Hz
    LSM303_MDATARATE_30    = 0b101, ///< 30 Hz
    LSM303_MDATARATE_75    = 0b110, ///< 75 Hz
    LSM303_MDATARATE_220   = 0b111  ///< 220 Hz
} lsm303_mf_do_t;

/// \brief Magnetic field gain
/// \details \c CRB_REG_M register field
/// \ingroup lsm303data
typedef enum {
    LSM303_MGAIN_1_3   = 0b001, ///< ±1.3 Gauss
    LSM303_MGAIN_1_9   = 0b010, ///< ±1.9 Gauss
    LSM303_MGAIN_2_5   = 0b011, ///< ±2.5 Gauss
    LSM303_MGAIN_4_0   = 0b100, ///< ±4.0 Gauss
    LSM303_MGAIN_4_7   = 0b101, ///< ±4.7 Gauss
    LSM303_MGAIN_5_6   = 0b110, ///< ±5.6 Gauss
    LSM303_MGAIN_8_1   = 0b111  ///< ±8.1 Gauss
} lsm303_mf_gain_t;

/// \brief Magnetic field operating mode
/// \details \c MR_REG_M register field
/// \ingroup lsm303data
typedef enum {
    LSM303_MMODE_CONTINUOUS    = 0b00, ///< Continuous-conversion mode
    LSM303_MMODE_SINGLE        = 0b01, ///< Single-conversion mode
    LSM303_MMODE_SLEEP0        = 0b10, ///< Sleep mode. Device is placed in sleep mode
    LSM303_MMODE_SLEEP1        = 0b11  ///< Sleep mode. Device is placed in sleep mode
} lsm303_mf_md_t;

/// \union lsm303_reg_int_cfg_a_t lsm303dlhc.h
/// \brief Interrup configuration
/// \details Using for configuration \c INT1_CFG_A or \c INT2_CFG_A register
/// \ingroup lsm303data
typedef union {
#ifdef DOXYGEN
    /// \struct lsm303_reg_int_cfg_a_t::_unnamed lsm303dlhc.h
    /// \brief Register \c INT1_CFG_A or \c INT2_CFG_A fields
    /// \details __attribute__((__packed__))
    /// \ingroup lsm303data
    struct _unnamed {
#else
    struct __attribute__((__packed__)) {
#endif
        uint8_t xle     : 1; ///< Enable interrupt generation on X low event (0: disable, 1: enable)
        uint8_t xhe     : 1; ///< Enable interrupt generation on X high event  (0: disable, 1: enable)
        uint8_t yle     : 1; ///< Enable interrupt generation on Y low event (0: disable, 1: enable)
        uint8_t yhe     : 1; ///< Enable interrupt generation on Y high event  (0: disable, 1: enable)
        uint8_t zle     : 1; ///< Enable interrupt generation on Z low event (0: disable, 1: enable)
        uint8_t zhe     : 1; ///< Enable interrupt generation on Z high event  (0: disable, 1: enable)
        uint8_t aoi6d   : 2; ///< Interrupt mode: see \c lsm303_la_irq_mode_t enumerator
    };
    uint8_t reg; ///< Register byte
} lsm303_reg_int_cfg_a_t;

/// \union lsm303_reg_int_src_a_t lsm303dlhc.h
/// \brief Interrup source register
/// \details Read-only \c INT1_SRC_A or \c INT2_SRC_A register
/// \ingroup lsm303data
typedef union {
#ifdef DOXYGEN
    /// \struct lsm303_reg_int_src_a_t::_unnamed lsm303dlhc.h
    /// \brief Register \c INT1_SRC_A or \c INT2_SRC_A fields
    /// \details __attribute__((__packed__))
    /// \ingroup lsm303data
    struct _unnamed {
#else
    struct __attribute__((__packed__)) {
#endif
        uint8_t xl      : 1; ///< X low. (0: no interrupt, 1: Z low event has occurred)
        uint8_t xh      : 1; ///< X high. (0: no interrupt, 1: Z high event has occurred)
        uint8_t yl      : 1; ///< Y low. (0: no interrupt, 1: Z low event has occurred)
        uint8_t yh      : 1; ///< Y high. (0: no interrupt, 1: Z high event has occurred)
        uint8_t zl      : 1; ///< Z low. (0: no interrupt, 1: Z low event has occurred)
        uint8_t zh      : 1; ///< Z high. (0: no interrupt, 1: Z high event has occurred)
        uint8_t ia      : 1; ///< Interrupt active. (0: no interrupt has been generated, 1: one or more interrupts have been generated)
        uint8_t reserv  : 1; ///< Reserved bit
    };
    uint8_t reg; ///< Register byte
} lsm303_reg_int_src_a_t;

/// \brief Linear accelerometer setup
/// \param i2c I2C handler
/// \param odr Data rate
/// \param lpe Low-power mode: \c 0 - disable, \c 1 - enable
/// \param hr High-resolution output mode: \c 0 - disable, \c 1 - enable
/// \param fs Full-scale selection
/// \return \c HAL_OK if success or error code
/// \ingroup lsm303func
uint8_t lsm303_la_setup(I2C_HandleTypeDef* i2c, const lsm303_la_datarate_t odr, const uint8_t lpe, const uint8_t hr, const lsm303_la_fs_t fs);

/// \brief Linear accelerometer interrupt by \c INT1
/// \details Need connect \c INT1 to you stm32 pin and configure interrupt handler
/// \param i2c I2C handler
/// \param cfg \c INT1_CFG_A register. Fill lsm303_reg_int_cfg_a_t fields and pass the lsm303_reg_int_cfg_a_t::reg field as a parameter
/// \param threshould Interrupt trigger threshold
/// \param duration Duration of the interrupt event
/// \return \c HAL_OK if success or error code
/// \note For deactivate this interrupt use \c cfg = \c 0U
/// \ingroup lsm303func
uint8_t lsm303_la_int1(I2C_HandleTypeDef* i2c, const uint8_t cfg, uint8_t threshould, uint8_t duration);

/// \brief Linear accelerometer read interrupt source by \c INT1
/// \details Read \c INT1_SRC_A register
/// \param i2c I2C handler
/// \param src \c INT1_SRC_A register pointer. Pass the &lsm303_reg_int_src_a_t::reg field as a parameter and read lsm303_reg_int_src_a_t fields
/// \return \c HAL_OK if success or error code
/// \ingroup lsm303func
uint8_t lsm303_la_src1(I2C_HandleTypeDef* i2c, uint8_t* src);

/// \brief Linear accelerometer read raw data \a without \a conversion
/// \param i2c I2C handler
/// \param x X axis raw data pointer
/// \param y Y axis raw data pointer
/// \param z Z axis raw data pointer
/// \return \c HAL_OK if success or error code
/// \ingroup lsm303func
uint8_t lsm303_la_raw(I2C_HandleTypeDef* i2c, int16_t* x, int16_t* y, int16_t* z);

/// \brief Linear accelerometer read data
/// \details Read Linear accelerometer data and conversion to \b g
/// \param i2c I2C handler
/// \param x X axis pointer
/// \param y Y axis pointer
/// \param z Z axis pointer
/// \return \c HAL_OK if success or error code
/// \ingroup lsm303func
uint8_t lsm303_la_read(I2C_HandleTypeDef* i2c, float* x, float* y, float* z);

/// \brief Magnetic field sensor setup
/// \param i2c I2C handler
/// \param ten Temperature sensor: \c 0 - disable, \c 1 - enable
/// \param odr Data rate
/// \param gn Gain setting
/// \param md Magnetic sensor operating mode
/// \return \c HAL_OK if success or error code
/// \ingroup lsm303func
uint8_t lsm303_mf_setup(I2C_HandleTypeDef* i2c, const uint8_t ten, const lsm303_mf_do_t odr, const lsm303_mf_gain_t gn, const lsm303_mf_md_t md);

/// \brief Magnetic field read data \a without \a conversion
/// \param i2c I2C handler
/// \param x X axis raw data pointer
/// \param y Y axis raw data pointer
/// \param z Z axis raw data pointer
/// \return \c HAL_OK if success or error code
/// \ingroup lsm303func
uint8_t lsm303_mf_raw(I2C_HandleTypeDef* i2c, int16_t* x, int16_t* y, int16_t* z);

/// \brief Magnetic field read data
/// \details Read magnetic field data and conversion \b nanotesla
/// \param i2c I2C handler
/// \param x X axis pointer
/// \param y Y axis pointer
/// \param z Z axis pointer
/// \return \c HAL_OK if success or error code
/// \ingroup lsm303func
uint8_t lsm303_mf_read(I2C_HandleTypeDef* i2c, float* x, float* y, float* z);

#endif // __LSM303_H__