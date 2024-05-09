#pragma once

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "spl06-001-reg.h"

/**
 * @brief measuring modes of spl06-001
 * 
 * @details STANDBY mode perform no measure. 
 *          CMD_PRS and CMD_TEMP perform single shot measure as we send command. 
 *          BGD_PRS, BGD_TEMP and BGD_PRS_TEMP perform continuous measuring
 */
typedef enum {
    STANDBY, // idle
    CMD_PRS, CMD_TEMP, // command mode, single shot measure by command
    BGD_PRS, BGD_TEMP, BGD_PRS_TEMP // background mode, continuous measure
} spl06_mode_t;

/**
 * @brief configs of a sensor. 
 * 
 * @details multi sensors are supported by making this type's variable more than one.
 */
typedef struct spl06_config {
    uint8_t addr; // 0x77(default) or 0x76
    i2c_inst_t *i2c; // i2c0 or i2c1
    spl06_mode_t mode;
    uint8_t prs_config[2]; // rate, oversample
    uint8_t temp_config[2]; // rate, oversample
} spl06_config_t;

/**
 * @brief coefficients read from internal registers
 * 
 */
typedef struct spl06_coef {
    // pressure
    int32_t c00, c10;
    int16_t c20, c30, c01, c11, c21;
    // temp
    int16_t c0, c1;
} spl06_coef_t;

// user functions
/**
 * @brief initialize spl06-001 sensor
 * 
 * @param config config of a spl06-001 sensor. must set value before call of this function
 * @param coef read coefficients will store in this variable
 * @return int8_t if success return 0, if fails return PICO_ERROR_GENERIC
 */
int8_t spl06_init(spl06_config_t *config, spl06_coef_t *coef);

/**
 * @brief read calibrated pressure
 * 
 * @param config config to specify sensor to be read from
 * @param coef coefficients to compensate raw value. must set before call (set in spl06_init()).
 * @param prs read & calculated pressure value(hPa)
 */
void spl06_read_press_cal(spl06_config_t *config, spl06_coef_t *coef, float *prs);

/**
 * @brief read calibrated temperature
 * 
 * @param config config to specify sensor to be read from
 * @param coef coefficients to compensate raw value. must set before call (set in spl06_init()).
 * @param temp read & caluculated temperature value (degC)
 */
void spl06_read_temp_cal(spl06_config_t *config, spl06_coef_t *coef, float *temp);

// internal functions, but allowed to call from other files
/**
 * @brief configure rate & oversampling of pressure
 * 
 * @param config config to specify sensor
 * @param rate measuring per second
 * @param oversample oversampling per measure
 * @return int8_t return 0 if success, return PICO_ERROR_GENERIC if fails
 * @attention refer datasheet for detail
 */
int8_t spl06_config_prs(spl06_config_t *config, uint8_t rate, uint8_t oversample);

/**
 * @brief configure rate & oversampling of temperature
 * 
 * @param config config to specify sensor
 * @param rate measuring per second
 * @param oversample oversampling per measure
 * @return int8_t return 0 if success, return PICO_ERROR_GENERIC if fails
 * @attention refer datasheet for detail
 */
int8_t spl06_config_temp(spl06_config_t *config, uint8_t rate, uint8_t oversample);

/**
 * @brief set measuring mode
 * 
 * @param config config to specify sensor
 * @param mode measuring mode to be set
 */
int8_t spl06_set_mode(spl06_config_t *config, spl06_mode_t mode);

/**
 * @brief read coefficients
 * 
 * @param config config to specify sensor
 * @param coef coefficients for compensate pressure and temperature
 * @return int8_t return 0 if success, return PICO_ERROR_GENERIC if fails
 */
int8_t spl06_read_coef(spl06_config_t *config, spl06_coef_t *coef);

/**
 * @brief calculate altitude from pressure, 1013.25 hPa reference.
 * 
 * @param prs pressure(hPa)
 * @return float calculated altitude(m)
 */
float spl06_calc_alt(float prs);

// internal functions, not allowed to call from other files
// read raw values
static void spl06_read_press_raw(spl06_config_t *config, int32_t *prs);
static void spl06_read_temp_raw(spl06_config_t *config, int32_t *temp);
// convert config values to register values
static int8_t rate2config(uint8_t rate);
static int8_t oversample2config(uint8_t oversample, int16_t *time);
// convert oversampling times to scaling factor(k) for caclulate compensated value
static int32_t oversample2k(uint8_t oversample);
