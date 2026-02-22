#ifndef BMP280_H
#define BMP280_H

#include "stm32u0xx_hal.h"

/* I2C addres (shifted one bit for HAL - SDO connected to ground) */
#define BMP280_ADDR (0x76 << 1)

/* Registers */
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CALIB        0x88 // 24 bit calibration register

/**
 * @brief Sturct with calibration data
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} BMP280_Calib_t;


/**
 * @brief Struct containing sensor data
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    float temperature;
    float pressure;
    BMP280_Calib_t calib;
} BMP280_t;

/* Function prototypes */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMP280_Read_All(BMP280_t *dev);

#endif