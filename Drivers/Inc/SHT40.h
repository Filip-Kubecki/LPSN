#ifndef SHT40_H
#define SHT40_H

#include "stm32u0xx_hal.h"

/* I2C addres (shifted one bit for HAL) */
#define SHT40_ADDR (0x44 << 1)

/* SHT40 commands */
#define SHT40_CMD_MEASURE_LP    0xE0  /* Low accuracy reading */
#define SHT40_CMD_READ_SERIAL   0x89  /* Read serial ID */
#define SHT40_CMD_SOFT_RESET    0x94  /* Soft reset */

/**
 * @brief Struct containing sensor data
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    float temperature;
    float humidity;
    uint32_t serial_number;
} SHT40_t;

/* Function prototypes */
HAL_StatusTypeDef SHT40_Init(SHT40_t *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef SHT40_GetSerialNumber(SHT40_t *dev);
HAL_StatusTypeDef SHT40_Read_Raw(SHT40_t *dev);
HAL_StatusTypeDef SHT40_Read_Data(SHT40_t *dev);

#endif /* SHT40_H */