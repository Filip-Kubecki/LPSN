#include "sht40.h"

/**
 * ====================================================================
 * PUBLIC FUNCTIONS
 * ====================================================================
 */

/**
 * @brief Initializes the SHT40 sensor structure and performs a soft reset.
 * Resets internal variables and sends the hardware reset command over I2C.
 */
HAL_StatusTypeDef SHT40_Init(SHT40_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    dev->temperature = 0.0f;
    dev->humidity = 0.0f;
    dev->serial_number = 0;

    // Send soft reset command to ensure sensor is in a clean state
    uint8_t cmd = SHT40_CMD_SOFT_RESET;
    
    if (HAL_I2C_Master_Transmit(dev->hi2c, SHT40_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }


    return HAL_OK;
}

/**
 * @brief Reads the unique 32-bit serial number from the sensor.
 * The sensor returns 6 bytes: [MSB, LSB, CRC, MSB, LSB, CRC].
 */
HAL_StatusTypeDef SHT40_GetSerialNumber(SHT40_t *dev) {
    uint8_t cmd = SHT40_CMD_READ_SERIAL;
    uint8_t buffer[6];

    // Request serial number
    if (HAL_I2C_Master_Transmit(dev->hi2c, SHT40_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Small delay required by the sensor to process the request
    HAL_Delay(1);

    // Receive 6 bytes (4 bytes for ID and 2 bytes for CRC)
    if (HAL_I2C_Master_Receive(dev->hi2c, SHT40_ADDR, buffer, 6, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine bytes into a 32-bit integer, skipping the CRC bytes at buffer[2] and buffer[5]
    dev->serial_number = ((uint32_t)buffer[0] << 24) | 
                         ((uint32_t)buffer[1] << 16) | 
                         ((uint32_t)buffer[3] << 8)  | 
                          (uint32_t)buffer[4];

    return HAL_OK;
}

/**
 * @brief Performs a measurement and calculates temperature and humidity.
 * Uses Low Precision mode to minimize power consumption.
 */
HAL_StatusTypeDef SHT40_Read_Data(SHT40_t *dev) {
    uint8_t cmd = SHT40_CMD_MEASURE_LP;
    uint8_t buffer[6];

    // Trigger measurement
    if (HAL_I2C_Master_Transmit(dev->hi2c, SHT40_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Wait for conversion (LP mode takes approx. 2.3ms)
    HAL_Delay(3);

    // Read 6 bytes of raw data
    if (HAL_I2C_Master_Receive(dev->hi2c, SHT40_ADDR, buffer, 6, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Convert raw temperature ticks to Celsius
    uint16_t t_ticks = (buffer[0] << 8) | buffer[1];
    dev->temperature = -45.0f + (175.0f * (float)t_ticks / 65535.0f);

    // Convert raw humidity ticks to Relative Humidity %
    uint16_t rh_ticks = (buffer[3] << 8) | buffer[4];
    float rh_val = -6.0f + (125.0f * (float)rh_ticks / 65535.0f);
    
    // Constrain humidity within physical limits (0-100%)
    if (rh_val < 0.0f) {
        rh_val = 0.0f;
    }
    
    if (rh_val > 100.0f) {
        rh_val = 100.0f;
    }
    
    dev->humidity = rh_val;

    return HAL_OK;
}