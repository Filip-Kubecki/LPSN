#include "bmp280.h"

/**
 * ====================================================================
 * PRIVATE HELPER FUNCTIONS
 * ====================================================================
 */

/**
 * @brief Reads a block of data from a specific register.
 */
static HAL_StatusTypeDef read_register(BMP280_t *dev, uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(dev->hi2c, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

/**
 * @brief Writes a single byte to a specific register.
 */
static HAL_StatusTypeDef write_register(BMP280_t *dev, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(dev->hi2c, BMP280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

/**
 * ====================================================================
 * PUBLIC FUNCTIONS
 * ====================================================================
 */

/**
 * @brief Initializes the BMP280 sensor.
 * Verifies the device ID and retrieves factory calibration parameters.
 */
HAL_StatusTypeDef BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    uint8_t chip_id = 0;

    // Verify sensor identity (expected ID for BMP280 is 0x58)
    if (read_register(dev, BMP280_REG_ID, &chip_id, 1) != HAL_OK) {
        return HAL_ERROR;
    }

    if (chip_id != 0x58) {
        return HAL_ERROR;
    }

    // Read the 24-byte factory calibration block
    uint8_t calib_buf[24];
    if (read_register(dev, BMP280_REG_CALIB, calib_buf, 24) != HAL_OK) {
        return HAL_ERROR;
    }

    // Map calibration buffer to the device structure (Little Endian conversion)
    dev->calib.dig_T1 = (uint16_t)((calib_buf[1] << 8) | calib_buf[0]);
    dev->calib.dig_T2 = (int16_t)((calib_buf[3] << 8) | calib_buf[2]);
    dev->calib.dig_T3 = (int16_t)((calib_buf[5] << 8) | calib_buf[4]);

    dev->calib.dig_P1 = (uint16_t)((calib_buf[7] << 8) | calib_buf[6]);
    dev->calib.dig_P2 = (int16_t)((calib_buf[9] << 8) | calib_buf[8]);
    dev->calib.dig_P3 = (int16_t)((calib_buf[11] << 8) | calib_buf[10]);
    dev->calib.dig_P4 = (int16_t)((calib_buf[13] << 8) | calib_buf[12]);
    dev->calib.dig_P5 = (int16_t)((calib_buf[15] << 8) | calib_buf[14]);
    dev->calib.dig_P6 = (int16_t)((calib_buf[17] << 8) | calib_buf[16]);
    dev->calib.dig_P7 = (int16_t)((calib_buf[19] << 8) | calib_buf[18]);
    dev->calib.dig_P8 = (int16_t)((calib_buf[21] << 8) | calib_buf[20]);
    dev->calib.dig_P9 = (int16_t)((calib_buf[23] << 8) | calib_buf[22]);

    return HAL_OK;
}

/**
 * @brief Triggers a measurement and updates sensor structure with compensated values.
 */
HAL_StatusTypeDef BMP280_Read_All(BMP280_t *dev) {
    // Trigger 'Forced Mode' for low power consumption
    // Configuration: Pressure oversampling x1, Temperature oversampling x1
    if (write_register(dev, BMP280_REG_CTRL_MEAS, 0x27) != HAL_OK) {
        return HAL_ERROR;
    }

    // Wait for the sensor to complete the conversion
    HAL_Delay(10);

    // Read 6 bytes of raw ADC data (Pressure and Temperature)
    uint8_t data[6];
    if (read_register(dev, BMP280_REG_PRESS_MSB, data, 6) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine bytes into 20-bit raw ADC values
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // Temperature Compensation (Bosch datasheet formula)
    float var1_t = (((float)adc_T) / 16384.0f - ((float)dev->calib.dig_T1) / 1024.0f) * ((float)dev->calib.dig_T2);
    
    float var2_t = ((((float)adc_T) / 131072.0f - ((float)dev->calib.dig_T1) / 8192.0f) *
                    (((float)adc_T) / 131072.0f - ((float)dev->calib.dig_T1) / 8192.0f)) * ((float)dev->calib.dig_T3);
    
    float t_fine = var1_t + var2_t;
    dev->temperature = t_fine / 5120.0f;

    // Pressure Compensation
    float var1_p = (t_fine / 2.0f) - 64000.0f;
    float var2_p = var1_p * var1_p * ((float)dev->calib.dig_P6) / 32768.0f;
    var2_p = var2_p + var1_p * ((float)dev->calib.dig_P5) * 2.0f;
    var2_p = (var2_p / 4.0f) + (((float)dev->calib.dig_P4) * 65536.0f);
    
    var1_p = (((float)dev->calib.dig_P3) * var1_p * var1_p / 524288.0f + ((float)dev->calib.dig_P2) * var1_p) / 524288.0f;
    var1_p = (1.0f + var1_p / 32768.0f) * ((float)dev->calib.dig_P1);

    // Avoid division by zero if sensor fails
    if (var1_p == 0.0f) {
        return HAL_ERROR;
    }

    float p = 1048576.0f - (float)adc_P;
    p = (p - (var2_p / 4096.0f)) * 6250.0f / var1_p;
    
    float var1_final = ((float)dev->calib.dig_P9) * p * p / 2147483648.0f;
    float var2_final = p * ((float)dev->calib.dig_P8) / 32768.0f;
    
    // Final pressure in hPa
    dev->pressure = (p + (var1_final + var2_final + ((float)dev->calib.dig_P7)) / 16.0f) / 100.0f;

    return HAL_OK;
}