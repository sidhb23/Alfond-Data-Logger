#include "ms8607.h"
#include "stm32u5xx_hal.h"  // Adjust according to your STM32 series
#include <stdint.h>
#include <math.h>

/**
 * @brief Initialize the MS8607 sensor.
 * @param hi2c Pointer to the I2C handle.
 */
void MS8607_Init(I2C_HandleTypeDef *hi2c) {
    MS8607_Reset(hi2c);
}

/**
 * @brief Reset the MS8607 sensor.
 * @param hi2c Pointer to the I2C handle.
 */
void MS8607_Reset(I2C_HandleTypeDef *hi2c) {
    uint8_t reset_cmd = MS8607_RESET_COMMAND;
    HAL_I2C_Master_Transmit(hi2c, MS8607_I2C_ADDRESS_PRESSURE << 1, &reset_cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10); // Wait for sensor reset
}

/**
 * @brief Read the PROM data from the MS8607.
 * @param hi2c Pointer to the I2C handle.
 * @param prom Array to store the PROM values (6 values).
 */
void MS8607_ReadPROM(I2C_HandleTypeDef *hi2c, uint16_t *prom) {
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t cmd = MS8607_PROM_READ_BASE + (i * 2);
        uint8_t data[2];
        HAL_I2C_Master_Transmit(hi2c, MS8607_I2C_ADDRESS_PRESSURE << 1, &cmd, 1, HAL_MAX_DELAY);
        HAL_Delay(10);
        HAL_I2C_Master_Receive(hi2c, MS8607_I2C_ADDRESS_PRESSURE << 1, data, 2, HAL_MAX_DELAY);
        prom[i] = (data[0] << 8) | data[1]; // Combine two bytes into a 16-bit value
    }
}

/**
 * @brief Read the raw ADC value from the MS8607.
 * @param hi2c Pointer to the I2C handle.
 * @return The raw ADC value.
 */
uint32_t MS8607_ReadADC(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd = MS8607_READ_ADC;
    uint8_t data[3];

    HAL_I2C_Master_Transmit(hi2c, MS8607_I2C_ADDRESS_PRESSURE << 1, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10); // Wait for conversion
    HAL_I2C_Master_Receive(hi2c, MS8607_I2C_ADDRESS_PRESSURE << 1, data, 3, HAL_MAX_DELAY);

    return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]; // Combine the 3 bytes into a 24-bit value
}

/**
 * @brief Read temperature from the MS8607.
 * @param hi2c Pointer to the I2C handle.
 * @param prom Array containing the calibration data.
 * @return The temperature in degrees Celsius.
 */
float MS8607_ReadTemperature(I2C_HandleTypeDef *hi2c, uint16_t *prom) {
    uint8_t cmd = MS8607_CONVERT_D2_COMMAND;
    HAL_I2C_Master_Transmit(hi2c, MS8607_I2C_ADDRESS_PRESSURE << 1, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10); // Wait for conversion

    // Read the ADC value
    uint32_t rawTemp = MS8607_ReadADC(hi2c);
    // Calculate temperature using calibration data
    uint32_t dT = rawTemp - ((int32_t)prom[4] << 8); // dT = D2 - T_ref * 2 ^ 8
    uint64_t temperature = 2000 + (dT * prom[5] / (1 << 23)); // Temp = 2000 + dT * TEMPSENS / 2^23
    if (temperature < 2000) {
            temperature -= (3 * dT * dT) / (1LL << 33);
        } else {
            temperature -= (5 * dT * dT) / (1LL << 38);
        }
    return (float)temperature/100.00; // Return temperature in Celsius
}

/**
 * @brief Read pressure from the MS8607.
 * @param hi2c Pointer to the I2C handle.
 * @return The pressure in hPa.
 */
float MS8607_ReadPressure(I2C_HandleTypeDef *hi2c, uint16_t *prom, float temperature) {
    uint8_t cmd = MS8607_CONVERT_D1_COMMAND;
    HAL_I2C_Master_Transmit(hi2c, MS8607_I2C_ADDRESS_PRESSURE << 1, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10); // Wait for conversion

    // Read the ADC value
    uint32_t rawPress = MS8607_ReadADC(hi2c);
    uint32_t rawTemp = MS8607_ReadADC(hi2c)+2;
    // Calculate pressure using calibration data
    int32_t dT = rawTemp - (prom[4] << 8); // dT = D1 - T_ref
    int32_t off = (prom[1] << 17) + ((prom[3] * dT) / (1 << 6));
    int32_t sens = (prom[0] << 16) + ((prom[2] * dT) / (1 << 7));

    float pressure = ((rawPress * sens) / (1 << 21) - off) / (1 << 15);
    return pressure/100.0; // Return pressure in hPa
}

/**
 * @brief Read humidity from the MS8607.
 * @param hi2c Pointer to the I2C handle.
 * @return The humidity in percentage.
 */
float MS8607_ReadHumidity(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd = MS8607_HUMIDITY_MEASURE_HOLD;
    uint8_t data[2];

    HAL_I2C_Master_Transmit(hi2c, MS8607_I2C_ADDRESS_HUMIDITY << 1, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10); // Wait for conversion
    HAL_I2C_Master_Receive(hi2c, MS8607_I2C_ADDRESS_HUMIDITY << 1, data, 2, HAL_MAX_DELAY);

    // Combine the two bytes into a 16-bit value and convert to percentage
    uint16_t rawHumidity = (data[0] << 8) | data[1];

    // Calculate humidity using calibration data
    float humidity = -6 + 125 * (float)rawHumidity/(1<<16); // Convert to percentage
    return humidity; // Return humidity in percentage
}
