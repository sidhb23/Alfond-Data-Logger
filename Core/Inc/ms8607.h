#ifndef MS8607_H
#define MS8607_H

#include "stm32u5xx_hal.h"  // Adjust to match your STM32 series
#include <stdint.h>

/* MS8607 I2C Addresses */
#define MS8607_I2C_ADDRESS_PRESSURE   0x76  // 7-bit I2C address for pressure/temp
#define MS8607_I2C_ADDRESS_HUMIDITY   0x40  // 7-bit I2C address for humidity

/* MS8607 Commands */
#define MS8607_RESET_COMMAND          0x1E  // Reset command for pressure sensor
#define MS8607_PROM_READ_BASE         0xA0  // PROM read start address
#define MS8607_CONVERT_D1_COMMAND     0x40  // Convert D1 (pressure) command
#define MS8607_CONVERT_D2_COMMAND     0x50  // Convert D2 (temperature) command
#define MS8607_READ_ADC               0x00  // Read ADC result command

#define MS8607_HUMIDITY_MEASURE_HOLD  0xE5  // Hold master mode humidity measure
#define MS8607_HUMIDITY_MEASURE_NOHOLD 0xF5  // No hold mode humidity measure

/* MS8607 Oversampling */
typedef enum {
    MS8607_OSR_256 = 0x00,
    MS8607_OSR_512 = 0x02,
    MS8607_OSR_1024 = 0x04,
    MS8607_OSR_2048 = 0x06,
    MS8607_OSR_4096 = 0x08
} ms8607_osr_t;

/* Function Prototypes */
void MS8607_Init(I2C_HandleTypeDef *hi2c);
void MS8607_Reset(I2C_HandleTypeDef *hi2c);
void MS8607_ReadPROM(I2C_HandleTypeDef *hi2c, uint16_t *prom);
uint32_t MS8607_ReadADC(I2C_HandleTypeDef *hi2c);
float MS8607_ReadTemperature(I2C_HandleTypeDef *hi2c, uint16_t *prom);
float MS8607_ReadPressure(I2C_HandleTypeDef *hi2c, uint16_t *prom, float temperature);
float MS8607_ReadHumidity(I2C_HandleTypeDef *hi2c);

#endif  // MS8607_H
