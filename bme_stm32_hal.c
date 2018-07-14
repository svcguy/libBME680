/** 
* @file     bme_stm32_hal.c 
*
* @brief    Wrappers for the Bosch BME series of sensors and the associated
*           library provided by Bosch
*           
*
*           Written to be used with the STM32CubeMX libraries for STM32 microcontrollers
*           https://www.st.com/en/development-tools/stm32cubemx.html
*           
*           Author:  Andy Josephson, 2018      
*/

#include <stdlib.h>
#include <string.h>
#include "bme680.h"
#include "stm32f1xx_hal.h"

// This would need to be changed to match your specific I2C instance
extern I2C_HandleTypeDef hi2c1;

// Private function prototypes
void bme680_delay_ms(uint32_t period);
int8_t bme680_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme680_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t bme680_begin(struct bme680_dev* dev);

/**
  * @brief Delay function for BME680 driver
  * @param period: Time in ms to delay
  * @retval None
  */
void bme680_delay_ms(uint32_t period)
{ 
  // Use HAL delay function
  HAL_Delay(period);
}

/**
  * @brief Wrapper for I2C read function call
  * @param dev_id: I2C Address for the device
  * @param reg_addr: Address of the register to read from
  * @param *reg_data: A pointer to a location to store read data
  * @param len: Number of bytes to read
  * @retval 0 on success, -1 on failure
  */
int8_t bme680_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t ret = 0;

  if( HAL_I2C_Master_Transmit( &hi2c1, (uint16_t)(dev_id<<1), &reg_addr, 1, 100 ) != HAL_OK )
  {
    return -1;
  }
  HAL_Delay(1);
  if( HAL_I2C_Master_Receive( &hi2c1, (uint16_t)(dev_id<<1), reg_data, len, 100 ) != HAL_OK )
  {
    return -1;
  }

  return ret;
}

/**
  * @brief Wrapper for I2C write function call
  * @param dev_id: I2C address of the device to write to
  * @param reg_addr: Address of the register to write to
  * @param *reg_data: A pointer to data to write
  * @param len: Number of bytes to write
  * @retval 0 on success, -1 on failure
  */
int8_t bme680_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t ret = 0;
  uint8_t* i2c_data;

  // This is super nasty but according to the BME680 library (https://github.com/BoschSensortec/BME680_driver),
  // the register address and data need to be sent in the same I2C transmission.  
  // Since they're seperate variables, I'm using malloc() to prepend reg_addr to 
  // reg_data and then send if off.  Sorry if this offends someone, I'm open to
  // any other solutions

  i2c_data = malloc( len+1 );
  i2c_data[0] = reg_addr;
  memcpy( &i2c_data[1], reg_data, len);

  // Send over I2C
  if( HAL_I2C_Master_Transmit( &hi2c1, (uint16_t)(dev_id<<1), i2c_data, len+1, 100 ) != HAL_OK )
  {
    return -1;
  }

  // Don't forget to free
  free(i2c_data);
  return ret;
}

/**
  * @brief Function to start up the BME680 driver
  * @param A pointer to a BME680 driver object
  * @retval 1 on success, 0 on failure
  */
int8_t bme680_begin(struct bme680_dev* dev)
{
  dev->dev_id = BME680_I2C_ADDR_SECONDARY;
  dev->intf = BME680_I2C_INTF;
  dev->read = bme680_i2c_read;
  dev->write = bme680_i2c_write;
  dev->delay_ms = bme680_delay_ms;
  /* amb_temp can be set to 25 prior to configuring the gas sensor 
  * or by performing a few temperature readings without operating the gas sensor.
  */
  dev->amb_temp = 25;

  if( bme680_init(dev) != BME680_OK )
  {
    dev = NULL;
    return 0;
  }

  uint8_t set_required_settings;

  /* Set the temperature, pressure and humidity settings */
  dev->tph_sett.os_hum = BME680_OS_2X;
  dev->tph_sett.os_pres = BME680_OS_4X;
  dev->tph_sett.os_temp = BME680_OS_8X;
  dev->tph_sett.filter = BME680_FILTER_SIZE_3;

  /* Set the remaining gas sensor settings and link the heating profile */
  dev->gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  dev->gas_sett.heatr_temp = 320; /* degree Celsius */
  dev->gas_sett.heatr_dur = 150; /* milliseconds */

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  dev->power_mode = BME680_FORCED_MODE; 

  /* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
                          | BME680_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  if( bme680_set_sensor_settings(set_required_settings, dev) != BME680_OK )
  {
    return 0;
  }

  /* Set the power mode */
  if( bme680_set_sensor_mode(dev) != BME680_OK )
  {
    return 0;
  }

  return 1;
}