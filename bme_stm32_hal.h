/** 
* @file     bme_stm32_hal.h 
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

#ifndef BME_STM32_HAL_H
#define BME_STM32_HAL_H

#include<stdint.h>
#include".\BME680_driver\bme680.h"
#include".\BME680_driver\bme680_defs.h"

#ifdef USE_HAL_DRIVER
#include "i2c.h"
#else
#error "The BME680 wrappers require the use of STM32CubeMX HAL drivers"
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
  * @brief Function to start up the BME680 driver
  * @param A pointer to a BME680 driver object
  * @retval 1 on success, 0 on failure
  */
int8_t bme680_begin(struct bme680_dev* dev);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BME_STM32_HAL_H */