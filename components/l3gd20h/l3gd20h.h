/**
 * Driver for L3GD20H 3-axes digital output gyroscope connected to I2C or SPI.
 * It can also be used with L3GD20 and L3G4200D.
 *
 * Part of esp-open-rtos [https://github.com/SuperHouse/esp-open-rtos]
 *
 * ---------------------------------------------------------------------------
 *
 * The BSD License (3-clause license)
 *
 * Copyright (c) 2017 Gunar Schorcht (https://github.com/gschorcht]
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __L3GD20H_H__
#define __L3GD20H_H__

// Uncomment one of the following defines to enable debug output
// #define L3GD20H_DEBUG_LEVEL_1    // only error messages
#define L3GD20H_DEBUG_LEVEL_2    // debug and error messages

// L3GD20H addresses
#define L3GD20H_I2C_ADDRESS_1           0x6a  // SDO pin is low
#define L3GD20H_I2C_ADDRESS_2           0x6b  // SDO pin is high

// L3GD20 addresses
#define L3GD20_I2C_ADDRESS_1            0x6a  // SDO pin is low
#define L3GD20_I2C_ADDRESS_2            0x6b  // SDO pin is high

// L3G4200D addresses
#define L3G4200D_I2C_ADDRESS_1          0x68  // SDO pin is low
#define L3G4200D_I2C_ADDRESS_2          0x69  // SDO pin is high

// L3GD20H chip id
#define L3GD20H_CHIP_ID                 0xd7  // L3GD20H_REG_WHO_AM_I<7:0>

// L3GD20 chip id
#define L3GD20_CHIP_ID                  0xd4  // L3GD20H_REG_WHO_AM_I<7:0>

// L3G4200D chip id
#define L3G4200D_CHIP_ID                0xd3  // L3GD20H_REG_WHO_AM_I<7:0>

// Definition of error codes
#define L3GD20H_OK                      0
#define L3GD20H_NOK                     -1

#define L3GD20H_INT_ERROR_MASK          0x000f
#define L3GD20H_DRV_ERROR_MASK          0xfff0

// Error codes for I2C and SPI interfaces ORed with L3GD20H driver error codes
#define L3GD20H_I2C_READ_FAILED         1
#define L3GD20H_I2C_WRITE_FAILED        2
#define L3GD20H_I2C_BUSY                3
#define L3GD20H_SPI_WRITE_FAILED        4
#define L3GD20H_SPI_READ_FAILED         5
#define L3GD20H_SPI_BUFFER_OVERFLOW     6
#define L3GD20H_SPI_SET_PAGE_FAILED     7

// L3GD20H driver error codes ORed with error codes for I2C and SPI interfaces
#define L3GD20H_WRONG_CHIP_ID              ( 1 << 8)
#define L3GD20H_WRONG_BANDWIDTH            ( 2 << 8)
#define L3GD20H_GET_RAW_DATA_FAILED        ( 3 << 8)
#define L3GD20H_GET_RAW_DATA_FIFO_FAILED   ( 4 << 8)
#define L3GD20H_WRONG_INT_TYPE             ( 5 << 8)
#define L3GD20H_CONFIG_INT_SIGNALS_FAILED  ( 6 << 8)
#define L3GD20H_CONFIG_INT1_FAILED         ( 7 << 8)
#define L3GD20H_CONFIG_INT2_FAILED         ( 8 << 8)
#define L3GD20H_INT1_SOURCE_FAILED         ( 9 << 8)
#define L3GD20H_INT2_SOURCE_FAILED         (10 << 8)
#define L3GD20H_SEL_OUT_FILTER_FAILED      (11 << 8)
#define L3GD20H_CONFIG_HPF_FAILED          (12 << 8)
#define L3GD20H_ENABLE_HPF_FAILED          (13 << 8)

#if defined(ESP_PLATFORM) || defined(__linux__)
#include "l3gd20h_types.h"
#else // ESP8266 (esp-open-rtos)
#include "l3gd20h/l3gd20h_types.h"
#endif

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)
#include "driver/spi_master.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief	Initialize the sensor
 *
 * Sensor is soft reset and put into sleep mode. All registers are reset to 
 * default values.
 *
 * @param   bus     I2C or SPI bus at which L3GD20H sensor is connected
 * @param   addr    I2C addr of the L3GD20H sensor, 0 for using SPI
 * @param   cs      SPI CS GPIO, ignored for I2C
 * @return          pointer to sensor data structure, or NULL on error
 */
l3gd20h_sensor_t* l3gd20h_init_sensor (uint8_t bus, uint8_t addr, uint8_t cs);


/**
 * @brief	Set sensor mode
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   mode    sensor mode with certain output data rate
 * @param   bw      bandwidth
 * @param   x       true enable x-axis, false disable x-axis
 * @param   y       true enable y-axis, false disable y-axis
 * @param   z       true enable z-axis, false disable z-axis
 * @return          true on success, false on error
 */
bool l3gd20h_set_mode (l3gd20h_sensor_t* dev, l3gd20h_mode_t mode, uint8_t bw,
                       bool x, bool y, bool z);
                       

/**
 * @brief	Set scale (full range range)
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   scale   range setting
 * @return          true on success, false on error
 */
bool l3gd20h_set_scale (l3gd20h_sensor_t* dev, l3gd20h_scale_t sens);
                              
                              
/**
 * @brief   Set FIFO mode
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   mode    FIFO mode
 * @return          true on success, false on error
 */
bool l3gd20h_set_fifo_mode (l3gd20h_sensor_t* dev, 
                            l3gd20h_fifo_mode_t mode);
                            

/**
 * @brief   Filter selection for raw data output values
 *
 * High pass filter (HPF) is configured with function *l3gd20h_config_hpf*. If
 * HPF is selected, it is enabled implicitly.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   filter   selected filters for output values
 * @return           true on success, false on error
 */
bool l3gd20h_select_output_filter (l3gd20h_sensor_t* dev,
                                   l3gd20h_filter_t filter);


/**
 * @brief	Test whether new sets of data are available
 *
 * In the bypass mode it returns 1 if new data are available. In any FIFO
 * mode, however, it returns the number of data sets waiting in the FIFO.
 *
 * @param   dev     pointer to the sensor device data structure
 * @return          number of sets of data available (max. 32) or 0.
 */
uint8_t l3gd20h_new_data (l3gd20h_sensor_t* dev);


/**
 * @brief	Get one set of output values in degree
 *
 * In the bypass mode, it returns the last measured value set. In any FIFO
 * mode, however, it returns the oldest (first) value set in the FIFO.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   data    pointer to float data structure filled with values
 * @return          true on success, false on error
 */
bool l3gd20h_get_float_data (l3gd20h_sensor_t* dev,
                             l3gd20h_float_data_t* data);


/**
 * @brief	Get all sets of output values in degree from the FIFO
 *
 * In bypass mode, it returns exactly one value set.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   data    array of 32 float data structures
 * @return          number of data sets read from fifo on success or 0 on error
 */
uint8_t l3gd20h_get_float_data_fifo (l3gd20h_sensor_t* dev,
                                     l3gd20h_float_data_fifo_t data);


/**
 * @brief	Get one set of raw output data as 16 bit two's complements
 *
 * In the bypass mode, it returns the last measured value set. In any FIFO
 * mode, however, it returns the oldest (first) value set in the FIFO.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   raw     pointer to raw data structure filled with values
 * @return          true on success, false on error
 */
bool l3gd20h_get_raw_data (l3gd20h_sensor_t* dev,
                           l3gd20h_raw_data_t* raw);


/**
 * @brief	Get all sets of raw output data from the FIFO
 *
 * In bypass mode, it returns exactly one raw data set.
 *
 * @param   dev     pointer to the sensor device data structure
 * @param   raw     arry of 32 raw data structures
 * @return          number of data sets read from fifo on success or 0 on error
 */
uint8_t l3gd20h_get_raw_data_fifo (l3gd20h_sensor_t* dev,
                                   l3gd20h_raw_data_fifo_t raw);
                                   

/**
 * @brief   Set cofiguration for interrupt signal INT1 (axis movement wake up)
 *
 * Set the configuration for interrupts that are generated when a certain
 * angular rate is higher or lower than defined thresholds.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   INT1 configuration
 * @return           true on success, false on error
 */
bool l3gd20h_set_int1_config (l3gd20h_sensor_t* dev, 
                              l3gd20h_int1_config_t* config);


/**
 * @brief   Get cofiguration for interrupt signal INT1 (axis movement wake up)
 *
 * Get the configuration for interrupts that are generated when a certain
 * angular rate is higher or lower than defined thresholds.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   config   INT1 configuration
 * @return           true on success, false on error
 */
bool l3gd20h_get_int1_config (l3gd20h_sensor_t* dev, 
                              l3gd20h_int1_config_t* config);


/**
 * @brief   Get the source of the interrupt signal INT1 when happened
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   type     pointer to the interrupt source
 * @return           true on success, false on error
 */
bool l3gd20h_get_int1_source (l3gd20h_sensor_t* dev, 
                              l3gd20h_int1_source_t* source);


/**
 * @brief   Enable/disable interrupt signal INT2 (data ready / fifo)
 *
 * Enables or diables interrupts that are generated either when data are 
 * ready to read or FIFO events happen.
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   type     type of interrupt to be enabled/disabled
 * @param   value    true to enable/false to disable the interrupt
 * @return           true on success, false on error
 */
bool l3gd20h_enable_int2 (l3gd20h_sensor_t* dev, 
                          l3gd20h_int2_types_t type, bool value);
                                   

/**
 * @brief   Get the source of the interrupt signal INT2 when happened
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   source   pointer to the interrupt source
 * @return           true on success, false on error
 */
bool l3gd20h_get_int2_source (l3gd20h_sensor_t* dev, 
                              l3gd20h_int2_source_t* source);


/**
 * @brief   Set signal configuration for INT1 and INT2 signals
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   level    define interrupt signal as low or high active
 * @param   type     define interrupt signal as pushed/pulled or open drain
 * @return           true on success, false on error
 */
bool l3gd20h_config_int_signals (l3gd20h_sensor_t* dev,
                                 l3gd20h_signal_level_t level,
                                 l3gd20h_signal_type_t type);

                              
/**
 * @brief   Config HPF (high pass filter)
 *
 * @param   dev      pointer to the sensor device data structure
 * @param   mode     high pass filter mode
 * @param   f_cutoff cutoff frequency (depends on output data rate) [0 ... 15]
 * @param   ref      reference in HPF reference mode, not used in other modes
 * @return           true on success, false on error
 */
bool l3gd20h_config_hpf (l3gd20h_sensor_t* dev, 
                         l3gd20h_hpf_mode_t mode, 
                         uint8_t f_cutoff, uint8_t ref);


/**
 * @brief   Get temperature
 *
 * @param   dev      pointer to the sensor device data structure
 * @return           temperature in degree as two's complement
 */
int8_t l3gd20h_get_temperature (l3gd20h_sensor_t* dev);


#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* __L3GD20H_H__ */
