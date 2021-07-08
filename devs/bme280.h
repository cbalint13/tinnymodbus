/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018,2021
 *
 * Balint Cristian <cristian dot balint at gmail dot com>
 * Stefan Reichhard <s.reichhard@netMedia.pro>
 *
 * TinnyModbus
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*

  bme280.h (Bosch BME280 temperature and pressure sensor)

*/

#ifndef _BME280_H
#define _BME280_H


#include <avr/io.h>
#include <avr/interrupt.h>

#define BME280_TEMP 0x00
#define BME280_PRES 0x01
#define BME280_HUM  0x02

#define BME280_ADDRESS 0x76


// REGISTER ADDRESS DEFINITION
#define BME280_CHIP_ID_REG          (0xD0)  //Chip ID Register
#define BME280_RST_REG              (0xE0)  //Softreset Register
#define BME280_CTRL_HUM_REG         (0xF2)  //Ctrl Humidity Register
#define BME280_STAT_REG             (0xF3)  //Status Register
#define BME280_CTRL_MEAS_REG        (0xF4)  //Ctrl Measure Register
#define BME280_CONFIG_REG           (0xF5)  //Configuration Register
#define BME280_PRESSURE_MSB_REG     (0xF7)  //Pressure MSB Register
#define BME280_PRESSURE_LSB_REG     (0xF8)  //Pressure LSB Register
#define BME280_PRESSURE_XLSB_REG    (0xF9)  //Pressure XLSB Register
#define BME280_TEMPERATURE_MSB_REG  (0xFA)  //Temperature MSB Reg
#define BME280_TEMPERATURE_LSB_REG  (0xFB)  //Temperature LSB Reg
#define BME280_TEMPERATURE_XLSB_REG (0xFC)  //Temperature XLSB Reg
#define BME280_HUMIDITY_MSB_REG     (0xFD)  //Humidity MSB Reg
#define BME280_HUMIDITY_LSB_REG     (0xFE)  //Humidity LSB Reg

// BIT LENGTH,POSITION AND MASK DEFINITION

// Status Register
#define BME280_STATUS_REG_MEASURING__POS               (3)
#define BME280_STATUS_REG_MEASURING__MSK               (0x08)
#define BME280_STATUS_REG_MEASURING__LEN               (1)
#define BME280_STATUS_REG_MEASURING__REG               (BME280_STAT_REG)

#define BME280_STATUS_REG_IM_UPDATE__POS               (0)
#define BME280_STATUS_REG_IM_UPDATE__MSK               (0x01)
#define BME280_STATUS_REG_IM_UPDATE__LEN               (1)
#define BME280_STATUS_REG_IM_UPDATE__REG               (BME280_STAT_REG)

// HUMIDITY OVERSAMPLING
#define BME280_CTRL_REG_OVERSAMP_HUMIDITY__POS         (0)
//#define BME280_CTRL_REG_OVERSAMP_HUMIDITY__MSK         (0x??)
#define BME280_CTRL_REG_OVERSAMP_HUMIDITY__LEN         (3)
#define BME280_CTRL_REG_OVERSAMP_HUMIDITY__REG         (BME280_CTRL_HUM_REG)

// TEMPERATURE OVERSAMPLING
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS (5)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK (0xE0)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN (3)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG (BME280_CTRL_MEAS_REG)

//FOR PRESSURE OVERSAMPLING
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS    (2)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK    (0x1C)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN    (3)
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG    (BME280_CTRL_MEAS_REG)

// FOR POWER MODE
#define BME280_CTRL_MEAS_REG_POWER_MODE__POS           (0)
#define BME280_CTRL_MEAS_REG_POWER_MODE__MSK           (0x03)
#define BME280_CTRL_MEAS_REG_POWER_MODE__LEN           (2)
#define BME280_CTRL_MEAS_REG_POWER_MODE__REG           (BME280_CTRL_MEAS_REG)

// FOR STANDBY DURATION
#define BME280_CONFIG_REG_STANDBY_DURN__POS            (5)
#define BME280_CONFIG_REG_STANDBY_DURN__MSK            (0xE0)
#define BME280_CONFIG_REG_STANDBY_DURN__LEN            (3)
#define BME280_CONFIG_REG_STANDBY_DURN__REG            (BME280_CONFIG_REG)

// FOR IIR FILTER
#define BME280_CONFIG_REG_FILTER__POS                  (2)
#define BME280_CONFIG_REG_FILTER__MSK                  (0x1C)
#define BME280_CONFIG_REG_FILTER__LEN                  (3)
#define BME280_CONFIG_REG_FILTER__REG                  (BME280_CONFIG_REG)

//FOR SPI ENABLE
#define BME280_CONFIG_REG_SPI3_ENABLE__POS             (0)
#define BME280_CONFIG_REG_SPI3_ENABLE__MSK             (0x01)
#define BME280_CONFIG_REG_SPI3_ENABLE__LEN             (1)
#define BME280_CONFIG_REG_SPI3_ENABLE__REG             (BME280_CONFIG_REG)

//FOR PRESSURE AND TEMPERATURE DATA REGISTERS
#define BME280_PRESSURE_XLSB_REG_DATA__POS             (4)
#define BME280_PRESSURE_XLSB_REG_DATA__MSK             (0xF0)
#define BME280_PRESSURE_XLSB_REG_DATA__LEN             (4)
#define BME280_PRESSURE_XLSB_REG_DATA__REG             (BME280_PRESSURE_XLSB_REG)

#define BME280_TEMPERATURE_XLSB_REG_DATA__POS          (4)
#define BME280_TEMPERATURE_XLSB_REG_DATA__MSK          (0xF0)
#define BME280_TEMPERATURE_XLSB_REG_DATA__LEN          (4)
#define BME280_TEMPERATURE_XLSB_REG_DATA__REG          (BME280_TEMPERATURE_XLSB_REG)

#define BME280_HUMIDITY_LSB_REG_DATA__POS             (4)
#define BME280_HUMIDITY_LSB_REG_DATA__MSK             (0xF0)
#define BME280_HUMIDITY_LSB_REG_DATA__LEN             (4)
#define BME280_HUMIDITY_LSB_REG_DATA__REG             (BME280_HUMIDITY_LSB_REG)

#define BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG        (0x88)
#define BME280_TEMPERATURE_CALIB_DIG_T1_MSB_REG        (0x89)
#define BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG        (0x8A)
#define BME280_TEMPERATURE_CALIB_DIG_T2_MSB_REG        (0x8B)
#define BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG        (0x8C)
#define BME280_TEMPERATURE_CALIB_DIG_T3_MSB_REG        (0x8D)
#define BME280_PRESSURE_CALIB_DIG_P1_LSB_REG           (0x8E)
#define BME280_PRESSURE_CALIB_DIG_P1_MSB_REG           (0x8F)
#define BME280_PRESSURE_CALIB_DIG_P2_LSB_REG           (0x90)
#define BME280_PRESSURE_CALIB_DIG_P2_MSB_REG           (0x91)
#define BME280_PRESSURE_CALIB_DIG_P3_LSB_REG           (0x92)
#define BME280_PRESSURE_CALIB_DIG_P3_MSB_REG           (0x93)
#define BME280_PRESSURE_CALIB_DIG_P4_LSB_REG           (0x94)
#define BME280_PRESSURE_CALIB_DIG_P4_MSB_REG           (0x95)
#define BME280_PRESSURE_CALIB_DIG_P5_LSB_REG           (0x96)
#define BME280_PRESSURE_CALIB_DIG_P5_MSB_REG           (0x97)
#define BME280_PRESSURE_CALIB_DIG_P6_LSB_REG           (0x98)
#define BME280_PRESSURE_CALIB_DIG_P6_MSB_REG           (0x99)
#define BME280_PRESSURE_CALIB_DIG_P7_LSB_REG           (0x9A)
#define BME280_PRESSURE_CALIB_DIG_P7_MSB_REG           (0x9B)
#define BME280_PRESSURE_CALIB_DIG_P8_LSB_REG           (0x9C)
#define BME280_PRESSURE_CALIB_DIG_P8_MSB_REG           (0x9D)
#define BME280_PRESSURE_CALIB_DIG_P9_LSB_REG           (0x9E)
#define BME280_PRESSURE_CALIB_DIG_P9_MSB_REG           (0x9F)

#define BME280_HUMIDITY_CALIB_DIG_H1_REG               (0xA1)
#define BME280_HUMIDITY_CALIB_DIG_H2_REG               (0xE1)
#define BME280_HUMIDITY_CALIB_DIG_H3_REG               (0xE3)
#define BME280_HUMIDITY_CALIB_DIG_H4_REG               (0xE4)
#define BME280_HUMIDITY_CALIB_DIG_H5_REG               (0xE5)
#define BME280_HUMIDITY_CALIB_DIG_H6_REG               (0xE7)


void bme280_init( void );
int32_t bme280_read_value( uint8_t TYPE );


#endif
