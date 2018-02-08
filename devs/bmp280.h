/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018
 *
 * Balint Cristian <cristian dot balint at gmail dot com>
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

  bmp280.h (Bosch BMP280 temperature and pressure sensor)

*/

#ifndef _BMP280_H
#define _BMP280_H


#include <avr/io.h>
#include <avr/interrupt.h>

#define BMP280_TEMP 0x00
#define BMP280_PRES 0x01

#define BMP280_ADDRESS 0x77

// REGISTER ADDRESS DEFINITION
#define BMP280_CHIP_ID_REG          (0xD0)  //Chip ID Register
#define BMP280_RST_REG              (0xE0)  //Softreset Register
#define BMP280_STAT_REG             (0xF3)  //Status Register
#define BMP280_CTRL_MEAS_REG        (0xF4)  //Ctrl Measure Register
#define BMP280_CONFIG_REG           (0xF5)  //Configuration Register
#define BMP280_PRESSURE_MSB_REG     (0xF7)  //Pressure MSB Register
#define BMP280_PRESSURE_LSB_REG     (0xF8)  //Pressure LSB Register
#define BMP280_PRESSURE_XLSB_REG    (0xF9)  //Pressure XLSB Register
#define BMP280_TEMPERATURE_MSB_REG  (0xFA)  //Temperature MSB Reg
#define BMP280_TEMPERATURE_LSB_REG  (0xFB)  //Temperature LSB Reg
#define BMP280_TEMPERATURE_XLSB_REG (0xFC)  //Temperature XLSB Reg

// BIT LENGTH,POSITION AND MASK DEFINITION

// Status Register
#define BMP280_STATUS_REG_MEASURING__POS               (3)
#define BMP280_STATUS_REG_MEASURING__MSK               (0x08)
#define BMP280_STATUS_REG_MEASURING__LEN               (1)
#define BMP280_STATUS_REG_MEASURING__REG               (BMP280_STAT_REG)

#define BMP280_STATUS_REG_IM_UPDATE__POS               (0)
#define BMP280_STATUS_REG_IM_UPDATE__MSK               (0x01)
#define BMP280_STATUS_REG_IM_UPDATE__LEN               (1)
#define BMP280_STATUS_REG_IM_UPDATE__REG               (BMP280_STAT_REG)

// TEMPERATURE OVERSAMPLING
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS (5)
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK (0xE0)
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN (3)
#define BMP280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__REG (BMP280_CTRL_MEAS_REG)

//FOR PRESSURE OVERSAMPLING
#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS    (2)
#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK    (0x1C)
#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN    (3)
#define BMP280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__REG    (BMP280_CTRL_MEAS_REG)

// FOR POWER MODE
#define BMP280_CTRL_MEAS_REG_POWER_MODE__POS           (0)
#define BMP280_CTRL_MEAS_REG_POWER_MODE__MSK           (0x03)
#define BMP280_CTRL_MEAS_REG_POWER_MODE__LEN           (2)
#define BMP280_CTRL_MEAS_REG_POWER_MODE__REG           (BMP280_CTRL_MEAS_REG)

// FOR STANDBY DURATION
#define BMP280_CONFIG_REG_STANDBY_DURN__POS            (5)
#define BMP280_CONFIG_REG_STANDBY_DURN__MSK            (0xE0)
#define BMP280_CONFIG_REG_STANDBY_DURN__LEN            (3)
#define BMP280_CONFIG_REG_STANDBY_DURN__REG            (BMP280_CONFIG_REG)

// FOR IIR FILTER
#define BMP280_CONFIG_REG_FILTER__POS                  (2)
#define BMP280_CONFIG_REG_FILTER__MSK                  (0x1C)
#define BMP280_CONFIG_REG_FILTER__LEN                  (3)
#define BMP280_CONFIG_REG_FILTER__REG                  (BMP280_CONFIG_REG)

//FOR SPI ENABLE
#define BMP280_CONFIG_REG_SPI3_ENABLE__POS             (0)
#define BMP280_CONFIG_REG_SPI3_ENABLE__MSK             (0x01)
#define BMP280_CONFIG_REG_SPI3_ENABLE__LEN             (1)
#define BMP280_CONFIG_REG_SPI3_ENABLE__REG             (BMP280_CONFIG_REG)

//FOR PRESSURE AND TEMPERATURE DATA REGISTERS
#define BMP280_PRESSURE_XLSB_REG_DATA__POS             (4)
#define BMP280_PRESSURE_XLSB_REG_DATA__MSK             (0xF0)
#define BMP280_PRESSURE_XLSB_REG_DATA__LEN             (4)
#define BMP280_PRESSURE_XLSB_REG_DATA__REG             (BMP280_PRESSURE_XLSB_REG)

#define BMP280_TEMPERATURE_XLSB_REG_DATA__POS          (4)
#define BMP280_TEMPERATURE_XLSB_REG_DATA__MSK          (0xF0)
#define BMP280_TEMPERATURE_XLSB_REG_DATA__LEN          (4)
#define BMP280_TEMPERATURE_XLSB_REG_DATA__REG          (BMP280_TEMPERATURE_XLSB_REG)


#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG        (0x88)
#define BMP280_TEMPERATURE_CALIB_DIG_T1_MSB_REG        (0x89)
#define BMP280_TEMPERATURE_CALIB_DIG_T2_LSB_REG        (0x8A)
#define BMP280_TEMPERATURE_CALIB_DIG_T2_MSB_REG        (0x8B)
#define BMP280_TEMPERATURE_CALIB_DIG_T3_LSB_REG        (0x8C)
#define BMP280_TEMPERATURE_CALIB_DIG_T3_MSB_REG        (0x8D)
#define BMP280_PRESSURE_CALIB_DIG_P1_LSB_REG           (0x8E)
#define BMP280_PRESSURE_CALIB_DIG_P1_MSB_REG           (0x8F)
#define BMP280_PRESSURE_CALIB_DIG_P2_LSB_REG           (0x90)
#define BMP280_PRESSURE_CALIB_DIG_P2_MSB_REG           (0x91)
#define BMP280_PRESSURE_CALIB_DIG_P3_LSB_REG           (0x92)
#define BMP280_PRESSURE_CALIB_DIG_P3_MSB_REG           (0x93)
#define BMP280_PRESSURE_CALIB_DIG_P4_LSB_REG           (0x94)
#define BMP280_PRESSURE_CALIB_DIG_P4_MSB_REG           (0x95)
#define BMP280_PRESSURE_CALIB_DIG_P5_LSB_REG           (0x96)
#define BMP280_PRESSURE_CALIB_DIG_P5_MSB_REG           (0x97)
#define BMP280_PRESSURE_CALIB_DIG_P6_LSB_REG           (0x98)
#define BMP280_PRESSURE_CALIB_DIG_P6_MSB_REG           (0x99)
#define BMP280_PRESSURE_CALIB_DIG_P7_LSB_REG           (0x9A)
#define BMP280_PRESSURE_CALIB_DIG_P7_MSB_REG           (0x9B)
#define BMP280_PRESSURE_CALIB_DIG_P8_LSB_REG           (0x9C)
#define BMP280_PRESSURE_CALIB_DIG_P8_MSB_REG           (0x9D)
#define BMP280_PRESSURE_CALIB_DIG_P9_LSB_REG           (0x9E)
#define BMP280_PRESSURE_CALIB_DIG_P9_MSB_REG           (0x9F)


void bmp280_init( void );
int32_t bmp280_read_value( uint8_t TYPE );


#endif
