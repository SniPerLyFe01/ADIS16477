/*
 * adis16477.h
 *
 *  Created on: Jul 18, 2023
 *      Author: shahab
 */

#ifndef APPLICATION_USER_CORE_INC_ADIS16477_H_
#define APPLICATION_USER_CORE_INC_ADIS16477_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

// Table 6: Memory Map from datasheet
#define FLASH_CNT   	0x00
#define DIAG_STAT   	0x02
#define X_GYRO_LOW  	0x04
#define X_GYRO_OUT  	0x06
#define Y_GYRO_LOW  	0x08
#define Y_GYRO_OUT  	0x0A
#define Z_GYRO_LOW  	0x0C
#define Z_GYRO_OUT  	0x0E
#define X_ACCL_LOW  	0x10
#define X_ACCL_OUT  	0x12
#define Y_ACCL_LOW  	0x14
#define Y_ACCL_OUT  	0x16
#define Z_ACCL_LOW  	0x18
#define Z_ACCL_OUT  	0x1A
#define TEMP_OUT    	0x1C
#define TIME_STAMP  	0x1E
#define X_DELTANG_LOW	0x24
#define X_DELTANG_OUT	0x26
#define Y_DELTANG_LOW	0x28
#define Y_DELTANG_OUT	0x2A
#define Z_DELTANG_LOW	0x2C
#define Z_DELTANG_OUT	0x2E
#define X_DELTVEL_LOW	0x30
#define X_DELTVEL_OUT	0x32
#define Y_DELTVEL_LOW	0x34
#define Y_DELTVEL_OUT	0x36
#define Z_DELTVEL_LOW	0x38
#define Z_DELTVEL_OUT	0x3A
#define XG_BIAS_LOW		0x40
#define XG_BIAS_HIGH	0x42
#define YG_BIAS_LOW		0x44
#define YG_BIAS_HIGH	0x46
#define ZG_BIAS_LOW		0x48
#define ZG_BIAS_HIGH	0x4A
#define XA_BIAS_LOW		0x4C
#define XA_BIAS_HIGH	0x4E
#define YA_BIAS_LOW		0x50
#define YA_BIAS_HIGH	0x52
#define ZA_BIAS_LOW		0x54
#define ZA_BIAS_HIGH	0x56
#define FILT_CTRL    	0x5C
#define RANG_MDL		0x5E
#define MSC_CTRL    	0x60
#define UP_SCALE    	0x62
#define DEC_RATE    	0x64
#define NULL_CFG    	0x66
#define GLOB_CMD    	0x68
#define FIRM_REV    	0x6C
#define FIRM_DM    		0x6E
#define FIRM_Y    		0x70
#define PROD_ID    		0x72
#define SERIAL_NUM      0x74
#define USER_SCR1    	0x76
#define USER_SCR2    	0x78
#define USER_SCR3    	0x7A
#define FLASHCNT_LOW    0x7C
#define FLASHCNT_HIGH   0x7E

#define ADIS_CHIP_ID	0x405D

// Here change the Chip_Select and Reset pins according to your Hardware setup
// and the SPI handle
#define ADIS_SPI		(&hspi2)
#define ADIS_CS			GPIOD
#define ADIS_CS_PIN		GPIO_PIN_14
#define HARD_RST		GPIOF
#define HARD_RST_PIN	GPIO_PIN_3


  typedef int8_t 	(*read_fptr_t)			(uint16_t reg_addr, uint16_t *read_data, uint16_t len);
  typedef int8_t 	(*burst_read_fptr_t)	(uint16_t reg_addr, uint16_t *read_data, uint16_t len);
  typedef int8_t 	(*write_fptr_t)			(uint16_t reg_addr, const uint16_t *read_data, uint16_t len);
  typedef void 		(*delay_fptr_t)			(uint32_t period);

 struct ADIS {
         uint16_t     diag_stat;
         uint16_t     gyro_x;
         uint16_t     gyro_y;
         uint16_t     gyro_z;
         uint16_t     accel_x;
         uint16_t     accel_y;
         uint16_t     accel_z;
         uint16_t     temp;
         uint16_t     DATA_CNTR;
         uint16_t     checksum;

         read_fptr_t read;
         burst_read_fptr_t burst_read;
         write_fptr_t write;
         delay_fptr_t delay;
     };

 int8_t ADIS_16477_Reset(struct ADIS *dev);
 int8_t ADIS_16477_Init(struct ADIS *dev);
 int8_t self_test_memory(struct ADIS *dev);
 int8_t self_test_sensor(struct ADIS *dev);
 int8_t fac_calib_restore(struct ADIS *dev);
 int8_t flash_mem_update(struct ADIS *dev);
 int8_t bias_correction(struct ADIS *dev);

 int8_t Burst_Read_16(struct ADIS *dev);
 int16_t checksum(struct ADIS *dev);
 double Scale_accel(int16_t sensor_val);
 double Scale_gyro(int16_t sensor_val);
 double Scale_temp(int16_t sensor_val);

#ifdef __cplusplus
}
#endif

#endif
