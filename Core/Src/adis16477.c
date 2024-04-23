/*
 * adis16477.c
 *
 *  Created on: Jul 18, 2023
 *      Author: shahab
 */

#include "adis16477.h"
#include "spi.h"
#include "gpio.h"
  int8_t ADIS_16477_Reset(struct ADIS *dev){

  int8_t rslt;
  uint16_t len 				= 1;
  uint16_t data 			= 0;
  uint16_t MSC_Byte			= 0x41;		//  01000001
  uint16_t SOFTWARE_RESET	= 0x80; 	//  bit 7

      // Start up time
      dev->delay(252);
      rslt = dev->write(GLOB_CMD, &SOFTWARE_RESET, len);
      dev->delay(193);
      rslt = dev->read(MSC_CTRL, &data, len);
      if(data == 0xC1){
      rslt = dev->write(MSC_CTRL, &MSC_Byte, len);
      dev->delay(1);
      }
      rslt = dev->read(MSC_CTRL, &data, len);
      if(data != MSC_Byte){ return -1;}
      return rslt;
  }

  int8_t ADIS_16477_Init(struct ADIS *dev){

    int8_t rslt;
    uint16_t Chip_ID 	= 0;

       rslt = fac_calib_restore(dev);
       rslt = ADIS_16477_Reset(dev);

      // Chip Id checks
      for (int i = 0; i < 2; i++) {
      rslt = dev->read(PROD_ID, &Chip_ID, 1);}
      if(Chip_ID != ADIS_CHIP_ID){return -9;}
      dev->delay(10);

	  rslt = bias_correction(dev);
	  rslt = self_test_sensor(dev);
	  rslt = flash_mem_update(dev);

return rslt;
  }

  int8_t self_test_memory(struct ADIS *dev){

    int8_t rslt;
    uint16_t data 		= 0;

    uint16_t FLASH_MEMORY_TEST	= 0x10; // GLOB_CMD bit 4

	 rslt = dev->write(GLOB_CMD, &FLASH_MEMORY_TEST, 1);

		dev->delay(32);

         rslt = dev->read(DIAG_STAT, &data, 1);

         if(data != 0x0000){return -9;}
                 dev->delay(2);

    return rslt;
  }
  int8_t self_test_sensor(struct ADIS *dev){

    int8_t rslt;
    uint16_t data 		= 0;

    uint16_t SENSOR_SELF_TEST	= 0x04; // GLOB_CMD bit 2

	  rslt = dev->write(GLOB_CMD, &SENSOR_SELF_TEST, 1);

    		dev->delay(14);

          rslt = dev->read(DIAG_STAT, &data, 1);

          if(data != 0x0000){return -9;}
                     dev->delay(1);

        return rslt;
  }
  int8_t fac_calib_restore(struct ADIS *dev){

          int8_t rslt;
          uint16_t data 		= 0;

          uint16_t FAC_CALIB_RESTORE	= 0x02; // GLOB_CMD bit 1

      	 rslt = dev->write(GLOB_CMD, &FAC_CALIB_RESTORE, 1);

      		dev->delay(142);

               rslt = dev->read(DIAG_STAT, &data, 1);

               if(data != 0x0000){return -9;}
                       dev->delay(1);

          return rslt;
        }
  int8_t flash_mem_update(struct ADIS *dev){

        int8_t rslt;
        uint16_t data 		= 0;

        uint16_t FLASH_MEMORY_UPDATE	= 0x08; // GLOB_CMD bit 3

    	 rslt = dev->write(GLOB_CMD, &FLASH_MEMORY_UPDATE, 1);

    		dev->delay(72);

             rslt = dev->read(DIAG_STAT, &data, 1);

             if(data != 0x0000){return -9;}
                     dev->delay(1);

        return rslt;
      }
  int8_t bias_correction(struct ADIS *dev){

      int8_t rslt;
      uint16_t data 		= 0;

      uint16_t BIAS_CORRECTION	= 0x01; // GLOB_CMD bit 0

  	 rslt = dev->write(GLOB_CMD, &BIAS_CORRECTION, 1);

  		dev->delay(300);

           rslt = dev->read(DIAG_STAT, &data, 1);

           if(data != 0x0000){return -9;}
                   dev->delay(1);

      return rslt;
    }
  int8_t Burst_Read_16(struct ADIS *dev){

  int8_t rslt;
  uint16_t sum = 0;
  uint16_t burstdata_16[10];
  uint16_t reg_addr = GLOB_CMD;


  rslt = dev->burst_read(reg_addr, burstdata_16, 10);

	  dev->diag_stat = burstdata_16[0];
 	  dev->gyro_x 	 = burstdata_16[1];
 	  dev->gyro_y 	 = burstdata_16[2];
 	  dev->gyro_z 	 = burstdata_16[3];
 	  dev->accel_x	 = burstdata_16[4];
 	  dev->accel_y 	 = burstdata_16[5];
 	  dev->accel_z 	 = burstdata_16[6];
 	  dev->temp 	 = burstdata_16[7];
 	  dev->DATA_CNTR = burstdata_16[8];
 	  dev->checksum  = burstdata_16[9];

 	 sum =  checksum(dev);
 	  if(dev->checksum != sum){
 	   return -1;
 	   }
  return rslt;
}

  int16_t checksum(struct ADIS *dev) {
  int16_t sum = 0;
  uint16_t burstArray [9] = {0};
  uint16_t *checksum_helper = (uint16_t *)&dev->diag_stat;

  for(int j = 0; j < 9 ; j++){
      burstArray [j] = checksum_helper[j];
  }

  for (int i = 0; i < 9; i++)
  {
      sum += (burstArray[i] & 0xFF); // Lower byte
      sum += ((burstArray[i] >> 8) & 0xFF); // Upper byte
  }
  return sum;
}

double Scale_accel(int16_t sensor_val)
{
  double rslt = sensor_val * 0.00125;
  return rslt;
}

double Scale_gyro(int16_t sensor_val)
{
  double rslt = sensor_val * 0.1;
  return rslt;
}

double Scale_temp(int16_t sensor_val)
{
  double rslt = (sensor_val * 0.1);
  return rslt;
}
