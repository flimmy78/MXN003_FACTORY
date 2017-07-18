/*
****************************************************************************
* Copyright (C) 2015 - 2016 Bosch Sensortec GmbH
*
* bma2x2_support.c
* Date: 2016/03/11
* Revision: 1.0.3 $
*
* Usage: Sensor Driver support file for  BMA2x2 sensor
*
****************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

/*---------------------------------------------------------------------------*/
/* Includes*/
/*---------------------------------------------------------------------------*/
#include "bma2x2.h"
#include "bma2x2_support.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sensor_drv.h"
#include "twi_master.h"

struct bma2x2_t bma2x2;

int8_t BMA2x2_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
		uint8_t w2_data[2];
		w2_data[0] = reg_addr;
		w2_data[1] = *reg_data;
		return twi_master_transfer(dev_addr,w2_data,2,TWI_ISSUE_STOP);
}

int8_t BMA2x2_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int8_t transfer_succeeded;
	transfer_succeeded = twi_master_transfer(dev_addr, &reg_addr, 1, TWI_DONT_ISSUE_STOP);
	transfer_succeeded &= twi_master_transfer(dev_addr | TWI_READ_BIT, reg_data, cnt, TWI_ISSUE_STOP);
	
	return transfer_succeeded;
}

void BMA2x2_delay_msek(uint32_t msek)
{
	/*Here you can write your own delay routine*/
}

static int8_t I2C_routine(void)
{
	bma2x2.bus_write = BMA2x2_I2C_bus_write;
	bma2x2.bus_read = BMA2x2_I2C_bus_read;
	bma2x2.delay_msec = BMA2x2_delay_msek;
	bma2x2.dev_addr = (BMA2x2_I2C_ADDR1 << 1);
	return BMA2x2_INIT_VALUE;
}

static void sensor_pwr_up(void)
{
	bma2x2_set_power_mode(BMA2x2_MODE_LOWPOWER1);
	bma2x2_set_sleep_durn(BMA2x2_SLEEP_DURN_1S);
}

static void sensor_pwr_down(void)
{
	bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
}



static void sensor_init_with_interrupt(void)
{
		sensor_pwr_up();


		bma2x2_set_range(BMA2x2_RANGE_2G);
		bma2x2_set_bw(BMA2x2_BW_125HZ);
	
		bma2x2_set_latch_intr(BMA2x2_LATCH_DURN_250MS);
	
		bma2x2_set_intr_slope(BMA2x2_INTR1_SLOPE,1);
		bma2x2_set_intr_enable(BMA2x2_SLOPE_X_INTR,1);
		bma2x2_set_intr_enable(BMA2x2_SLOPE_Y_INTR,1);
		bma2x2_set_intr_enable(BMA2x2_SLOPE_Z_INTR,1);
	
		//bma2x2_set_latch_intr(BMA2x2_LATCH_DURN_250MS);
		bma2x2_set_durn(BMA2x2_SLOPE_DURN,1);
		bma2x2_set_thres(BMA2x2_SLOPE_THRES,sensor_config_data.sensor_threshold);

	
		
}

void bma2x2_power_on(uint8_t threshold)
{
	if(threshold <= 0){
		return;
	}
	sensor_config_data.sensor_threshold = threshold;
	sensor_init_with_interrupt();
}


void bma2x2_power_off(void)
{
	sensor_pwr_down();
}

int8_t bm2x2_check_chipid(void){
	I2C_routine();
	/*i2c init*/
	twi_master_init();  
		/*sw rest*/
	bma2x2_soft_rst();
	bma2x2_init(&bma2x2);
	if(bma2x2.chip_id == 0xFA){
		return 1;// bam253
	}else if(bma2x2.chip_id == 0x03) {  //bma250
		return 2;
	}else{
		return -1;
	}
}


#if 0
int32_t bma2x2_data_readout_template(void)
{
		int32_t com_rslt = ERROR;
		u8 bw_value_u8 = BMA2x2_INIT_VALUE;
			
		I2C_routine();
		twi_master_init();  //i2c init
		nrf_delay_ms(10);
		com_rslt = bma2x2_init(&bma2x2);
	
		com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

		bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
		com_rslt += bma2x2_set_bw(bw_value_u8);

		return com_rslt;
}
#endif





