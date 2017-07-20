#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sensor_drv.h"


sensor_config_data_struct sensor_config_data;//= {0};


static sensor_type_enum sensor_type_init(void)
{
	int8_t  result;
	static uint8_t count = 0;
	REREAD:
	result = bm2x2_check_chipid();
	if (result > 0){
		if(result == 1){
			count = 0;
			return SENSOR_BMA253;
		}else if(result == 2){
			count = 0;
			return SENSOR_BMA250;
		}
	}
	count ++;
	if(count < 2){
		goto REREAD;
	}
	count = 0;
	return SENSOR_INIT_ERROR;
}

static void sensor_function_init(sensor_type_enum sensor_type)
{
	switch(sensor_type)
	{
		case SENSOR_BMA253:
			sensor_config_data.sensor_on  = bma2x2_power_on;
			sensor_config_data.sensor_off = bma2x2_power_off;
			break;
		case SENSOR_BMA250:
			break;
		default:
			break;
	}
}

int8_t sensor_type_auto_maching_init(void)
{
		sensor_config_data.sensor_type = sensor_type_init();

		if((sensor_config_data.sensor_type <= SENSOR_NOT_READY) ||\
				(sensor_config_data.sensor_type >= SENSOR_TYPE_MAX)){
				return -1;
		}else{
			sensor_function_init(sensor_config_data.sensor_type);
			return sensor_config_data.sensor_type;
		}
}


sensor_type_enum sensor_get_type(void)
{
	return sensor_config_data.sensor_type;
}

uint8_t sensor_get_level(void)
{
	return sensor_config_data.sensor_level;
}

uint8_t sensor_get_threshold(void)
{
	return sensor_config_data.sensor_threshold;
}

static uint8_t sensor_threshold_convert(sensor_threshold_level level)
{
	sensor_type_enum sensor_type;
	const uint8_t threshold_tab[SENSOR_TYPE_MAX - 1][5] = {
		{15,50,150,200,255},
		{23,25,27,29,31},
	};
	
	sensor_type = sensor_get_type();
	if((sensor_type <= SENSOR_NOT_READY) || (sensor_type >= SENSOR_TYPE_MAX)){
		return 0;
	}
	if((level <= SENSOR_LEVEL_MIN) || (level >= SENSOR_LEVEL_MAX))
	{
		return 0;
	}
	
	return threshold_tab[sensor_type - 1][level -1];
}


int8_t open_sensor_monitor(uint8_t level)
{
	uint8_t threshold;
	sensor_type_enum sensor_type;
	
	if(level <= 0){
		return -1;
	}
	
	sensor_config_data.sensor_level = level;
	
	sensor_type = sensor_get_type();
	
	if((sensor_type < SENSOR_NOT_READY) || sensor_type >= SENSOR_TYPE_MAX){
		return 0;
	}
	threshold = sensor_threshold_convert((sensor_threshold_level)level);
	
	if(threshold == 0){
		return -1;
	}
	
	sensor_config_data.sensor_threshold = threshold;
	if(sensor_config_data.sensor_on != NULL){
		sensor_config_data.sensor_on(threshold);
	}
	
	return 1;
}

uint8_t close_sensor_monitor(void)
{
	sensor_type_enum sensor_type;
	sensor_type = sensor_get_type();
	
	if((sensor_type <= SENSOR_NOT_READY) || (sensor_type >= SENSOR_TYPE_MAX)){
		return 0;
	}
	if(sensor_config_data.sensor_off != NULL){
		sensor_config_data.sensor_off();
	}
	
	return 1;
	
}


