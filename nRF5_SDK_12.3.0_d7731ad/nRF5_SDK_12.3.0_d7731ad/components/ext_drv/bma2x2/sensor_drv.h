#ifndef  __SENSOR_DRV_H
#define __SENSOR_DRV_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bma2x2_support.h"

typedef enum{
	SENSOR_INIT_ERROR = -1,   //Unrecognized
	SENSOR_NOT_READY ,
	SENSOR_BMA253,
	SENSOR_BMA250,   //Not supported
	SENSOR_TYPE_MAX,
}sensor_type_enum;


typedef enum{
	SENSOR_LEVEL_MIN,
	SENSOR_LEVEL_1,
	SENSOR_LEVEL_2,
	SENSOR_LEVEL_3,
	SENSOR_LEVEL_4,
	SENSOR_LEVEL_5,
	SENSOR_LEVEL_MAX,
}sensor_threshold_level;

typedef struct{
	sensor_type_enum sensor_type;
	uint8_t sensor_level;
	uint8_t sensor_threshold;
	void (*sensor_on)(uint8_t level);
	void (*sensor_off)(void);
}sensor_config_data_struct;


//Global data
extern sensor_config_data_struct sensor_config_data;

//Global function
/*Configure and initialize*/
int8_t sensor_type_auto_maching_init(void);

/*open sensor Monitor*/
int8_t open_sensor_monitor(uint8_t level);

/*close sensor Monitor*/
uint8_t close_sensor_monitor(void);

/*call back handle*/
void sensor_callback_handle(void);

sensor_type_enum sensor_get_type(void);
#endif

