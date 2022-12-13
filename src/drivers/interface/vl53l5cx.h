#ifndef VL53L5CX_H
#define VL53L5CX_H

#include "i2cdev.h"
//#include "vl53l5cx_platform.h"
#include "vl53l5cx_api.h"


typedef struct {
    VL53L5CX_Configuration 	dev;			/* Sensor configuration */
    VL53L5CX_ResultsData 	results;		/* Results data from VL53L5CX */
} VL53L5CX_Dev_t;



/*
typedef struct {
	VL53L1_DevData_t   Data;
	uint32_t  new_data_ready_poll_duration_ms;
} VL53L1_Dev_t;
*/

bool vl53l5cxInit(VL53L5CX_Dev_t* pdev, I2C_Dev* I2Cx);

bool vl53l5cTestConnection(VL53L5CX_Dev_t* pdev);


#endif /* VL53L5CX_H */
