/*
 * app.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

#ifndef APP_H_
#define APP_H_

#include "bsp_accGyr.h"

typedef struct {
	Acc_Data_t 		acc;
	Gyro_Data_t 	gyro;
	uint32_t 		timestamp;	// ms
	_Bool			dataOK_flag;
}AccGyro_Msg_t;


void App_Start(void);
void App_AccGyroSensor_Data(AccGyro_Msg_t data);


#endif /* APP_H_ */
