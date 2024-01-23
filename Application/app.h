/*
 * app.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

#ifndef APP_H_
#define APP_H_

//----------------------------------------------------------------------
// Includes
#include "bsp_accGyr.h"

//----------------------------------------------------------------------
// Type Definitions
typedef struct {
	Acc_Data_t 		acc_mps2;
	Gyro_Data_t 	gyro_degps;
	uint32_t 		timestamp_ms;	// ms
	_Bool			dataOK_flag;
}AccGyro_Msg_t;

//----------------------------------------------------------------------
// Function Definitions
void App_Start(void);
void App_AccGyroSensor_Data(AccGyro_Msg_t data);


#endif /* APP_H_ */
