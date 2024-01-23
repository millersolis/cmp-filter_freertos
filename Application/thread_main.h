/*
 * thread_main.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

#ifndef THREAD_MAIN_H_
#define THREAD_MAIN_H_

#include "app.h"


void Thread_Main_Start(void);
void Thread_Main_AccGyroSensor_Data(AccGyro_Msg_t data);

#endif /* THREAD_MAIN_H_ */
