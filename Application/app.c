/*
 * app.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

//----------------------------------------------------------------------
// Includes
#include "app.h"
#include "thread_main.h"
#include "thread_i2c.h"
#include <printf/printf.h>

//----------------------------------------------------------------------
// Function Implementations
void App_Start(void)
{
	Thread_Main_Start();
	Thread_I2C_Start();
}

void App_AccGyroSensor_Data(AccGyro_Msg_t data)
{
	// Send to main thread to use for complementary filter
	// implementation/calculations
	Thread_Main_AccGyroSensor_Data(data);
}
