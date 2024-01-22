/*
 * app.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

#include "app.h"
#include "thread_main.h"
#include "thread_i2c.h"


void App_Start(void)
{
	Thread_Main_Start();
	Thread_I2C_Start();
}
