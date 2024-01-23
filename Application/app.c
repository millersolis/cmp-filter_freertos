/*
 * app.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

#include "app.h"
#include "thread_main.h"
#include "thread_i2c.h"
#include "bsp_terminal.h"// temp for testing
#include <printf/printf.h>


void App_Start(void)
{
	Thread_Main_Start();
	Thread_I2C_Start();
}

static void test_rx_q(AccGyro_Msg_t data)
{
	static uint8_t tx_buffer[1000];
	static Gyro_Data_t gyro_dat;

	gyro_dat = data.gyro_degps;

	sprintf_((char *)tx_buffer,
		  "p=%4.2f,q=%4.2f,r=%4.2f\r\n",
		  gyro_dat.p, gyro_dat.q, gyro_dat.r);

	tx_com(tx_buffer, strlen((char const *)tx_buffer));
}

void App_AccGyroSensor_Data(AccGyro_Msg_t data)
{
	// Send to main thread to use for complementary filter
	// implementation/calculations
	Thread_Main_AccGyroSensor_Data(data);

	// test
//	test_rx_q(data);

}


