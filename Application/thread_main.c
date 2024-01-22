/*
 * thread_main.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

//----------------------------------------------------------------------
// Includes
#include "thread_main.h"
#include "usart.h"
#include "bsp_terminal.h"

#include <RCfilter.h>
#include <printf/printf.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdint.h>
#include <string.h>

//----------------------------------------------------------------------
// Defines


//----------------------------------------------------------------------
// Task Defines
#define MAIN_STACK_DEPTH		(8 * 1024)
#define MAIN_TASK_NAME			("main")
#define MAIN_TASK_PRIO			5


//----------------------------------------------------------------------
// Task Local Variables
static TaskHandle_t 		taskHandle = {0};
static const char* const 	taskName = MAIN_TASK_NAME;
static UBaseType_t			taskPriority = MAIN_TASK_PRIO;
static const uint32_t 		stackDepth = MAIN_STACK_DEPTH;
static StackType_t 			stackBuffer[MAIN_STACK_DEPTH];
static StaticTask_t 		taskBuffer;

static void Thread_Main_Run(void *args);


void Thread_Main_Start(void)
{
    //-------------------------------------------------
    // Create task
	taskHandle = xTaskCreateStatic(
				Thread_Main_Run,
				taskName,
				stackDepth,
				NULL,
				taskPriority,
				&stackBuffer[0],
				&taskBuffer);

    configASSERT(taskHandle);


}

void test_oscilloscope(void)
{
	static uint8_t tx_buffer[1000];

	for (int i = 0; ; i++) {
		sprintf_((char *)tx_buffer,
				  "%4.2f,%4.2f,%4.2f\r",
				  (double)(i%100), (double)(i%100), (double)(i%100));
//		  tx_com(tx_buffer, strlen((char const *)tx_buffer));

		  vTaskDelay(100);
	}
}

void acc_angle_estimate(void)
{
#define SAMPLE_TIME_MS		20

#define  DEG_TO_RAD			0.0174532925f

	RCFilter lpfAcc[3];
	RCFilter lpfGyr[3];

	float rollEstimate_rad;
	float pitchEstimate_rad;

	for (uint8_t i = 0; i < 3; i++) {
		RCFilter_Init(&lpfAcc[i], 5.0f, 0.01f);
		RCFilter_Init(&lpfAcc[i], 25.0f, 0.01f);
	}

	rollEstimate_rad = 0.0f;
	pitchEstimate_rad = 0.0f;

	float phiHat_deg 	= 0.0f;
	float thetaHat_deg 	= 0.0f;

	while (1) {

//		if ((HAL_GetTick()- timerI2C) >= SAMPLE_TIME_MS) {
//			float ax_mps2 = lpfAcc[0].out[0];
//			float ay_mps2 = lpfAcc[0].out[0];
//			float az_mps2 = lpfAcc[0].out[0];
//		}
	}

}

static void Thread_Main_Run(void *args)
{
	test_oscilloscope();
}




