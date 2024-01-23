/*
 * thread_main.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

//----------------------------------------------------------------------
// Includes
#include "thread_main.h"
#include "bsp_terminal.h"

#include <RCfilter.h>
#include <printf/printf.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

//----------------------------------------------------------------------
// Defines
#define  DEG_TO_RAD			0.0174532925f

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

//----------------------------------------------------------------------
// Queue Defines
#define MAIN_QUEUE_MSG_NB		8
#define MAIN_QUEUE_MSG_SIZE		(sizeof(AccGyro_Msg_t))
#define MAIN_QUEUE_STORAGE		(MAIN_QUEUE_MSG_NB * MAIN_QUEUE_MSG_SIZE)

//----------------------------------------------------------------------
// Queue Local Variables
static QueueHandle_t 	q_main;   							// queue handle
static StaticQueue_t 	qbuf_main;  						// static queue buffer
static uint8_t 			qs_main[MAIN_QUEUE_STORAGE] = {0};  // storage buffer

//----------------------------------------------------------------------
// Function Definitions
static void Thread_Main_Run(void *args);

//----------------------------------------------------------------------
// Function Implementations

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

    //-------------------------------------------------
    // Create queue
    q_main = xQueueCreateStatic(MAIN_QUEUE_MSG_NB,
    							MAIN_QUEUE_MSG_SIZE,
								&qs_main[0],
								&qbuf_main);
    configASSERT(q_main);
}

void Thread_Main_AccGyroSensor_Data(AccGyro_Msg_t data)
{
	AccGyro_Msg_t msg = data;
	if (xQueueSendToBack(q_main, &msg, 0) != pdTRUE) {
		Error_Handler();
	}
}

static void test_rx_q(AccGyro_Msg_t data)
{
	static uint8_t tx_buffer[1000];
	static Gyro_Data_t gyro_dat;

	gyro_dat = data.gyro;

	sprintf_((char *)tx_buffer,
		  "p=%4.2f,q=%4.2f,r=%4.2f\r\n",
		  gyro_dat.p, gyro_dat.q, gyro_dat.r);

	tx_com(tx_buffer, strlen((char const *)tx_buffer));
}

static void Thread_Main_Run(void *args)
{
	// Thread message loop
	// Receive data for complementary filer implementation/calculations
	AccGyro_Msg_t msg;
    while (true) {
        if (xQueueReceive(q_main, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {

            // tests
            test_rx_q(msg);
        }
    }

}

void acc_angle_estimate(void)
{


	static RCFilter lpfAcc[3];
	static RCFilter lpfGyr[3];

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
