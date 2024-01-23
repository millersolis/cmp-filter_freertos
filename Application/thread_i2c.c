/*
 * thread_i2c.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

//----------------------------------------------------------------------
// Includes
#include "thread_i2c.h"
#include "i2c.h"
#include "bsp_accGyr.h"
#include "bsp_terminal.h"	//temp for testing
#include "app.h"
#include <printf/printf.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

#include <stdint.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>

//----------------------------------------------------------------------
// Defines
#define ACCGYR_SAMPLE_INTERVAL_MS		100//ms -> 10Hz
#define pdTICKS_TO_MS(ticks)	((TickType_t)(ticks) * (1000U/configTICK_RATE_HZ))

//----------------------------------------------------------------------
// Task Defines
#define I2C_STACK_DEPTH			(8 * 1024)
#define I2C_TASK_NAME			("i2c")
#define I2C_TASK_PRIO			4

//----------------------------------------------------------------------
// Task Local Variables
static TaskHandle_t 		taskHandle = {0};
static const char* const 	taskName = I2C_TASK_NAME;
static UBaseType_t			taskPriority = I2C_TASK_PRIO;
static const uint32_t 		stackDepth = I2C_STACK_DEPTH;
static StackType_t 			stackBuffer[I2C_STACK_DEPTH];
static StaticTask_t 		taskBuffer;

//----------------------------------------------------------------------
// Queue Local Types
typedef enum {
	None = 0,
	AccGyrSensorTimerExpired,
} I2C_Msg_ID;

typedef uint8_t	msg_t;

//----------------------------------------------------------------------
// Queue Defines
#define I2C_QUEUE_MSG_NB		8
#define I2C_QUEUE_MSG_SIZE		(sizeof(msg_t))
#define I2C_QUEUE_STORAGE		(I2C_QUEUE_MSG_NB * I2C_QUEUE_MSG_SIZE)

//----------------------------------------------------------------------
// Queue Local Variables
static QueueHandle_t 	q_i2c;   							// queue handle
static StaticQueue_t 	qbuf_i2c;  							// static queue buffer
static uint8_t 			qs_i2c[I2C_QUEUE_STORAGE] = {0};  	// storage buffer

// Flags are set to true when there are messages of the corresponding
// type waiting in the queue. Could prevent flooding the message
// queue with too many messages.
_Bool accGyrSensorMsgWaiting = false;

//----------------------------------------------------------------------
// Timers Local Definitions
static TimerHandle_t accGyrSensorTimer = NULL;
static StaticTimer_t accGyrSensorTimerBuffer = {};
void accGyrSensorTimerCallback(TimerHandle_t xTimer);

//----------------------------------------------------------------------
// Function Definitions
static void Thread_I2C_Run(void *args);
static void handleMsg(msg_t msg);
static _Bool waitForNotify(uint32_t ms);
void I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

//----------------------------------------------------------------------
// Function Implementations

void Thread_I2C_Start(void)
{
    //-------------------------------------------------
    // Create task
	taskHandle = xTaskCreateStatic(
				Thread_I2C_Run,
				taskName,
				stackDepth,
				NULL,
				taskPriority,
				&stackBuffer[0],
				&taskBuffer);

    configASSERT(taskHandle);

    //-------------------------------------------------
    // Create timer

    // Timer to sample acc and gyr data
    accGyrSensorTimer = xTimerCreateStatic(
                "i2c_ReadAccGyrSensor",
                pdMS_TO_TICKS(ACCGYR_SAMPLE_INTERVAL_MS),
                pdTRUE,
				(void *) 0,
				accGyrSensorTimerCallback,
                &accGyrSensorTimerBuffer);

    configASSERT(accGyrSensorTimer);

    //-------------------------------------------------
    // Create queue

    // Using queue instead of semaphore to be able to use
    // with more timer callbacks in the future if needed.
    q_i2c = xQueueCreateStatic(I2C_QUEUE_MSG_NB,
    							I2C_QUEUE_MSG_SIZE,
								&qs_i2c[0],
								&qbuf_i2c);
    configASSERT(q_i2c);

}

static void test_acc(void)
{
	static uint8_t tx_buffer[1000];
	static Acc_Data_t acc_dat;


	for (int i = 0; ; i++) {
		BSP_Acc_GetData(&acc_dat);

		sprintf_((char *)tx_buffer,
			  "x=%4.2f,y=%4.2f,z=%4.2f\r\n",
			  acc_dat.x, acc_dat.y, acc_dat.z);

		tx_com(tx_buffer, strlen((char const *)tx_buffer));

		vTaskDelay(pdMS_TO_TICKS(ACCGYR_SAMPLE_INTERVAL_MS));
	}
}

static void test_gyro(void)
{
	static uint8_t tx_buffer[1000];
	static Gyro_Data_t gyro_dat;


	for (int i = 0; ; i++) {
		BSP_Gyro_GetData(&gyro_dat);

		sprintf_((char *)tx_buffer,
			  "p=%4.2f,q=%4.2f,r=%4.2f\r\n",
			  gyro_dat.p, gyro_dat.q, gyro_dat.r);

		tx_com(tx_buffer, strlen((char const *)tx_buffer));

		vTaskDelay(pdMS_TO_TICKS(ACCGYR_SAMPLE_INTERVAL_MS));
	}
}


// Only a single thread to hanlde all I2C bus operations
static void Thread_I2C_Run(void *args)
{
	extern I2C_HandleTypeDef hi2c2;

	// Register Callbacks
	HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_MEM_TX_COMPLETE_CB_ID,
							 I2C_MemTxCpltCallback);
	HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_MEM_RX_COMPLETE_CB_ID,
							 I2C_MemRxCpltCallback);

	// Initialize Accel and Gyro IC
	BSP_AccGyr_Init();

	// Start Timer
	if (xTimerStart(accGyrSensorTimer, pdMS_TO_TICKS(100)) != pdTRUE) {
		Error_Handler();
	}

	//Thread message loop
	msg_t msg;
    while (true) {
        if (xQueueReceive(q_i2c, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {
            handleMsg(msg);
        }
    }

	 // tests
//	test_acc();
//	test_gyro();
}

static void handleMsg(msg_t msg)
{
	switch (msg) {

	    case AccGyrSensorTimerExpired: {
	    	static AccGyro_Msg_t data_app;

	    	accGyrSensorMsgWaiting = false;

	    	data_app.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());

	    	_Bool success = BSP_AccGyr_GetData(&(data_app.acc), &(data_app.gyro));

	    	if (!success) {
	    		// set error flag
	    		data_app.dataOK_flag = false;

	    		// Make corrupted data easier to spot
	    		data_app.acc.x = -999.0f;
	    		data_app.acc.y = -999.0f;
	    		data_app.acc.z = -999.0f;

	    		data_app.gyro.p = -999.0f;
	    		data_app.gyro.q = -999.0f;
	    		data_app.gyro.r = -999.0f;
	    	}
	    	else {
	    		// unset error flag
				data_app.dataOK_flag = true;
	    	}

	    	// Send data to app
	    	App_AccGyroSensor_Data(data_app);
	    } break;

	    // Add more as needed

	    default:
	        	break;
	}
}

static _Bool waitForNotify(uint32_t ms)
{
	uint32_t n = 0;
    BaseType_t result = xTaskNotifyWait(0, ULONG_MAX, &n, pdMS_TO_TICKS(ms));
    return result == pdTRUE;
}

void accGyrSensorTimerCallback(TimerHandle_t xTimer)
{
	if (!accGyrSensorMsgWaiting) {
		BaseType_t yield = pdFALSE;
		accGyrSensorMsgWaiting = true;
		msg_t msg = AccGyrSensorTimerExpired;
		if (xQueueSendToBackFromISR(q_i2c, (void*)&msg, &yield) != pdTRUE) {
			Error_Handler();
		}
		portYIELD_FROM_ISR(yield);
	}
}

//----------------------------------------------------------------------
// BSP Functions
void BSP_AccGyr_Delay(uint32_t ms)
{
	uint32_t delayTicks = pdMS_TO_TICKS(ms);
	vTaskDelay(delayTicks);
}

_Bool BSP_WaitForRxTx(uint32_t ms)
{
	return waitForNotify(ms);
}

//----------------------------------------------------------------------
// I2C Callbacks
void I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  BaseType_t yield = pdFALSE;
  // Use the eNoAction flag to simply wake the thread once the
  // I2C transmit operation completes.
  // According to FreeRTOS documentation, this is faster than
  // using a binary semaphore.
  xTaskGenericNotifyFromISR(taskHandle, 0, eNoAction, NULL, &yield);
  portYIELD_FROM_ISR(yield);
}

void I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);

  BaseType_t yield = pdFALSE;
  // Use the eNoAction flag to simply wake the thread once the
  // I2C receive operation completes.
  // According to FreeRTOS documentation, this is faster than
  // using a binary semaphore.
  xTaskGenericNotifyFromISR(taskHandle, 0, eNoAction, NULL, &yield);
  portYIELD_FROM_ISR(yield);
}
