/*
 * thread_main.c
 *
 *  Created on: Jan 21, 2024
 *      Author: Miller Solis
 */

#include "thread_main.h"
#include "usart.h"
#include <printf/printf.h>

#include <FreeRTOS.h>
#include <task.h>

#include <stdint.h>
#include <string.h>


#define MAIN_STACK_DEPTH		(8 * 1024)
#define MAIN_TASK_NAME			("main")
#define MAIN_TASK_PRIO			5

#define	SERIAL_COM				huart3


static TaskHandle_t 		taskHandle = {0};
static const char* const 	taskName = MAIN_TASK_NAME;
static const uint32_t 		stackDepth = MAIN_STACK_DEPTH;
static StackType_t 			stackBuffer[MAIN_STACK_DEPTH];
static StaticTask_t 		taskBuffer;


static void Thread_Main_Run(void *args);


void Thread_Main_Start(void)
{

	taskHandle = xTaskCreateStatic(Thread_Main_Run,
			taskName, stackDepth, NULL, MAIN_TASK_PRIO,
			&stackBuffer[0], &taskBuffer);

    // Check that task was successfully created
//    ASSERT(taskHandle);

}

static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
  HAL_UART_Transmit(&SERIAL_COM, tx_buffer, len, 1000);
}

void test_oscilloscope(void)
{
	static uint8_t tx_buffer[1000];

	for (int i = 0; ; i++) {
		sprintf_((char *)tx_buffer,
				  "%4.2f,%4.2f,%4.2f\r",
				  (double)(i%100), (double)(i%100), (double)(i%100));
		  tx_com(tx_buffer, strlen((char const *)tx_buffer));

		  vTaskDelay(100);
	}
}

static void Thread_Main_Run(void *args)
{
	test_oscilloscope();
}


