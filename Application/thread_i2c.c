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
#include <printf/printf.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include <stdint.h>
#include <string.h>
#include <limits.h>

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
// Timers Local Definitions
static TimerHandle_t accGyrSensorTimer = NULL;
static StaticTimer_t accGyrSensorTimerBuffer = {};
void accGyrSensorTimerCallback(TimerHandle_t xTimer);


static void Thread_I2C_Run(void *args);
static _Bool waitForNotify(uint32_t ms);

void I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);


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
    // Create timers

    // Timer to sample acc and gyr data
    accGyrSensorTimer = xTimerCreateStatic(
                "i2c_ReadAccGyrSensor",
                pdMS_TO_TICKS(2000),
                pdTRUE,
				(void *) 0,
				accGyrSensorTimerCallback,
                &accGyrSensorTimerBuffer);

    configASSERT(accGyrSensorTimer);

}

void test(void)
{
	static uint8_t tx_buffer[1000];
	static Acc_Data_t acc_dat;


	for (int i = 0; ; i++) {
	BSP_Acc_GetData(&acc_dat);

	sprintf_((char *)tx_buffer,
		  "x=%4.2f,y=%4.2f,z=%4.2f\r\n",
		  acc_dat.x, acc_dat.y, acc_dat.z);

#include "bsp_terminal.h"
	tx_com(tx_buffer, strlen((char const *)tx_buffer));

	vTaskDelay(100);
	}
}
//void poll_samples (void)
//{
//	 /* Read samples in polling mode (no int) */
//	  while (1) {
//	    lsm6dsm_reg_t reg;
//	    /* Read output only if new value is available */
//	    lsm6dsm_status_reg_get(&dev_ctx, &reg.status_reg);
//
//	    if (reg.status_reg.xlda) {
//	      /* Read acceleration field data */
//	      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
//	      lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
//	      acceleration_mg[0] =
//	        lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[0]);
//	      acceleration_mg[1] =
//	        lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[1]);
//	      acceleration_mg[2] =
//	        lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[2]);
//	      sprintf_((char *)tx_buffer,
//	              "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
//	              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
//	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//	    }
//
//	    if (reg.status_reg.gda) {
//	      /* Read angular rate field data */
//	      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
//	      lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
//	      angular_rate_mdps[0] =
//	        lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
//	      angular_rate_mdps[1] =
//	        lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
//	      angular_rate_mdps[2] =
//	        lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
//	      sprintf_((char *)tx_buffer,
//	              "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
//	              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
//	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//	    }
//
//	    if (reg.status_reg.tda) {
//	      /* Read temperature data */
//	      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//	      lsm6dsm_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//	      temperature_degC = lsm6dsm_from_lsb_to_celsius(
//	                           data_raw_temperature);
//	      sprintf_((char *)tx_buffer,
//	              "Temperature [degC]:%6.2f\r\n",
//	              temperature_degC);
//	      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//	    }
//	    HAL_Delay(10);
//	  }
//}

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

	 // test
	test();
}

static _Bool waitForNotify(uint32_t ms)
{
	uint32_t n = 0;
    BaseType_t result = xTaskNotifyWait(0, ULONG_MAX, &n, pdMS_TO_TICKS(ms));
    return result == pdTRUE;
}

void accGyrSensorTimerCallback(TimerHandle_t xTimer)
{

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
