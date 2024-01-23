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
#include <math.h>

//----------------------------------------------------------------------
// Defines
#define DEG_TO_RAD				 0.0174532925f
#define	RAD_TO_DEG			    57.2957795131f
#define G_MPS2					 9.8066500000f

#define COMP_FILTER_ALPHA		 0.0500000000f

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
// Complementary Filter Type Definitions
typedef struct {
	float	roll;	// in deg
	float	pitch;	// in deg
} inclination_t;

//----------------------------------------------------------------------
// Complementary Filter Function Definitions
static void comp_filer_init(void);
static _Bool comp_filter_update(AccGyro_Msg_t data);
static void comp_filter_compute(inclination_t *incl);
static void incl_to_term(inclination_t incl);

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

	gyro_dat = data.gyro_degps;

	sprintf_((char *)tx_buffer,
		  "p=%4.2f,q=%4.2f,r=%4.2f\r\n",
		  gyro_dat.p, gyro_dat.q, gyro_dat.r);

	tx_com(tx_buffer, strlen((char const *)tx_buffer));
}

static void Thread_Main_Run(void *args)
{
	// Thread message loop

	// Init comp filter
	comp_filer_init();

	// Receive data for complementary filer implementation/calculations
	AccGyro_Msg_t msg;
    while (true) {
        if (xQueueReceive(q_main, &msg, pdMS_TO_TICKS(1000)) == pdTRUE) {

        	// Calculate inclination
        	inclination_t incl;

        	if (comp_filter_update(msg)) {
        		// Compute inclination
        		comp_filter_compute(&incl);

        		// Plot roll and pitch to oscilloscope
        		incl_to_term(incl);
        	}

            // tests
//            test_rx_q(msg);
        }
    }

}

//----------------------------------------------------------------------
// Complementary Filter Local Variables
static RCFilter lpfAcc[3];
static RCFilter lpfGyr[3];

static float phiHat_rad 	= 0.0f;
static float thetaHat_rad 	= 0.0f;

static uint32_t sample_time_ms = 0;

//----------------------------------------------------------------------
// Complementary Filter Function Implementations

static void comp_filer_init(void)
{
	// i2c thread currently sampling data at 20ms -> 50Hz
	// Max LPF cutoff would be 25Hz
	for (uint8_t i = 0; i < 3; i++) {
		RCFilter_Init(&lpfAcc[i], 5.0f, 0.01f);
		RCFilter_Init(&lpfGyr[i], 25.0f, 0.01f);
	}
}

static _Bool comp_filter_update(AccGyro_Msg_t data)
{
	static uint32_t prev_timestamp_ms = 0;
	static _Bool is_first_sampl = true;

	if (data.dataOK_flag) {
		/* Filter accelerometer data */
		RCFilter_Update(&lpfAcc[0], data.acc_mps2.x);
		RCFilter_Update(&lpfAcc[1], data.acc_mps2.y);
		RCFilter_Update(&lpfAcc[2], data.acc_mps2.z);

		/* Filter gyroscope data */
		RCFilter_Update(&lpfGyr[0], data.gyro_degps.p);
		RCFilter_Update(&lpfGyr[1], data.gyro_degps.q);
		RCFilter_Update(&lpfGyr[2], data.gyro_degps.r);

		/* Update sample time */
		sample_time_ms = data.timestamp_ms - prev_timestamp_ms;
		prev_timestamp_ms = data.timestamp_ms;

		if (is_first_sampl) {
			is_first_sampl = false;
			return false;
		}

		return true;
	}
}

static void comp_filter_compute(inclination_t *incl)
{
	/* Get filtered gyro measurements */
	float ax_mps2 = lpfAcc[0].out[0];
	float ay_mps2 = lpfAcc[1].out[0];
	float az_mps2 = lpfAcc[2].out[0];

	/* Estimate angles using accel measurements */
	float phiHat_acc_rad 	= atanf(ay_mps2 / az_mps2);
	float thetaHat_acc_rad 	= asinf(ax_mps2 / G_MPS2);

	/* Get filtered gyro measurements */
	float p_radps = (lpfGyr[0].out[0]) * DEG_TO_RAD;
	float q_radps = (lpfGyr[1].out[0]) * DEG_TO_RAD;
	float r_radps = (lpfGyr[2].out[0]) * DEG_TO_RAD;

	/* Transform body rates to Euler rates */
	float phiDot_radps = p_radps + tanf(thetaHat_rad) *
							(sinf(phiHat_rad) * q_radps + cosf(phiHat_rad) * r_radps);

	float thetaDot_radps = 	cosf(phiHat_rad) * q_radps - sinf(phiHat_rad) * r_radps;

	/* Combine accel estimates with integral if gyro readings */
	phiHat_rad 	= 		 COMP_FILTER_ALPHA 	*  phiHat_acc_rad										// accel
				+ 	(1 - COMP_FILTER_ALPHA)	* (phiHat_rad + (sample_time_ms / 1000.0f) * phiDot_radps);	// gyro integral

	thetaHat_rad = 		 COMP_FILTER_ALPHA 	*  thetaHat_acc_rad										// accel
				+ 	(1 - COMP_FILTER_ALPHA)	* (thetaHat_rad + (sample_time_ms / 1000.0f) * thetaDot_radps);	// gyro integral

	/* Populate inclination in deg */
	incl->roll 	= phiHat_rad 	* RAD_TO_DEG;
	incl->pitch	= thetaHat_rad 	* RAD_TO_DEG;
}

static void incl_to_term(inclination_t incl)
{
	static uint8_t tx_buffer[1000];

	sprintf_((char *)tx_buffer,
		  "roll=%4.2f,pitch=%4.2f\r\n",
		  incl.roll, incl.pitch);

	tx_com(tx_buffer, strlen((char const *)tx_buffer));
}
