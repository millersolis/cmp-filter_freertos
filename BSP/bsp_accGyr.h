/*
 * bsp_accGyr.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Miller Solis
 */

#ifndef BSP_ACCGYR_H_
#define BSP_ACCGYR_H_

#include <stdint.h>

typedef struct
{
	float x;
	float y;
	float z;
} Acc_Data_t;	// in m/s^2

typedef struct
{
	float p;
	float q;
	float r;
} Gyro_Data_t;	// in deg/s


void BSP_AccGyr_Init(void);

_Bool BSP_AccGyr_GetData (Acc_Data_t *acc, Gyro_Data_t *gyro);
_Bool BSP_Acc_GetData (Acc_Data_t *acc);
_Bool BSP_Gyro_GetData (Gyro_Data_t *gyro);


//----------------------------------------------------------------------
// Weakly Implemented in "bsp_accGyr.c"
// To be implemented by application code
void BSP_AccGyr_Delay(uint32_t ms);
_Bool BSP_WaitForRxTx(uint32_t ms);


#endif /* BSP_ACCGYR_H_ */
