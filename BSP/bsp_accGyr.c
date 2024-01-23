/*
 * bsp_accGyr.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Miller Solis
 */

//----------------------------------------------------------------------
// Includes
#include "bsp_accGyr.h"

#include "i2c.h"
#include "lsm6dsm_reg.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

//----------------------------------------------------------------------
// Defines
#define		BOOT_TIME          	15 		//ms
#define 	BUS_TIMEOUT			15		//ms

#define 	mG_TO_MPS2			0.00980665f
#define 	mDEG_TO_DEG			0.00100000f

//----------------------------------------------------------------------
// Function Definitions
static int32_t BSP_AccGyr_WriteReg(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t BSP_AccGyr_ReadReg(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

//----------------------------------------------------------------------
// External Variables
extern I2C_HandleTypeDef 	hi2c2;

//----------------------------------------------------------------------
// Local Variables
static stmdev_ctx_t 	dev_ctx;
static uint8_t 			whoamI, rst;
static I2C_HandleTypeDef* 	bus_handle;

//----------------------------------------------------------------------
// Function Implementations

void BSP_AccGyr_Init(void)
{
	bus_handle = &hi2c2;

	/*  Initialize mems driver interface */
	dev_ctx.write_reg = BSP_AccGyr_WriteReg;
	dev_ctx.read_reg = BSP_AccGyr_ReadReg;
	dev_ctx.handle = (void*) bus_handle;
	dev_ctx.mdelay = BSP_AccGyr_Delay;

	/* Wait sensor boot time */
	dev_ctx.mdelay(BOOT_TIME);

	/* Check device ID */
	lsm6dsm_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != LSM6DSM_ID)
	while (1) {
	  /* manage here device not found */
	}

	/* Restore default configuration */
	lsm6dsm_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do {
	lsm6dsm_reset_get(&dev_ctx, &rst);
	} while (rst);

	/*  Enable Block Data Update */
	lsm6dsm_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate for Acc and Gyro */
	lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_104Hz);
	lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_104Hz);

	/* Set full scale */
	lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_2g);
	lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_2000dps);

	/* Configure filtering chain(No aux interface)
	* Accelerometer - analog filter
	*/
	lsm6dsm_xl_filter_analog_set(&dev_ctx, LSM6DSM_XL_ANA_BW_400Hz);

	/* Accelerometer - LPF1 path (LPF2 not used) */
	//lsm6dsm_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSM_XL_LP1_ODR_DIV_4);

	/* Accelerometer - LPF1 + LPF2 path */
	lsm6dsm_xl_lp2_bandwidth_set(&dev_ctx,
							   LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_100);

	/* Accelerometer - High Pass / Slope path */
	//lsm6dsm_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
	//lsm6dsm_xl_hp_bandwidth_set(&dev_ctx, LSM6DSM_XL_HP_ODR_DIV_100);

	/* Gyroscope - filtering chain */
	lsm6dsm_gy_band_pass_set(&dev_ctx, LSM6DSM_HP_260mHz_LP1_STRONG);
}

_Bool BSP_AccGyr_GetData (Acc_Data_t *acc, Gyro_Data_t *gyro)
{
	static lsm6dsm_reg_t reg;

	// Acceleration Variables
	static int16_t data_raw_acceleration[3];
	static float acceleration_mg[3];


	// Gyro Variables
	static int16_t data_raw_angular_rate[3];
	static float angular_rate_mdps[3];

	_Bool ret = false;

	/* Read output only if new value is available */
	lsm6dsm_status_reg_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.xlda && reg.status_reg.gda) {
		/* Read acceleration field data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

		/* Convert to mG */
		acceleration_mg[0] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[2]);

		/* Convert to m/s^2 */
		acc->x = acceleration_mg[0] * mG_TO_MPS2;
		acc->y = acceleration_mg[1] * mG_TO_MPS2;
		acc->z = acceleration_mg[2] * mG_TO_MPS2;

		/* Read angular rate field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);

		/* Convert to mdeg/s */
		angular_rate_mdps[0] = lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] = lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] = lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);

		/* Convert to deg/s  */
		gyro->p = angular_rate_mdps[0] * mDEG_TO_DEG;
		gyro->q = angular_rate_mdps[1] * mDEG_TO_DEG;
		gyro->r = angular_rate_mdps[2] * mDEG_TO_DEG;

		ret = true;
	}

	return ret;
}

_Bool BSP_Acc_GetData (Acc_Data_t *acc)
{
	static int16_t data_raw_acceleration[3];
	static float acceleration_mg[3];
	static lsm6dsm_reg_t reg;

	_Bool ret = false;

	/* Read output only if new value is available */
	lsm6dsm_status_reg_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.xlda) {
		/* Read acceleration field data */
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration);

		/* Convert to mG */
		acceleration_mg[0] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[2]);

		/* Convert to m/s^2 */
		acc->x = acceleration_mg[0] * mG_TO_MPS2;
		acc->y = acceleration_mg[1] * mG_TO_MPS2;
		acc->z = acceleration_mg[2] * mG_TO_MPS2;

		ret = true;
	}

	return ret;
}

_Bool BSP_Gyro_GetData (Gyro_Data_t *gyro)
{
	static int16_t data_raw_angular_rate[3];
	static float angular_rate_mdps[3];
	static lsm6dsm_reg_t reg;

	_Bool ret = false;

	/* Read output only if new value is available */
	lsm6dsm_status_reg_get(&dev_ctx, &reg.status_reg);

	if (reg.status_reg.gda) {
		/* Read angular rate field data */
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);

		/* Convert to mdeg/s */
		angular_rate_mdps[0] = lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		angular_rate_mdps[1] = lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		angular_rate_mdps[2] = lsm6dsm_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);

		/* Convert to deg/s  */
		gyro->p = angular_rate_mdps[0] * mDEG_TO_DEG;
		gyro->q = angular_rate_mdps[1] * mDEG_TO_DEG;
		gyro->r = angular_rate_mdps[2] * mDEG_TO_DEG;

		ret = true;
	}

	return ret;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t BSP_AccGyr_WriteReg(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;

//	status = HAL_I2C_Mem_Write(handle, LSM6DSM_I2C_ADD_H, reg,
//                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, BUS_TIMEOUT);
	status = HAL_I2C_Mem_Write_IT(handle, LSM6DSM_I2C_ADD_H, reg,
									I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len);
	assert(status == HAL_OK);
	if (!BSP_WaitForRxTx(BUS_TIMEOUT)) {
		// This is a critical fault
		Error_Handler();
	}

	return status;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t BSP_AccGyr_ReadReg(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;

//	status = HAL_I2C_Mem_Read(handle, LSM6DSM_I2C_ADD_H, reg,
//                   I2C_MEMADD_SIZE_8BIT, bufp, len, BUS_TIMEOUT);
	status = HAL_I2C_Mem_Read_IT(handle, LSM6DSM_I2C_ADD_H, reg,
					 	 	 	 	 I2C_MEMADD_SIZE_8BIT, bufp, len);
	assert(status == HAL_OK);
	if (!BSP_WaitForRxTx(BUS_TIMEOUT)) {
		// This is a critical fault
		Error_Handler();
	}

	return status;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
__weak void BSP_AccGyr_Delay(uint32_t ms)
{
	// Implement in user application according to needs
	HAL_Delay(ms);
}

/*
 * @brief  platform specific bus TX RX wait(platform dependent)
 *
 * @param  ms        timeout in ms
 *
 */
__weak _Bool BSP_WaitForRxTx(uint32_t ms)
{
	UNUSED(ms);
	// Implement in user application if needed

	return true;
}
