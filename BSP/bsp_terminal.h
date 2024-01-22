/*
 * bsp_terminal.h
 *
 *  Created on: Jan 22, 2024
 *      Author: Miller Solis
 */

#ifndef BSP_TERMINAL_H_
#define BSP_TERMINAL_H_

#include <stdint.h>


void tx_com( uint8_t *tx_buffer, uint16_t len );

#endif /* BSP_TERMINAL_H_ */
