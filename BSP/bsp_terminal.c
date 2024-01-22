/*
 * bsp_terminal.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Miller Solis
 */

//----------------------------------------------------------------------
// Includes
#include "bsp_terminal.h"
#include "usart.h"


//----------------------------------------------------------------------
// Defines
#define	SERIAL_COM				huart3


/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
void tx_com(uint8_t *tx_buffer, uint16_t len)
{

  HAL_UART_Transmit(&SERIAL_COM, tx_buffer, len, 1000);
}
