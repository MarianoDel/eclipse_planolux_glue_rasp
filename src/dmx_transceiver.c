/*
 * dmx_transceiver.c
 *
 *  Created on: 09/02/2016
 *      Author: Mariano
 */
#include "dmx_transceiver.h"
#include "hard.h"
#include "stm32f0xx.h"
#include "stm32f0x_tim.h"
#include "stm32f0x_uart.h"
#include "stm32f0x_gpio.h"



//--- VARIABLES GLOBALES ---//
volatile unsigned char dmx_state = 0;


//--- FUNCIONES DEL MODULO ---//
void DMX_Ena(void)
{
	//habilito la interrupción
	EXTIOn ();
	USART1->CR1 |= USART_CR1_UE;
}

void DMX_Disa(void)
{
	//deshabilito la interrupción
	EXTIOff ();
	USART1->CR1 &= ~USART_CR1_UE;
}

//funcion con maquina de estados, la va llamando TIM16IRQ para ir avanzando en la transmision
//
void SendDMXPacket (unsigned char new_func)
{
	if (new_func == PCKT_INIT)
	{
		//empiezo la maquina de estados
		dmx_state = PCKT_INIT;
	}
	else if (new_func != PCKT_UPDATE)	//update de la maquina de estados
		return;

	switch (dmx_state)
	{
		case PCKT_INIT:
			SW_TX;
			LED_ON;
			dmx_state++;
			DMX_TX_PIN_ON;	//mando break
			OneShootTIM16(88);
			break;

		case PCKT_END_BREAK:
			dmx_state++;
			DMX_TX_PIN_OFF;	//espero mark after break
			OneShootTIM16(8);
			break;

		case PCKT_END_MARK:
			dmx_state++;
			UsartSendDMX();
			break;

		case PCKT_TRANSMITING:	//se deben haber transmitido el start code + los 512 canales
			dmx_state = PCKT_END_TX;
			DMX_TX_PIN_OFF;
			LED_OFF;
			break;

		case PCKT_END_TX:	//estado de espera luego de transmitir
			break;

		default:
			dmx_state = PCKT_END_TX;
			DMX_TX_PIN_OFF;
			break;
	}
}

