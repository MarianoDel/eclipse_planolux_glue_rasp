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
volatile unsigned char dmx_kind_of_pckt = 0;
volatile unsigned char dmx_bytes_left = 0;



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


unsigned char SendDMX_GetStatus (void)
{
	return dmx_state;
}


//funcion con maquina de estados, la va llamando TIM16IRQ para ir avanzando en la transmision
//tiene wrappers para DMX y RDM
//
void SendPacket (unsigned char new_func, unsigned char kind_of_pckt, unsigned char rdm_bytes)
{
	if (new_func == PCKT_INIT)
	{
		//empiezo la maquina de estados
		dmx_state = PCKT_INIT;
		if ((kind_of_pckt == KIND_OF_RDM) && (rdm_bytes))
		{
			dmx_kind_of_pckt = KIND_OF_RDM;
			dmx_bytes_left = rdm_bytes;
		}
		else
		{
			//deberia ser DMX
			dmx_kind_of_pckt = KIND_OF_DMX;
			dmx_bytes_left = 0;
		}
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
			if (dmx_kind_of_pckt == KIND_OF_DMX)
				UsartSendDMX();
			if (dmx_kind_of_pckt == KIND_OF_RDM)
				UsartSendRDM(dmx_bytes_left);

			break;

		case PCKT_TRANSMITING:	//se deben haber transmitido el start code + los 512 canales
								//o el paquete RDM
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
