/**
  ******************************************************************************
  * @file    Template_2/stm32f0_uart.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   UART functions.
  ******************************************************************************
  * @attention
  *
  * Use this functions to configure serial comunication interface (UART).
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hard.h"
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx_adc.h"
//#include "stm32f0xx_can.h"
//#include "stm32f0xx_cec.h"
//#include "stm32f0xx_comp.h"
//#include "stm32f0xx_crc.h"
//#include "stm32f0xx_crs.h"
//#include "stm32f0xx_dac.h"
//#include "stm32f0xx_dbgmcu.h"
//#include "stm32f0xx_dma.h"
//#include "stm32f0xx_exti.h"
//#include "stm32f0xx_flash.h"
#include "stm32f0xx_gpio.h"
//#include "stm32f0xx_i2c.h"
//#include "stm32f0xx_iwdg.h"
#include "stm32f0xx_misc.h"
//#include "stm32f0xx_pwr.h"
#include "stm32f0xx_rcc.h"
//#include "stm32f0xx_rtc.h"
#include "stm32f0xx_spi.h"
//#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_usart.h"
//#include "stm32f0xx_wwdg.h"
#include "system_stm32f0xx.h"
#include "stm32f0xx_it.h"

#include "stm32f0x_uart.h"
#include "stm32f0x_gpio.h"
#include "dmx_transceiver.h"

#include <string.h>




//--- Private typedef ---//
//--- Private define ---//
//--- Private macro ---//

//#define USE_USARTx_TIMEOUT

#ifdef USE_USARTx_TIMEOUT
#define BUFFRX_TIMEOUT 200 //200ms
#define BUFFTX_TIMEOUT 200 //200ms
#endif


//--- VARIABLES EXTERNAS ---//
//extern volatile unsigned char buffrx_ready;
//extern volatile unsigned char *pbuffrx;

extern volatile unsigned char Packet_Detected_Flag;
extern volatile unsigned char dmx_receive_flag;
extern volatile unsigned short DMX_channel_received;
extern volatile unsigned short DMX_channel_selected;
extern volatile unsigned char DMX_channel_quantity;
extern volatile unsigned char data1[];
//static unsigned char data_back[10];
extern volatile unsigned char data[];

extern volatile unsigned char rasp_info_ready;
extern unsigned char Buff2rx_bkp [];


volatile unsigned char * pdmx;

//--- Private variables ---//
//Reception buffer.
unsigned char Buff2rx [SIZEOF_BUFF2_RX];
unsigned char * pBuff2rx;
volatile unsigned char rdm_bytes_left = 0;
volatile unsigned char pckt_kind = 0;

//Transmission buffer.

//--- Private function prototypes ---//
//--- Private functions ---//

void Usart_Time_1ms (void)
{
#ifdef USE_USARTx_TIMEOUT
	if (buffrx_timeout > 1)
		buffrx_timeout--; //Se detiene con buffrx_timeout = 1.

	if (bufftx_timeout > 1)
		bufftx_timeout--; //Se detiene con bufftx_timeout = 1.
#endif
}
//-------------------------------------------//
// @brief  UART configure.
// @param  None
// @retval None
//------------------------------------------//
void USART1_IRQHandler(void)
{
	unsigned short i;
	unsigned char dummy;

	/* USART in mode Receiver --------------------------------------------------*/
	//if (USART_GetITStatus(USARTx, USART_IT_RXNE) == SET)
	if (USART1->ISR & USART_ISR_RXNE)
	{
		//RX DMX
		//data0 = USART_ReceiveData(USART3);
		dummy = USARTx->RDR & 0x0FF;

		if (dmx_receive_flag)
		{
			if (DMX_channel_received == 0)		//empieza paquete
				LED_ON;

			//TODO: aca ver si es DMX o RDM
			if (DMX_channel_received < 511)
			{
				data1[DMX_channel_received] = dummy;
				DMX_channel_received++;
			}
			else
				DMX_channel_received = 0;

			//TODO: revisar canales 510 + 4
			if (DMX_channel_received >= (DMX_channel_selected + DMX_channel_quantity))
			{
				//los paquetes empiezan en 0 pero no lo verifico
				for (i=0; i<DMX_channel_quantity; i++)
				{
					data[i] = data1[(DMX_channel_selected) + i];
				}

				/*
				if ((data[0] < 10) || (data[0] > 240))	//revisa el error de salto de canal
					LED2_ON;
				else
					LED2_OFF;	//trata de encontrar el error de deteccion de trama
				*/

				//--- Reception end ---//
				DMX_channel_received = 0;
				//USARTx_RX_DISA;
				dmx_receive_flag = 0;
				Packet_Detected_Flag = 1;
				LED_OFF;	//termina paquete
			}
		}
		else
			USART1->RQR |= 0x08;	//hace un flush de los datos sin leerlos
	}

	/* USART in mode Transmitter -------------------------------------------------*/
	//if (USART_GetITStatus(USARTx, USART_IT_TXE) == SET)


	if (USART1->CR1 & USART_CR1_TXEIE)
	{
		if (USART1->ISR & USART_ISR_TXE)
		{
			//me fijo el tipo de paquete a enviar
			if (pckt_kind == USART_PCKT_DMX)
			{
				if (pdmx < &data1[512])
				{
					USARTx->TDR = *pdmx;
					pdmx++;
				}
				else
				{
					pckt_kind = USART_NO_PCKT;
					USART1->CR1 &= ~USART_CR1_TXEIE;
					SendDMXPacket(PCKT_UPDATE);
				}
			}

			if (pckt_kind == USART_PCKT_RDM)
			{
				if ((pdmx < &data1[512]) && (rdm_bytes_left))
				{
					USARTx->TDR = *pdmx;
					pdmx++;
					rdm_bytes_left--;
				}
				else
				{
					//termine de enviar los bytes o hubo un error
					rdm_bytes_left = 0;
					pckt_kind = USART_NO_PCKT;
					USART1->CR1 &= ~USART_CR1_TXEIE;
					SendRDMPacket(PCKT_UPDATE, 0);
				}
			}
		}
	}

	if ((USART1->ISR & USART_ISR_ORE) || (USART1->ISR & USART_ISR_NE) || (USART1->ISR & USART_ISR_FE))
	{
		USART1->ICR |= 0x0e;
		dummy = USART1->RDR;
	}
}

void USART2_IRQHandler(void)
{
	unsigned char dummy;

	/* USART in mode Receiver --------------------------------------------------*/
	//if (USART_GetITStatus(USARTx, USART_IT_RXNE) == SET)
	if (USART2->ISR & USART_ISR_RXNE)
	{
//		dummy = USARTx->RDR & 0x0FF;

		//Lectura del dato recibido.
		*pBuff2rx = USART2->RDR & 0x0FF;

		if (*(pBuff2rx) != 0)
		{
			if ((*(pBuff2rx) == '\n') || (*(pBuff2rx) == 26))
			{
				*(pBuff2rx+1) = 0;

				strcpy((char *)&Buff2rx_bkp[0], (const char *)&Buff2rx[0]);

				rasp_info_ready = 1;
				pBuff2rx = &Buff2rx[0];
				*pBuff2rx = 0;
			}
			else
			{
				//-- Mueve el puntero ---//
				if (pBuff2rx < &Buff2rx[SIZEOF_BUFF2])
					pBuff2rx++;
				else
					pBuff2rx = &Buff2rx[0];
			}
		}

		//funcion de loop
		//USART2->TDR = dummy;
	}

	/* USART in mode Transmitter -------------------------------------------------*/
	//if (USART_GetITStatus(USARTx, USART_IT_TXE) == SET)

/*
	if (USART2->CR1 & USART_CR1_TXEIE)
	{
		if (USART2->ISR & USART_ISR_TXE)
		{
	//		USARTx->CR1 &= ~0x00000088;	//bajo TXEIE bajo TE
			//USART1->CR1 &= ~USART_CR1_TXEIE;
			//USARTx->TDR = 0x00;

			if (pdmx < &data1[512])
			{
				USARTx->TDR = *pdmx;
				pdmx++;
			}
			else
			{
				USART1->CR1 &= ~USART_CR1_TXEIE;
				SendDMXPacket(PCKT_UPDATE);
			}
		}
	}
*/


	if ((USART2->ISR & USART_ISR_ORE) || (USART2->ISR & USART_ISR_NE) || (USART2->ISR & USART_ISR_FE))
	{
		USART2->ICR |= 0x0e;
		dummy = USART2->RDR;
	}
}

void UsartSendDMX (void)
{
	pdmx = &data1[0];
	pckt_kind = USART_PCKT_DMX;
	USART1->CR1 |= USART_CR1_TXEIE;
}

void UsartSendRDM (unsigned char bytes)
{
	pdmx = &data1[0];
	pckt_kind = USART_PCKT_RDM;
	rdm_bytes_left = bytes;
	USART1->CR1 |= USART_CR1_TXEIE;
}

void USART1Config(void)
{
	if (!USART1_CLK)
		USART1_CLK_ON;


	USART1->BRR = USART_250000;
	USART1->CR2 |= USART_CR2_STOP_1;	//2 bits stop
	USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 5);

}

void USART1Send(unsigned char value)
{
	while ((USART1->ISR & 0x00000080) == 0);
	USART1->TDR = value;
}

void USART2Config(void)
{
	//NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable USART clock */
	if (!USART2_CLK)
		USART2_CLK_ON;

	GPIOA->AFR[0] |= 0x00001100;

	/* USARTx configuration ----------------------------------------------------*/
	// oversampling by 16, 9600 baud
	// 8 data bit, 1 start bit, 1 stop bit, no parity
	//USART2->BRR = USART_9600;
	USART2->BRR = USART_115200;
	USART2->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
	//USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

	/* NVIC configuration */
	/* Enable the USARTx Interrupt */
	/*
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 6
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	pBuff2rx = Buff2rx;

	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 6);

	/* Enable USART */
	//USART2->CR1 |= USART_CR1_UE;

	//  pdmx = &data1[0];
	//USART2->CR1 &= ~USART_CR1_TXEIE;	//apago int del transmisor

}

void USART2Send(unsigned char value)
{
	while ((USART2->ISR & 0x00000080) == 0);
	USART2->TDR = value;
}

void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

  /* Enable USART clock */
  USARTx_APBPERIPHCLOCK(USARTx_CLK, ENABLE);

  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);

  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

  /* USARTx configuration ----------------------------------------------------*/
  /* USARTx configured as follow:
  - BaudRate = 9600 baud
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  //USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_BaudRate = 250000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_2;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);

  /* NVIC configuration */
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);


#ifdef USE_USARTx_TIMEOUT
  buffrx_timeout = 0;
  bufftx_timeout = 0;
#endif

  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);

//  pdmx = &data1[0];
  USART1->CR1 &= ~USART_CR1_TXEIE;	//apago int del transmisor

  //--- Enable receiver interrupt ---//
  USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
}

void USARTSend(unsigned char value)
{
	while ((USARTx->ISR & 0x00000080) == 0);
	USARTx->TDR = value;
}

//--- end of file ---//

