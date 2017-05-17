/**
  ******************************************************************************
  * @file    Template_2/main.h
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   Header for gpio module
  ******************************************************************************
  * @attention
  *
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0X_UART_H
#define __STM32F0X_UART_H

//--- Exported types ---//
//--- Exported constants ---//
//--- Exported macro ---//
#define USART1_CLK (RCC->APB2ENR & 0x00004000)
#define USART1_CLK_ON RCC->APB2ENR |= 0x00004000
#define USART1_CLK_OFF RCC->APB2ENR &= ~0x00004000

#define USART2_CLK (RCC->APB1ENR & 0x00020000)
#define USART2_CLK_ON RCC->APB1ENR |= 0x00020000
#define USART2_CLK_OFF RCC->APB1ENR &= ~0x00020000

#define USART_9600		5000
#define USART_115200		416
#define USART_250000		192

#define USARTx                           USART1
#define USARTx_CLK                       RCC_APB2Periph_USART1
#define USARTx_APBPERIPHCLOCK            RCC_APB2PeriphClockCmd
//#define USARTx_IRQn                      USART1_IRQn
//#define USARTx_IRQHandler                USART1_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource9
#define USARTx_TX_AF                     GPIO_AF_1

#define USARTx_RX_PIN                    GPIO_Pin_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource10
#define USARTx_RX_AF                     GPIO_AF_1

#define USART2_TX_PIN                    GPIO_Pin_2
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_TX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USART2_TX_SOURCE                 GPIO_PinSource2
#define USART2_TX_AF                     GPIO_AF_1

#define USART2_RX_PIN                    GPIO_Pin_3
#define USART2_RX_GPIO_PORT              GPIOA
#define USART2_RX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USART2_RX_SOURCE                 GPIO_PinSource3
#define USART2_RX_AF                     GPIO_AF_1

#define USARTx_RX_DISA	USARTx->CR1 &= 0xfffffffb
#define USARTx_RX_ENA	USARTx->CR1 |= 0x04

#define SIZEOF_BUFF2	128
#define SIZEOF_BUFF2_RX		SIZEOF_BUFF2

//---- tipos de paquetes ----
#define USART_NO_PCKT	0
#define USART_PCKT_DMX	1
#define USART_PCKT_RDM	2

//--- Exported functions ---//
void USART_Config(void);
void Usart_Time_1ms (void);
void USARTx_IRQHandler(void);
void USARTSend(unsigned char);

void USART1Config(void);
void USART2Config(void);
void USART1Send(unsigned char);
void USART2Send(unsigned char);

void UsartSendDMX (void);
void UsartSendRDM (unsigned char);

#endif //--- End ---//


//--- END OF FILE ---//
