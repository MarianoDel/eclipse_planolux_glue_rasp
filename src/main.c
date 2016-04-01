/**
  ******************************************************************************
  * @file    Template_2/main.c
  * @author  Nahuel
  * @version V1.0
  * @date    22-August-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Use this template for new projects with stm32f0xx family.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//--- My includes ---//
#include "stm32f0x_gpio.h"
#include "stm32f0x_tim.h"
#include "stm32f0x_uart.h"

#include "hard.h"
//#include "main.h"



#include "core_cm0.h"
#include "adc.h"
#include "flash_program.h"
#include "synchro.h"
#include "dmx_transceiver.h"

//librerias RDM
#include "rdm_frame.h"
#include "utils.h"
#include "rdm_util.h"

//--- VARIABLES EXTERNAS ---//
volatile unsigned char timer_1seg = 0;

volatile unsigned short timer_led_comm = 0;
volatile unsigned short timer_for_cat_switch = 0;
volatile unsigned short timer_for_cat_display = 0;

// ------- Externals de los USART -------
volatile unsigned char buffrx_ready = 0;
volatile unsigned char *pbuffrx;
volatile unsigned short wait_ms_var = 0;

volatile unsigned char rasp_info_ready = 0;
unsigned char Buff2rx_bkp [SIZEOF_BUFF2_RX];

//volatile unsigned char TxBuffer_SPI [TXBUFFERSIZE];
//volatile unsigned char RxBuffer_SPI [RXBUFFERSIZE];
//volatile unsigned char *pspi_tx;
//volatile unsigned char *pspi_rx;
//volatile unsigned char spi_bytes_left = 0;

// ------- Externals del DMX -------
volatile unsigned char Packet_Detected_Flag;
volatile unsigned char dmx_receive_flag = 0;
volatile unsigned short DMX_channel_received = 0;
volatile unsigned short DMX_channel_selected = 1;
volatile unsigned char DMX_channel_quantity = 4;

volatile unsigned char data1[512];
//static unsigned char data_back[10];
volatile unsigned char data[10];

volatile unsigned char orig_shine_slider [120];
volatile unsigned char orig_shine_room [4];

// ------- Externals de los timers -------
volatile unsigned short prog_timer = 0;

// ------- Externals de los modos -------




//--- VARIABLES GLOBALES ---//


// ------- de los timers -------
volatile unsigned short timer_standby;
volatile unsigned short timer_dmx_display_show;
volatile unsigned char display_timer;
volatile unsigned char filter_timer;
static __IO uint32_t TimingDelay;

//volatile unsigned char door_filter;
//volatile unsigned char take_sample;
//volatile unsigned char move_relay;
//volatile unsigned char secs = 0;
//volatile unsigned short minutes = 0;

// ------- del display -------
unsigned char v_opt [10];


// ------- del DMX -------
volatile unsigned char signal_state = 0;
volatile unsigned char dmx_timeout_timer = 0;
//unsigned short tim_counter_65ms = 0;

// ------- de los filtros DMX -------
#define LARGO_F		32
#define DIVISOR_F	5
unsigned char vd0 [LARGO_F + 1];
unsigned char vd1 [LARGO_F + 1];
unsigned char vd2 [LARGO_F + 1];
unsigned char vd3 [LARGO_F + 1];
unsigned char vd4 [LARGO_F + 1];


#define IDLE	0
#define LOOK_FOR_BREAK	1
#define LOOK_FOR_MARK	2
#define LOOK_FOR_START	3

//--- FUNCIONES DEL MODULO ---//
void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);

void UpdateDMX (unsigned char *, unsigned short, unsigned char);
void Update_PWM (unsigned short);
unsigned char ReadPckt(unsigned char *);
void ReadPcktR(unsigned char *);
void ReadPcktS(unsigned char *);
unsigned char ReadPcktT(unsigned char *);
void FillHeaderK (RDMHeaderK *);
void FillMDBK (RDMHeaderK *, unsigned char * , unsigned char);


//unsigned char GetValue (unsigned char * , const unsigned short * );
unsigned short GetValue (unsigned char * );

// ------- de los paquetes RDM -------
#define RDM_NO_PCKT	0
#define PCKT_TYPE0	1
#define PCKT_TYPE1	2
#define PCKT_TYPE2	3
#define PCKT_TYPE3	4

#define SIZEOF_PCKT_TYPE0	1
#define SIZEOF_PCKT_TYPE1	2
#define SIZEOF_PCKT_TYPE2	3
#define SIZEOF_PCKT_TYPE3	4

#define SIZEOF_HEADERK		15
#define SIZEOF_CHECKSUM		2

// ------- del DMX -------
extern void EXTI4_15_IRQHandler(void);
#define DMX_TIMEOUT	20
unsigned char MAFilter (unsigned char, unsigned char *);

//--- FILTROS DE SENSORES ---//
#define LARGO_FILTRO 16
#define DIVISOR      4   //2 elevado al divisor = largo filtro
//#define LARGO_FILTRO 32
//#define DIVISOR      5   //2 elevado al divisor = largo filtro
unsigned short vtemp [LARGO_FILTRO + 1];
unsigned short vpote [LARGO_FILTRO + 1];

//--- FIN DEFINICIONES DE FILTRO ---//

const unsigned char s_comm_status [] = {
		0xCC,	//CC
		PID_COMMS_STATUS,	//PID
		0x00	//PDL
};
//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
	unsigned char i;
	unsigned short ii;
	unsigned char rdm_is_needed = 0;

	//!< At this stage the microcontroller clock setting is already configured,
    //   this is done through SystemInit() function which is called from startup
    //   file (startup_stm32f0xx.s) before to branch to application main.
    //   To reconfigure the default setting of SystemInit() function, refer to
    //   system_stm32f0xx.c file

	//GPIO Configuration.
	GPIO_Config();


	//ACTIVAR SYSTICK TIMER
	if (SysTick_Config(48000))
	{
		while (1)	/* Capture error */
		{
			if (LED)
				LED_OFF;
			else
				LED_ON;

			for (i = 0; i < 255; i++)
			{
				asm (	"nop \n\t"
						"nop \n\t"
						"nop \n\t" );
			}
		}
	}


	 //PRUEBA LED Y OE
	/*
	 while (1)
	 {
		 if (LED)
		 {
//			 CTRL_BKL_ON;
			 LED_OFF;
		 }
		 else
		 {
			 LED_ON;
//			 CTRL_BKL_OFF;
		 }

		 Wait_ms(150);
	 }
	 */

	 //FIN PRUEBA LED Y OE

	//TIM Configuration.
	TIM_3_Init();
	TIM_14_Init();
	//Timer_3_Init();
	//Timer_4_Init();

	//--- PRUEBA DISPLAY LCD ---
	EXTIOff ();

	LED_ON;
	Wait_ms(1000);
	LED_OFF;

	//DE PRODUCCION Y PARA PRUEBAS EN DMX
	Packet_Detected_Flag = 0;
	DMX_channel_selected = 1;
	DMX_channel_quantity = 4;
	USART1Config();
//	USART_Config();
	USART2Config();
//	DMX_Disa();



	//RELAY_ON;

	//--- PRUEBA USART
	/*
	//EXTIOff();
	USART_Config();
	while (1)
	{
		DMX_channel_received = 0;
		data1[0] = 0;
		data1[1] = 0;
		data1[2] = 0;
		dmx_receive_flag = 1;
		USARTSend('M');
		USARTSend('E');
		USARTSend('D');
		Wait_ms(1);
		if ((data1[0] == 'M') && (data1[1] == 'E') && (data1[2] == 'D'))
			LED_ON;
		else
			LED_OFF;
		Wait_ms(200);
	}
	*/
	//--- FIN PRUEBA USART

	//--- PRUEBA USART2
	/*
	while(1)	//pruebo pata tx
	{
		TX_ON;
		Wait_ms(10);
		TX_OFF;
		Wait_ms(10);
	}
	*/
/*
	while (1)
	{
		//USART1Send('M');
		//USARTSend('M');
		USART2Send('M');
		Wait_ms(1009);
	}
*/
	//--- FIN PRUEBA USART2




	//--- PRUEBA EXTI PA8 con DMX y sw a TX
	/*
	while (1)
	{
		//cuando tiene DMX mueve el LED
		EXTIOn();
		SW_RX;
		Wait_ms(200);
		EXTIOff();
		SW_TX;
		Wait_ms(200);
	}
	*/
	//--- FIN PRUEBA EXTI PA8 con DMX


	//--- PRUEBA CH0 DMX con switch de display	inicializo mas arriba USART y variables
	/*
//	DMX_Ena();

	if (ADC_Conf() == 0)
	{
		while (1)
		{
			if (LED)
				LED_OFF;
			else
				LED_ON;

			Wait_ms(150);
		}
	}

	while (1)
	{
		if (Packet_Detected_Flag)
		{
			//llego un paquete DMX
			Packet_Detected_Flag = 0;

			//en data tengo la info
			Update_TIM3_CH1 (data[0]);

			ii = data[0] * 100;
			ii = ii / 255;
			if (ii > 100)
				ii = 100;

			if (last_percent != ii)
			{
				last_percent = ii;

				LCD_2DO_RENGLON;
				LCDTransmitStr((const char *) "            ");

				//Lcd_SetDDRAM(0x40 + 12);
				sprintf(s_lcd, "%3d", ii);
				LCDTransmitStr(s_lcd);
				LCDTransmitStr("%");

				LCD_2DO_RENGLON;
				for (i = 0; i < last_percent; i=i+10)
				{
					LCDStartTransmit(0xff);
				}
			}
		}

		UpdateSwitches ();
		UpdateIGrid();

	}
	*/
	//--- FIN PRUEBA CH0 DMX

	//--- PRUEBA CH0 DMX con switch de display	inicializo mas arriba USART y variables
	/*
	if (ADC_Conf() == 0)
	{
		while (1)
		{
			if (LED)
				LED_OFF;
			else
				LED_ON;

			Wait_ms(150);
		}
	}

	//DMX_Disa();
	while (1)
	{
		if (CheckS1() > S_NO)
		{
			sw_state = 1;
		//	RELAY_ON;
		}
		else if (CheckS2() > S_NO)
		{
			sw_state = 0;
		//	RELAY_OFF;
		}

		if (sw_state == 1)		//si tengo que estar prendido
		{
			if (Packet_Detected_Flag)
			{
				//llego un paquete DMX
				Packet_Detected_Flag = 0;

				//en data tengo la info
				Update_TIM3_CH1 (data[0]);
				//Update_TIM3_CH2 (data[1]);
				//Update_TIM3_CH3 (data[2]);
				//Update_TIM3_CH4 (data[3]);
				sprintf(s_lcd, "%03d", data[0]);
				LCD_2DO_RENGLON;
				LCDTransmitStr(s_lcd);
				sprintf(s_lcd, "  %04d", GetIGrid());
				LCDTransmitStr(s_lcd);
			}

			if (!timer_standby)
			{
				timer_standby = 1000;
				sprintf(s_lcd, "%03d", ii);
				LCD_1ER_RENGLON;
				LCDTransmitStr(s_lcd);
				if (ii < 255)
					ii++;
				else
					ii = 0;
			}
		}
		else	//apago los numeros
		{
			if (sw_state == 0)
			{
				LCD_2DO_RENGLON;
				LCDTransmitStr((const char *) "                ");
				sw_state = 2;
			}
		}

		UpdateSwitches ();
		UpdateIGrid();

	}
	*/
	//--- FIN PRUEBA CH0 DMX

	//--- PRUEBA DMX cuento paquetes
/*
	LCD_1ER_RENGLON;
	LCDTransmitStr((const char *) "Paquete numero: ");
	LCD_2DO_RENGLON;
	LCDTransmitStr((const char *) "                ");
	ii = 0;
	DMX_Ena();
	//DMX_Disa();
	while (1)
	{
		if (Packet_Detected_Flag)
		{
			Packet_Detected_Flag = 0;
			if (ii < 65532)
				ii++;
			else
				ii = 0;

			sprintf(s_lcd, "%d", ii);
			LCD_2DO_RENGLON;
			LCDTransmitStr(s_lcd);
		}

		UpdateSwitches ();

	}
*/
	//--- FIN PRUEBA DMX cuento paquetes

	//--- PRUEBA CH0 con SW_AC modo STAND_ALONE
	/*
	Wait_ms(3000);
	LCD_1ER_RENGLON;
	LCDTransmitStr((const char *) "  Lights OFF    ");
	LCD_2DO_RENGLON;
	LCDTransmitStr((const char *) "  PLANOLUX LLC  ");

	RELAY_ON;

	if (ADC_Conf() == 0)
	{
		while (1)
		{
			if (LED)
				LED_OFF;
			else
				LED_ON;

			Wait_ms(150);
		}
	}

	Update_TIM3_CH1 (0);
	sw_state = 1;
	while (1)
	{
		switch (sw_state)
		{
			case 1:
				if (CheckACSw() > S_NO)
					sw_state = 2;

				break;

			case 2:
				if (CheckACSw() > S_HALF)
					sw_state = 10;

				if (CheckACSw() == S_NO)
					sw_state = 3;

				break;

			case 3:	//aca lo prendo
				LCD_1ER_RENGLON;
				LCDTransmitStr((const char *) "Switching ON... ");
				ii = 51;
				sw_state++;
				break;

			case 4:	//filtro de subida
				if (!timer_standby)
				{
					timer_standby = 20;
					if (ii < 255)
					{
						Update_TIM3_CH1 (ii);
						ii++;
					}
					else
					{
						Update_TIM3_CH1 (255);
						LCD_1ER_RENGLON;
						LCDTransmitStr((const char *) "  Lights ON     ");
						sw_state = 5;
					}
				}
				break;

			case 5:	//aca me trabo prendido
				if (CheckACSw() > S_NO)
					sw_state = 6;

				break;

			case 6:
				if (CheckACSw() > S_HALF)
					sw_state = 10;

				if (CheckACSw() == S_NO)
					sw_state = 7;

				break;

			case 7:	//aca lo apago
				LCD_1ER_RENGLON;
				LCDTransmitStr((const char *) "Switching OFF...");
				ii = 255;
				sw_state++;
				break;

			case 8:	//filtro de bajada
				if (!timer_standby)
				{
					timer_standby = 20;
					if (ii > 0)
					{
						Update_TIM3_CH1 (ii);
						ii--;
					}
					else
					{
						Update_TIM3_CH1 (0);
						sw_state = 1;
						LCD_1ER_RENGLON;
						LCDTransmitStr((const char *) "  Lights OFF    ");
					}
				}
				break;

			case 10:
				break;

		}
		UpdateSwitches ();
		UpdateIGrid();
		UpdateACSwitch();

	}
	*/
	//--- FIN PRUEBA CH0 con SW_AC modo STAND_ALONE

	//--- PRUEBA ONE SHOOT TIMER 16
	/*
	DMX_TX_PIN_OFF;
	SW_TX;
	TIM_16_Init();
	LED_OFF;
	ii = 0;

	while (1)
	{
		if (!timer_standby)
		{
			timer_standby = 40;	//transmito cada 40ms

			if (ii == 0)
			{
				DMX_TX_PIN_ON;
				OneShootTIM16(1000);
				ii = 1;
			}
			else
			{
				DMX_TX_PIN_ON;
				OneShootTIM16(4000);
				ii = 0;
			}
		}
	}
	*/
	//--- FIN PRUEBA ONE SHOOT TIMER 16

	//--- PRUEBA DMX SOLO PAQUETE
	/*
	while (1)
	{
		if (!timer_standby)		//mando paquete DMX cada 25ms
		{
			timer_standby = 25;	//transmito cada 25ms
			UsartSendDMX ();
			//USART1Send('M');
		}
	}
	*/
	//--- FIN PRUEBA DMX SOLO PAQUETE

	//--- PRUEBA HARDWARE DE MASTER EN CH0 DMX inicializo mas arriba USART y variables

	USART2Send('M');
	DMX_TX_PIN_OFF;
	SW_TX;
	TIM_16_Init();
	LED_OFF;
	ii = 0;
	data1[0] = 0;
	SendPacketReset();

	while (1)
	{
		/*
		if (!timer_standby)		//mando paquete DMX cada 25ms
		{
			//me fijo si puedo enviar DMX
			if (SendDMX_GetStatus() == PCKT_END_TX)
			{
				SendDMXPacket(PCKT_INIT);
				timer_standby = 25;	//transmito cada 25ms
			}
		}
		*/

		if (rdm_is_needed)
		{
			//me fijo si puedo enviar y el tipo de paquete que armo
			if (SendDMX_GetStatus() == PCKT_END_TX)
			{
				//puedo enviar
				data1[0] = 0xCC;

				switch (rdm_is_needed)	//en rdm_is_needed tengo el tipo de paquete a enviar
				{
					case PCKT_TYPE0:	//Communication Status (COMMS_STATUS)
						FillHeaderK((RDMHeaderK *) data1);
						FillMDBK((RDMHeaderK *) data1, (unsigned char *) s_comm_status, sizeof(s_comm_status));

						SendRDMPacket(PCKT_INIT, (((RDMHeaderK *)data1)->message_length) + SIZEOF_CHECKSUM);
						break;

					case PCKT_TYPE1:
						//cargo paquete RDM modelo 1

						SendRDMPacket(PCKT_INIT, SIZEOF_PCKT_TYPE1);

						break;

					case PCKT_TYPE2:
						//cargo paquete RDM modelo 1

						SendRDMPacket(PCKT_INIT, SIZEOF_PCKT_TYPE2);

						break;

					case PCKT_TYPE3:
						//cargo paquete RDM modelo 1

						SendRDMPacket(PCKT_INIT, SIZEOF_PCKT_TYPE3);

						break;





				}
				rdm_is_needed = RDM_NO_PCKT;
			}
		}

		if (rasp_info_ready)
		{
			//llego info desde la raspberry
			rasp_info_ready = 0;
			rdm_is_needed = ReadPckt(Buff2rx_bkp);
		}

	}

	//--- FIN PRUEBA HARDWARE MASTER CH0 DMX

	return 0;
}


//--- End of Main ---//

unsigned char ReadPckt(unsigned char * p)
{
	unsigned char resp = 0;

	//me fijo el tipo de paquete
	switch (*p)
	{
		case 'r':
			ReadPcktR(p);
			break;

		case 's':
			ReadPcktS(p);
			break;

		case 't':
			resp = ReadPcktT(p);	//requiere transmitir RDM
			break;

		default:
			break;
	}
	return resp;
}

//en R me llega un parametro general
//del brillo de la habitacion
//r0,100;\r\n
void ReadPcktR(unsigned char * p)
{
	unsigned char new_shine;
	unsigned short ii;
	unsigned short a;

	if (*(p+2) != ',')
		return;

	//if ((GetValue((p + 3), &new_shine)) == 0)
	if (GetValue(p + 3) == 0xffff)
		return;

	new_shine = GetValue(p + 3);

	switch (*(p+1))
	{
		case '0':
			orig_shine_room[0] = new_shine;
			for (ii = 1; ii < 31; ii++)
			{
				a = (new_shine + 1) * orig_shine_slider[ii];	//muevo el valor de todos los sliders del room juntos
				a >>= 8;
				data1[ii] = a;
			}
			break;

		case '1':
			orig_shine_room[1] = new_shine;
			for (ii = 31; ii < 61; ii++)
			{
				a = (new_shine + 1) * orig_shine_slider[ii];
				a >>= 8;
				data1[ii] = a;
			}
			break;

		case '2':
			orig_shine_room[2] = new_shine;
			for (ii = 61; ii < 91; ii++)
			{
				a = (new_shine + 1) * orig_shine_slider[ii];
				a >>= 8;
				data1[ii] = a;
			}
			break;

		case '3':
			orig_shine_room[3] = new_shine;
			for (ii = 91; ii < 121; ii++)
			{
				a = (new_shine + 1) * orig_shine_slider[ii];
				a >>= 8;
				data1[ii] = a;
			}
			break;

		default:
			break;
	}
}

//en S me llega un parametro particular
//del brillo de esa lampara pero indicando cada habitacion
//s0,0,255;\r\n
void ReadPcktS(unsigned char * p)
{
	char room;
	unsigned char slider;
	unsigned char new_shine;
	unsigned short ii;

	if (*(p+2) != ',')
		return;

	room = *(p+1) - '0';

	if ((room < 0) || (room > 3))
		return;

	ii = GetValue(p + 3);
	if (ii == 0xffff)
		return;
	else
		slider = (unsigned char) ii;

	if (slider < 10)
	{
		ii = GetValue(p + 5);
		if (ii == 0xffff)
			return;
		else
			new_shine = (unsigned char) ii;
	}
	else if (slider < 30)
	{
		ii = GetValue(p + 6);
		if (ii == 0xffff)
			return;
		else
			new_shine = (unsigned char) ii;
	}
	else
		return;

	ii = (room * 30) + slider + 1;
	orig_shine_slider[ii] = new_shine;
	data1[ii] = new_shine;
}

//en T me llega un pedido de generar paquete RDM
//paquete: trdm,x;\r\n
unsigned char ReadPcktT(unsigned char * p)
{
	unsigned char pckt_type;

	if (*(p+4) != ',')
		return RDM_NO_PCKT;

	pckt_type = *(p+5) - '0';
	return (pckt_type + 1);		//esto es paquete 0 vuelve como 1
}


//completa el Header RDM de Kirno
//
void FillHeaderK (RDMHeaderK * pH)
{
	 //command;

	pH->start_code = 0xCC;
	pH->message_length = SIZEOF_HEADERK;		//el header tiene siempre el mismo tamaño

	pH->dest_uid[0] = 0x0F;
	pH->dest_uid[1] = 0x0F;
	pH->dest_uid[2] = 0xFF;
	pH->dest_uid[3] = 0xFF;
	pH->dest_uid[4] = 0xFF;
	pH->dest_uid[5] = 0xFF;

	pH->src_uid [0] = 0x00;
	pH->src_uid [1] = 0x00;
	pH->src_uid [2] = 0x00;
	pH->src_uid [3] = 0x00;
	pH->src_uid [4] = 0x00;
	pH->src_uid [5] = 0x00;

	pH->port_id = GET_COMMAND;
}

void FillMDBK (RDMHeaderK * pH, unsigned char * pMDB, unsigned char mdb_bytes)
{
	unsigned short checksum = 0;
	unsigned char * pSt;

	memcpy((pH + SIZEOF_HEADERK), pMDB, mdb_bytes);
	pH->message_length += mdb_bytes;
	checksum = Checksum((unsigned char *) pH, pH->message_length);

	pSt = (unsigned char *) pH;
	pSt += pH->message_length;
	*(pSt) = ShortMSB(checksum);
	*(pSt + 1) = ShortLSB(checksum);
}

/*
unsigned char GetValue (unsigned char * pn, const unsigned short * new_val)
{
	unsigned char i;
	unsigned char colon = 0;

	//me fijo la posiciones de la , o ;
	for (i = 0; i < 6; i++)
	{
		if ((*(pn + i) == ',') || ((*(pn + i) == ';')))
		{
			colon = i;
			i = 6;
		}
	}

	if ((colon == 0) || (colon >= 5))
		return 0;

	switch (colon)
	{
		case 1:
			*new_val = *pn - '0';
			break;

		case 2:
			*new_val = (*pn - '0') * 10 + (*(pn + 1) - '0');
			break;

		case 3:
			*new_val = (*pn - '0') * 100 + (*(pn + 1) - '0') * 10 + (*(pn + 2) - '0');
			break;

		case 4:
			*new_val = (*pn - '0') * 1000 + (*(pn + 1) - '0') * 100 + (*(pn + 2) - '0') * 10 + (*(pn + 2) - '0');
			break;

	}
	return 1;
}
*/

unsigned short GetValue (unsigned char * pn)
{
	unsigned char i;
	unsigned char colon = 0;
	unsigned short new_val = 0xffff;

	//me fijo la posiciones de la , o ;
	for (i = 0; i < 6; i++)
	{
		if ((*(pn + i) == ',') || ((*(pn + i) == ';')))
		{
			colon = i;
			i = 6;
		}
	}

	if ((colon == 0) || (colon >= 5))
		return 0;

	switch (colon)
	{
		case 1:
			new_val = *pn - '0';
			break;

		case 2:
			new_val = (*pn - '0') * 10 + (*(pn + 1) - '0');
			break;

		case 3:
			new_val = (*pn - '0') * 100 + (*(pn + 1) - '0') * 10 + (*(pn + 2) - '0');
			break;

		case 4:
			new_val = (*pn - '0') * 1000 + (*(pn + 1) - '0') * 100 + (*(pn + 2) - '0') * 10 + (*(pn + 2) - '0');
			break;

	}
	return new_val;
}

void Update_PWM (unsigned short pwm)
{
	Update_TIM3_CH1 (pwm);
	Update_TIM3_CH2 (4095 - pwm);
}

void UpdateDMX (unsigned char * pckt, unsigned short ch, unsigned char val)
{
	if ((ch > 0) && (ch < 512))
		*(pckt + ch) = val;
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

unsigned short Get_Temp (void)
{
	unsigned short total_ma;
	unsigned char j;

	//Kernel mejorado ver 2
	//si el vector es de 0 a 7 (+1) sumo todas las posiciones entre 1 y 8, acomodo el nuevo vector entre 0 y 7
	total_ma = 0;
	vtemp[LARGO_FILTRO] = ReadADC1 (CH_IN_TEMP);
    for (j = 0; j < (LARGO_FILTRO); j++)
    {
    	total_ma += vtemp[j + 1];
    	vtemp[j] = vtemp[j + 1];
    }

    return total_ma >> DIVISOR;
}

unsigned char MAFilter (unsigned char new_sample, unsigned char * vsample)
{
	unsigned short total_ma;
	unsigned char j;

	//Kernel mejorado ver 2
	//si el vector es de 0 a 7 (+1) sumo todas las posiciones entre 1 y 8, acomodo el nuevo vector entre 0 y 7
	total_ma = 0;
	*(vsample + LARGO_F) = new_sample;

    for (j = 0; j < (LARGO_F); j++)
    {
    	total_ma += *(vsample + j + 1);
    	*(vsample + j) = *(vsample + j + 1);
    }

    return total_ma >> DIVISOR_F;
}

unsigned short MAFilter16 (unsigned char new_sample, unsigned char * vsample)
{
	unsigned short total_ma;
	unsigned char j;

	//Kernel mejorado ver 2
	//si el vector es de 0 a 7 (+1) sumo todas las posiciones entre 1 y 8, acomodo el nuevo vector entre 0 y 7
	total_ma = 0;
	*(vsample + LARGO_F) = new_sample;

    for (j = 0; j < (LARGO_F); j++)
    {
    	total_ma += *(vsample + j + 1);
    	*(vsample + j) = *(vsample + j + 1);
    }

    return total_ma >> DIVISOR_F;
}





void EXTI4_15_IRQHandler(void)
{
	unsigned short aux;


	if(EXTI->PR & 0x0100)	//Line8
	{

		//si no esta con el USART detecta el flanco	PONER TIMEOUT ACA?????
		if ((dmx_receive_flag == 0) || (dmx_timeout_timer == 0))
		//if (dmx_receive_flag == 0)
		{
			switch (signal_state)
			{
				case IDLE:
					if (!(DMX_INPUT))
					{
						//Activo timer en Falling.
						TIM14->CNT = 0;
						TIM14->CR1 |= 0x0001;
						signal_state++;
					}
					break;

				case LOOK_FOR_BREAK:
					if (DMX_INPUT)
					{
						//Desactivo timer en Rising.
						aux = TIM14->CNT;

						//reviso BREAK
						//if (((tim_counter_65ms) || (aux > 88)) && (tim_counter_65ms <= 20))
						if ((aux > 87) && (aux < 210))	//Consola STARLET 6
						//if ((aux > 87) && (aux < 2000))		//Consola marca CODE tiene break 1.88ms
						{
							LED_ON;
							//Activo timer para ver MARK.
							//TIM2->CNT = 0;
							//TIM2->CR1 |= 0x0001;

							signal_state++;
							//tengo el break, activo el puerto serie
							DMX_channel_received = 0;
							//dmx_receive_flag = 1;

							dmx_timeout_timer = DMX_TIMEOUT;		//activo el timer cuando prendo el puerto serie
							//USARTx_RX_ENA;
						}
						else	//falso disparo
							signal_state = IDLE;
					}
					else	//falso disparo
						signal_state = IDLE;

					TIM14->CR1 &= 0xFFFE;
					break;

				case LOOK_FOR_MARK:
					if ((!(DMX_INPUT)) && (dmx_timeout_timer))	//termino Mark after break
					{
						//ya tenia el serie habilitado
						//if ((aux > 7) && (aux < 12))
						dmx_receive_flag = 1;
					}
					else	//falso disparo
					{
						//termine por timeout
						dmx_receive_flag = 0;
						//USARTx_RX_DISA;
					}
					signal_state = IDLE;
					LED_OFF;
					break;

				default:
					signal_state = IDLE;
					break;
			}
		}

		EXTI->PR |= 0x0100;
	}
}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{
		TimingDelay--;
	}

	if (wait_ms_var)
		wait_ms_var--;

	if (display_timer)
		display_timer--;

	if (timer_standby)
		timer_standby--;

	if (dmx_timeout_timer)
		dmx_timeout_timer--;

	if (timer_dmx_display_show)
		timer_dmx_display_show--;

	if (prog_timer)
		prog_timer--;

//	if (take_sample)
//		take_sample--;

	if (filter_timer)
		filter_timer--;


	/*
	//cuenta 1 segundo
	if (button_timer_internal)
		button_timer_internal--;
	else
	{
		if (button_timer)
		{
			button_timer--;
			button_timer_internal = 1000;
		}
	}
	*/
}




