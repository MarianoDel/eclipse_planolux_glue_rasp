/*
 * dmx_transceiver.h
 *
 *  Created on: 09/02/2016
 *      Author: Mariano
 */

#ifndef DMX_TRANSCEIVER_H_
#define DMX_TRANSCEIVER_H_


//--- ESTADOS DE TRANSMISION DE PAQUETES DMX512
#define PCKT_INIT			0
#define PCKT_END_BREAK		1
#define PCKT_END_MARK		2
#define PCKT_TRANSMITING	3
#define PCKT_END_TX			4


#define PCKT_UPDATE			10

#define KIND_OF_DMX		1
#define KIND_OF_RDM		2

//--- FUNCIONES DEL MODULO ---//
unsigned char SendDMX_GetStatus (void);
void SendPacket (unsigned char, unsigned char, unsigned char);
void SendPacketReset (void);
void DMX_Ena(void);
void DMX_Disa(void);

//wrappers de SendPacket
#define SendDMXPacket(X) SendPacket(X,KIND_OF_DMX,0)
#define SendRDMPacket(X,Y) SendPacket(X,KIND_OF_RDM,Y)


#endif /* DMX_TRANSCEIVER_H_ */
