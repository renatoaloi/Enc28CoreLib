/*
 * Copyright (c) 2013 by Renato Aloi <renato.aloi@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 * 
 * 2013-12-26 :: REV 1 by Renato Aloi (missing DHCP part)
 * 2014-01-28 :: REV 2 by Renato Aloi (general clean up)
*/

#ifndef	TCPSTACK_H_INCLUDED
#define	TCPSTACK_H_INCLUDED

#include <stdint.h>
#include "enc28socket.h"
#include "enc28typedef.h"

#define SOCKET_CLOSED 		0x00
#define SOCKET_LISTEN 		0x01
#define SOCKET_RCVD			0x10
#define SOCKET_ESTABLISHED 	0x0F
#define SOCKET_CLOSE_WAIT	0xF0
#define SOCKET_LAST_ACK		0xFF
#define SOCKET_FIN_WAIT		0xF1
#define SOCKET_READING		0xA0
#define SOCKET_WRITING		0x0A
#define SOCKET_SENDING		0xAA
#define SOCKET_LAST_SEND	0xA7

#define TIMEOUT				15000L

class TcpStackClass 
{

public:
	void init(unsigned char*);
	void begin();
	void open();
	void start();

	char getMosiPin();
	char getSckPin();
	char getMisoPin();
	char getControlCsPin();

	void setCsPinPassive();

	uint16_t getRxPointer();
	uint16_t getTxPointer();

	void setRxPointer(uint16_t);
	void setTxPointer(uint16_t);

	uint8_t dealRXArrival();
	uint8_t getPacketCount();

	void setSourcePort(uint16_t);
	void setSubnetMask(uint8_t*);
	void getSubnetMask(uint8_t*);
	void setGatewayIp(uint8_t*);
	void getGatewayIp(uint8_t*);
	void setIpAddress(uint8_t*);
	void getIpAddress(uint8_t*);
	void setMacAddress(uint8_t*);
	void getMacAddress(uint8_t*);

	uint8_t getHardwareRevision();
	void discardPacket();

	TCP_PACKET getTcpPacket();

	void sendSynResponse(uint16_t);
	void sendPshResponse(TCP_PACKET, uint32_t, uint32_t);
	void sendAckResponse(TCP_PACKET, uint32_t, uint32_t);
	void sendFinRequest(uint16_t, uint8_t);

	void readSocketBuffer(uint8_t*, uint16_t, uint16_t);

	void sendProcessing(const uint8_t*, uint16_t, uint16_t);
	void prepareSend(uint16_t, uint16_t, uint16_t, uint32_t, uint32_t);
	void sendPush(uint16_t, uint16_t);
	
private:
	void SendBufferTx(void);
};

extern TcpStackClass TcpStack;


class SocketClass
{
private:
    uint16_t    sizeRx;
    uint16_t	stopRx;
    uint16_t 	lastRx;
    uint16_t 	lastTx;

    uint8_t     status;
    uint8_t     internalStatus; 

    uint16_t    sessionPort;  
    uint16_t    idNum;  

    uint16_t 	clientPayload;
    uint16_t	lastPayload;

    uint32_t    ackNum;
    uint32_t    seqNum;

public:
	void init();
	
	uint8_t getStatus();
	uint8_t getInternalStatus();

	void setStatus(uint8_t);
	void setInternalStatus(uint8_t);

	uint8_t available(TCP_PACKET, uint16_t);
	uint8_t established(TCP_PACKET, uint16_t);
	uint8_t pushed(TCP_PACKET, uint16_t);
	uint8_t acked(TCP_PACKET, uint16_t);
	uint8_t finished(TCP_PACKET, uint16_t);

	uint16_t getSessionPort();
	void setSessionPort(uint16_t);

	uint16_t getIdNum();
	void setIdNum(uint16_t);

	uint16_t getRxSize();
	void setRxSize(uint16_t);

	uint16_t getTxPointer();
	void setTxPointer(uint16_t);
	uint16_t getRxPointer();
	void setRxPointer(uint16_t);

	uint16_t getLastPayload();
	void setLastPayload(uint16_t);
	uint16_t getClientPayload();
	void setClientPayload(uint16_t);

	uint32_t getSeqNum();
	void setSeqNum(uint32_t);
	uint32_t getAckNum();
	void setAckNum(uint32_t);

	uint16_t getStopRx();
	void setStopRx(uint16_t);
	uint16_t getLastRx();
	void setLastRx(uint16_t);
	uint16_t getLastTx();
	void setLastTx(uint16_t);
};

extern SocketClass SocketStack;

#endif