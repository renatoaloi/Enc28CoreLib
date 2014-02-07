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

#include "tcpStack.h"
#include <Arduino.h>
#include <MemoryFree.h>

 extern "C"
{
    #include "enc28j60.h"
}

static void WaitForDMACopy(void);

// Init Method
//
void SocketClass::init()
{
    sizeRx = 0;
    stopRx = 0;
    lastRx = 0;
    lastTx = 0;

    status = SnSR::CLOSED;
    internalStatus = SOCKET_CLOSED;

    sessionPort = 0;  
    idNum = 0;

    clientPayload = 0;
    lastPayload = 0;

    ackNum = 0;
    seqNum = 0;
}

uint8_t SocketClass::available(TCP_PACKET packet, uint16_t _startRx)
{
    if (packet.tcp.Flags.Bits.FlagSYN)
    {
        // Copy packet from
        // ENC28J60's RX buffer to socket RX buffer
        // With options 'cause we send
        // MMS option along with SYN/ACK
        DMACopyTo(RX, _startRx, TCP_BUFF_SIZE_W_OPT);
        WaitForDMACopy();

        return 1;
    }

    return 0;
}

uint8_t SocketClass::established(TCP_PACKET packet, uint16_t _startRx)
{
    if (packet.tcp.Flags.Bits.FlagACK)
    {
        // Copy packet from
        // ENC28J60's RX buffer to socket RX buffer
        DMACopyTo(RX, _startRx, TCP_BUFF_SIZE);
        WaitForDMACopy();

        return 1;
    }

    return 0;
}

uint8_t SocketClass::pushed(TCP_PACKET packet, uint16_t _startRx)
{
    if (packet.tcp.Flags.Bits.FlagPSH)
    {
        // Copy packet from
        // ENC28J60's RX buffer to socket RX buffer
        //DMACopyTo(RX, _startRx, TCP_BUFF_SIZE);
        DMACopyTo(RX, _startRx, packet.ip.TotalLength.Value + ETH_BUFF_SIZE);
        WaitForDMACopy();

        return 1;
    }

    return 0;
}

uint8_t SocketClass::acked(TCP_PACKET packet, uint16_t _startRx)
{
    if (packet.tcp.Flags.Bits.FlagACK)
    {
        // Copy packet from
        // ENC28J60's RX buffer to socket RX buffer
        //DMACopyTo(RX, _startRx, TCP_BUFF_SIZE);
        DMACopyTo(RX, _startRx, TCP_BUFF_SIZE);
        WaitForDMACopy();

        return 1;
    }

    return 0;
}

uint8_t SocketClass::finished(TCP_PACKET packet, uint16_t _startRx)
{
    if (packet.tcp.Flags.Bits.FlagFIN)
    {
        // Copy packet from
        // ENC28J60's RX buffer to socket RX buffer
        //DMACopyTo(RX, _startRx, TCP_BUFF_SIZE);
        DMACopyTo(RX, _startRx, TCP_BUFF_SIZE);
        WaitForDMACopy();

        return 1;
    }

    return 0;
}

uint8_t SocketClass::getStatus()
{
	return status;
}

uint8_t SocketClass::getInternalStatus()
{
	return internalStatus;
}

void SocketClass::setStatus(uint8_t st)
{
	status = st;
}

void SocketClass::setInternalStatus(uint8_t st)
{
	internalStatus = st;
}

void SocketClass::setSessionPort(uint16_t port)
{
    sessionPort = port;
}

uint16_t SocketClass::getSessionPort()
{
    return sessionPort;
}

void SocketClass::setIdNum(uint16_t id)
{
    idNum = id;
}

uint16_t SocketClass::getIdNum()
{
    return idNum;
}

void SocketClass::setRxSize(uint16_t size)
{
    sizeRx = size;
}

uint16_t SocketClass::getRxSize()
{
    return sizeRx;
}

void SocketClass::setRxPointer(uint16_t _pt)
{
    SOCKETSetRxPointer(_pt);
}

uint16_t SocketClass::getRxPointer()
{
    return SOCKETGetRxPointer();
}

void SocketClass::setTxPointer(uint16_t _pt)
{
    SOCKETSetTxPointer(_pt);
}

uint16_t SocketClass::getTxPointer()
{
    return SOCKETGetTxPointer();
}

void SocketClass::setClientPayload(uint16_t _payload)
{
    clientPayload = _payload;
}

uint16_t SocketClass::getClientPayload()
{
    return clientPayload;
}

void SocketClass::setLastPayload(uint16_t _payload)
{
    lastPayload = _payload;
}

uint16_t SocketClass::getLastPayload()
{
    return lastPayload;
}

void SocketClass::setAckNum(uint32_t _num)
{
    ackNum = _num;
}

uint32_t SocketClass::getAckNum()
{
    return ackNum;
}

void SocketClass::setSeqNum(uint32_t _num)
{
    seqNum = _num;
}

uint32_t SocketClass::getSeqNum()
{
    return seqNum;
}

uint16_t SocketClass::getStopRx()
{
    return stopRx;
}

void SocketClass::setStopRx(uint16_t _stop)
{
    stopRx = _stop;
}

uint16_t SocketClass::getLastRx()
{
    return lastRx;
}
    
void SocketClass::setLastRx(uint16_t _last)
{
    lastRx = _last;
}

uint16_t SocketClass::getLastTx()
{
    return lastTx;
}
    
void SocketClass::setLastTx(uint16_t _last)
{
    lastTx = _last;
}

// Wait for DMA Copy finish
//
static void WaitForDMACopy(void)
{
    unsigned long timerSendTrigger = millis();

    while(!IsDMACopyDone())
    {
        if (timerSendTrigger + 1000 < millis())
        {
            //Serial.println(F("ERRO DMA COPY!"));
            //delay(10);
            break;
        }
    }
}
