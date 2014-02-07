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

#include "enc28.h"
#include <Arduino.h>
#include "tcpStack.h"

// ENC28 controller instance
ENC28Class ENC28;
TcpStackClass stack;
SocketClass socket[MAX_SOCK_NUM];

static uint32_t timeoutControl = 0;

// Init Method
//
void ENC28Class::init(void)
{
    // 1.
    // Ethernet begin
    delay(300);
    
    // initialize I/O
    // ss as output:
    pinMode(stack.getControlCsPin(), OUTPUT);
    //CS Passive mode
    stack.setCsPinPassive();
    pinMode(stack.getMosiPin(), OUTPUT);
    pinMode(stack.getSckPin(), OUTPUT);
    pinMode(stack.getMisoPin(), INPUT);
    digitalWrite(stack.getMosiPin(), LOW);
    digitalWrite(stack.getSckPin(), LOW);
    
    // initialize SPI interface
    // master mode and Fosc/2 clock:
    SPCR = (1<<SPE)|(1<<MSTR);
    SPSR |= (1<<SPI2X);
    
    // Initializating sockets
    for (unsigned int i = 0; i < MAX_SOCK_NUM; i++)
    {
        socket[i].init();
    }
    
    // Init ENC28J60
    stack.begin(); 
}

//
// Socket Methods
//
// Main method
// Returns socket status
//
uint8_t ENC28Class::readSnSR(SOCKET _s)
{
    TCP_PACKET tcpPacket;
    uint16_t tmpAcc;

    if (_s >= MAX_SOCK_NUM) return 0;

    if (stack.getPacketCount() > 0) 
    {
        // Deal with incoming packets
        // Returns true if TCP packet arrived
        // and is my IP address
        if (stack.dealRXArrival())
        {
            //Serial.print(F("Socket #: "));
            //Serial.println(_s, DEC);

            tcpPacket = stack.getTcpPacket();

            switch(socket[_s].getInternalStatus())
            {
                case SOCKET_LISTEN:
                    if (socket[_s].available(tcpPacket, SOCKET_RX_START(_s)))
                    {
                        //Serial.println(F("SYN RCVD!"));

                        // We've got SYN
                        stack.sendSynResponse(SOCKET_RX_START(_s));

                        // Adjusting internal status
                        // to reflect SYN received
                        socket[_s].setInternalStatus(SOCKET_RCVD);

                        // Grabbing source port to bind session
                        socket[_s].setSessionPort(tcpPacket.tcp.SourcePort);

                        // Getting IP chunk id
                        socket[_s].setIdNum(tcpPacket.ip.Identification.Value);

                        // starting timeout controller
                        timeoutControl = millis();
                    }
                    break;
                case SOCKET_RCVD:
                    if (tcpPacket.tcp.SourcePort == socket[_s].getSessionPort())
                    {
                        //Serial.println(F("SOCKET RCVD!"));

                        if (socket[_s].established(tcpPacket, SOCKET_RX_START(_s)))
                        {
                            //Serial.println(F("SOCKET RCVD!"));

                            // FAIL! Totally buged!
                            // Checking if IP id is in sequence!
                            //if (tcpPacket.ip.Identification.Value > socket[_s].getIdNum())
                            //{
                            //Serial.println(F("INTERNAL ESTABLISHED!"));

                            // Adjusting internal status
                            // to reflect internal established
                            socket[_s].setInternalStatus(SOCKET_ESTABLISHED);
                            //}
                        }
                        else if (socket[_s].finished(tcpPacket, SOCKET_RX_START(_s)))
                        {
                            //Serial.println(F("FINISHED!"));

                            // Update Seq and Ack numbers
                            socket[_s].setSeqNum(tcpPacket.tcp.AckNum);
                            socket[_s].setAckNum(tcpPacket.tcp.SeqNum + 1);

                            // Sending final ACK for sending part
                            stack.sendAckResponse(tcpPacket, socket[_s].getSeqNum(), socket[_s].getAckNum());

                            // Ok, we are good to get back
                            // to closing process
                            socket[_s].setInternalStatus(SOCKET_FIN_WAIT);
                        }
                    }
                    break;
                case SOCKET_ESTABLISHED:
                    if (tcpPacket.tcp.SourcePort == socket[_s].getSessionPort())
                    {
                        if (socket[_s].pushed(tcpPacket, SOCKET_RX_START(_s)))
                        {
                           // Serial.println(F("ESTABLISHED!"));

                            //Serial.print(F("RxSize: "));
                            //Serial.println(tcpPacket.ip.TotalLength.Value - TOTAL_HEADER_SIZE, DEC);
                            //delay(100);



                            // Client's payload
                            socket[_s].setClientPayload(tcpPacket.ip.TotalLength.Value - TOTAL_HEADER_SIZE);

                            // Configuring ACK/SEQ numbers
                            socket[_s].setSeqNum(tcpPacket.tcp.AckNum);
                            socket[_s].setAckNum((uint32_t)(tcpPacket.tcp.SeqNum + socket[_s].getClientPayload()));
                            socket[_s].setLastPayload(0);

                            // We've got PUSH
                            // sending ACK back 
                            // to client quit bothering
                            stack.sendPshResponse(tcpPacket, socket[_s].getSeqNum(), socket[_s].getAckNum());

                            // Set pointers positions
                            socket[_s].setRxPointer(SOCKET_RX_START(_s) + TCP_BUFF_SIZE);
                            
                            // Calc. endRx pointer
                            tmpAcc = socket[_s].getRxPointer() + socket[_s].getClientPayload();
                            if (tmpAcc <= SOCKET_RX_END(_s))
                                socket[_s].setStopRx(tmpAcc);
                            else
                                socket[_s].setStopRx(SOCKET_RX_END(_s));

                            // Adjusting internal status
                            // to reflect reading step
                            socket[_s].setInternalStatus(SOCKET_READING);

                            // Adjusting status
                            // to start reading
                            socket[_s].setStatus(SnSR::ESTABLISHED);

                            // Read Push size
                            // This must be last instruction
                            // because it will release reading routine
                            socket[_s].setRxSize(socket[_s].getClientPayload());
                        }
                        else if (socket[_s].finished(tcpPacket, SOCKET_RX_START(_s)))
                        {
                            // Update Seq and Ack numbers
                            socket[_s].setSeqNum(tcpPacket.tcp.AckNum);
                            socket[_s].setAckNum(tcpPacket.tcp.SeqNum + 1);

                            // Sending final ACK for sending part
                            stack.sendAckResponse(tcpPacket, socket[_s].getSeqNum(), socket[_s].getAckNum());

                            // Ok, we are good to get back
                            // to closing process
                            socket[_s].setInternalStatus(SOCKET_FIN_WAIT);
                        }
                    }
                    break;
                case SOCKET_READING:
                    // Rev2 - Updating RX pointer when 
                    //        we got any packet
                    //        because the receiveing process 
                    //        mess with our RX Pointer
                    socket[_s].setRxPointer(socket[_s].getLastRx());

                    //Serial.print(F("Last Rx: "));
                    //Serial.println(socket[_s].getLastRx(), DEC);
                    //delay(100);

                    break;
                case SOCKET_WRITING:
                    break;
                case SOCKET_SENDING:
                    // Checking for sync ACK's
                    //
                    // Rev2:
                    // In this state, means we was writing
                    // the buffer but we reached the top.
                    // Ok, we already sent the TCP packet,
                    // but we are waiting for ACK from client
                    // to unlock writing and get back to its state

                    // Verifying if source port is in session
                    if (tcpPacket.tcp.SourcePort == socket[_s].getSessionPort())
                    {
                        if (socket[_s].acked(tcpPacket, SOCKET_RX_START(_s)))
                        {
                            //Serial.println(F("GOT ACK!"));

                            // Update Seq and Ack numbers
                            socket[_s].setSeqNum(tcpPacket.tcp.AckNum);
                            socket[_s].setAckNum(tcpPacket.tcp.SeqNum);

                            // Ok, we are good to get back
                            // to writing process
                            socket[_s].setInternalStatus(SOCKET_WRITING);
                        }
                    }
                    break;
                case SOCKET_LAST_SEND:
                    // Rev2: Disconn was called and
                    // Rev2: remaining buffer already sent
                    if (tcpPacket.tcp.SourcePort == socket[_s].getSessionPort())
                    {
                        if (socket[_s].acked(tcpPacket, SOCKET_RX_START(_s)))
                        {
                            //Serial.println(F("GOT LAST SEND!"));

                            // Update Seq and Ack numbers
                            socket[_s].setSeqNum(tcpPacket.tcp.AckNum);
                            socket[_s].setAckNum(tcpPacket.tcp.SeqNum);

                            // Sending final ACK for sending part
                            stack.sendAckResponse(tcpPacket, socket[_s].getSeqNum(), socket[_s].getAckNum());

                            // Ok, we are good to get back
                            // to closing process
                            socket[_s].setInternalStatus(SOCKET_CLOSE_WAIT);
                        }
                    }
                    break;
                case SOCKET_CLOSE_WAIT:
                    // Out of receiveing packets routine
                    break;
                case SOCKET_LAST_ACK:
                    if (tcpPacket.tcp.SourcePort == socket[_s].getSessionPort())
                    {
                        if (socket[_s].finished(tcpPacket, SOCKET_RX_START(_s)))
                        {
                            //Serial.println(F("GOT LAST ACK!"));

                            // Update Seq and Ack numbers
                            socket[_s].setSeqNum(tcpPacket.tcp.AckNum);
                            socket[_s].setAckNum(tcpPacket.tcp.SeqNum);

                            // Sending final ACK for sending part
                            stack.sendAckResponse(tcpPacket, socket[_s].getSeqNum(), socket[_s].getAckNum() + 1);

                            // Ok, we are good to get back
                            // to closing process
                            socket[_s].setInternalStatus(SOCKET_CLOSED);
                            socket[_s].setStatus(SnSR::CLOSED);
                        }
                    }
                    break;
                case SOCKET_CLOSED:
                    // We got closed by client.
                    socket[_s].setInternalStatus(SOCKET_CLOSED);
                    socket[_s].setStatus(SnSR::CLOSED);
                    break;
                default:
                    break;
            }
        }

        
        stack.discardPacket();
    }

    // Rev2: Internal status callback's 
    // 
    if (socket[_s].getInternalStatus() == SOCKET_CLOSE_WAIT)
    {
        //Serial.println(F("SENT FIN FOR CLOSE_WAIT!"));

        stack.sendFinRequest(SOCKET_RX_START(_s), SOCKET_CLOSE_WAIT);

        // Ok, we now just waiting last ack
        socket[_s].setInternalStatus(SOCKET_LAST_ACK);
    }
    else if (socket[_s].getInternalStatus() == SOCKET_FIN_WAIT)
    {
        //Serial.println(F("SENT FIN FOR FIN_WAIT!"));
        
        stack.sendFinRequest(SOCKET_RX_START(_s), SOCKET_FIN_WAIT);

        // We dont need to wait last ACK, 
        // just close the socket
        socket[_s].setInternalStatus(SOCKET_CLOSED);

        // Internally we just set status when ack arrives
    }

    // checking timeout
    if (((timeoutControl + TIMEOUT) < millis()) && (timeoutControl != 0))
    {
        socket[_s].setInternalStatus(SOCKET_CLOSED);
        socket[_s].setStatus(SnSR::CLOSED);

        timeoutControl = 0;
    }

    // return pre-processed socket status
    return socket[_s].getStatus();
}

// Command trigger
//
void ENC28Class::execCmdSn(SOCKET s, SockCMD _cmd)
{
    unsigned char macHR;

    if (s >= MAX_SOCK_NUM) return;

    
    switch(_cmd)
    {
        case Sock_RECV:
            // Receive socket process
            socket[s].setRxSize(socket[s].getStopRx() - socket[s].getRxPointer());
            break;
        case Sock_SEND:
            // Send socket process 
            // Nothing to do.            
            break;
        case Sock_OPEN:
            // Open socket process
            socket[s].setStatus(SnSR::INIT);

            macHR = stack.getHardwareRevision();
            Serial.print(F("Ethernet Hardware Rev.: "));
            Serial.println(macHR, DEC);


            break;
        case Sock_CLOSE:
            // Close socket process
            socket[s].setStatus(SnSR::CLOSED);
            socket[s].setInternalStatus(SOCKET_CLOSED);
            break;
        case Sock_LISTEN:
            // Reset socket
            socket[s].init();

            // Puts socket in a listening process
            socket[s].setStatus(SnSR::LISTEN);
            socket[s].setInternalStatus(SOCKET_LISTEN);


            break;
        case Sock_CONNECT:
            // Connect socket process
            break;
        case Sock_DISCON:
            // Disconnect socket process
            if (socket[s].getInternalStatus() == SOCKET_WRITING)
            {
                if (socket[s].getTxPointer() > (SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1))
                {
                    // If we in this state, 
                    // means we need to perform a send
                    // to flush the buffer
                    stack.prepareSend(SOCKET_RX_START(s), 
                        socket[s].getLastTx() - (SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1), SOCKET_TX_START(s),
                        socket[s].getSeqNum(), socket[s].getAckNum());

                    stack.sendPush(SOCKET_TX_START(s), 
                        socket[s].getLastTx() - (SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1));

                    // take time
                    // the spice must flow!
                    delay(1);

                    // Then we reset TX Pointer
                    socket[s].setTxPointer(SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1);

                    // Setting Last sending status
                    socket[s].setInternalStatus(SOCKET_LAST_SEND);
                }
                else
                {
                    // In this state, we already
                    // sent all data packets

                    //Serial.println(F("GIVE LAST SEND!"));

                    // Update Seq and Ack numbers
                    //socket[s].setSeqNum(tcpPacket.tcp.AckNum);
                    //socket[s].setAckNum(tcpPacket.tcp.SeqNum);

                    // Sending final ACK for sending part
                    //stack.sendAckResponse(tcpPacket, socket[_s].getSeqNum(), socket[_s].getAckNum());

                    // Ok, we are good to get back
                    // to closing process
                    socket[s].setInternalStatus(SOCKET_CLOSE_WAIT);
                }
            }
            
            
            //socket[s].setStatus(SnSR::FIN_WAIT);
            //socket[s].setInternalStatus(SOCKET_FIN_WAIT);
            
            break;
        default:
            // command not implemented
            break;
    }
}

// Send data
//
void ENC28Class::send_data_processing(SOCKET s, const uint8_t *data, uint16_t len)
{
    if (s >= MAX_SOCK_NUM) return;

    send_data_processing_offset(s, 0, data, len);
}

// Send data with offset
//
void ENC28Class::send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
    // Rev2: Using half buffer for sending
    // Rev2: some bug at EthernetSup I must solve.
    uint16_t nextPt = 0;
    uint16_t txlen = (uint16_t)(SOCKET_TX_LEN(s) / 2);

    if (s >= MAX_SOCK_NUM) return;

    // Adjusting internal status
    // to reflect writing step
    socket[s].setInternalStatus(SOCKET_WRITING);

    // Recalc Tx Pointer
    // based on last recalc
    if (socket[s].getLastTx() == 0)
    {
        //Serial.println(F("Clear TX Pointer!"));
        //delay(1);

        socket[s].setTxPointer(SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1);
    }
    else
    {
        // Getting last TX to avoid 
        // hardware mess with our pointers
        socket[s].setTxPointer(socket[s].getLastTx());
    }

    nextPt = (socket[s].getTxPointer() + len + data_offset) - (SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1);
    
    /*for (int i = 0; i < 10; i++)
    {
        Serial.print((char)data[i]);
    }
    Serial.println();
    */
    /*
    Serial.print(F("Tx Pointer: "));
    Serial.println(nextPt, DEC);
    Serial.print(F("Sock Len: "));
    Serial.println(txlen, DEC);
    delay(1);
    */


    // checking if hardware buffer is full
    // Rev2: Using 512b buffer
    // Rev2: Tryied with 1k, but something went wrong
    // Rev2: Dont know why, but when writing reaches
    // Rev2: about 800 bytes, it stops writing,
    // Rev2: having more data to write.
    if (nextPt >= txlen)
    {
        //Serial.println(F("Buffer full! Sending..."));
        //Serial.print(F("Buff Size:"));
        //Serial.println(socket[s].getLastTx() - (SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1));

        // If the buffer is full, we must send it
        stack.prepareSend(SOCKET_RX_START(s), 
                    socket[s].getLastTx() - (SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1), SOCKET_TX_START(s),
                    socket[s].getSeqNum(), socket[s].getAckNum());

        stack.sendPush(SOCKET_TX_START(s), 
                    socket[s].getLastTx() - (SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1));

        // take time
        // the spice must flow!
        delay(1);

        // Then we reset TX Pointer
        socket[s].setTxPointer(SOCKET_TX_START(s) + TCP_BUFF_SIZE + 1);

        // Adjusting internal status
        // to reflect sending step
        socket[s].setInternalStatus(SOCKET_SENDING);
    }

    // Adjusting Tx pointer with offset
    socket[s].setTxPointer(socket[s].getTxPointer() + data_offset);

    // DMA Copying TX Buffer
    stack.sendProcessing((uint8_t *)data, len, socket[s].getTxPointer());

    // Setting last tx pointer
    socket[s].setLastTx(socket[s].getTxPointer());
}

// Receive data
//
void ENC28Class::recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
    if (s >= MAX_SOCK_NUM) return;

    uint16_t _rxPointer = socket[s].getRxPointer();
    stack.readSocketBuffer(data, len, _rxPointer);

    if (!peek)
    {
        socket[s].setLastRx(_rxPointer + len);
        socket[s].setRxPointer(socket[s].getLastRx());
    }
}

// Get TX Free size
//
uint16_t ENC28Class::getTXFreeSize(SOCKET s)
{
    if (s >= MAX_SOCK_NUM) return 0;

    return (SOCKET_TX_END(s) - socket[s].getTxPointer());
}

// Get RX received buffer size
// As received size is calculated by RX pointer location,
// it will decreased while reading
//
uint16_t ENC28Class::getRXReceivedSize(SOCKET s)
{
    if (s >= MAX_SOCK_NUM) return 0;

    return socket[s].getRxSize();
}

// Read Mode
//
uint8_t ENC28Class::readSnMR(SOCKET _s)
{
    if (_s >= MAX_SOCK_NUM) return 0;

    return 1;
}

// Read interrupt
//
uint8_t ENC28Class::readSnIR(SOCKET _s)
{
    if (_s >= MAX_SOCK_NUM) return 0;

    if (socket[_s].getInternalStatus() == SOCKET_WRITING)
    {
        return SnIR::SEND_OK;
    }
    
    return 1;
}

// Write interrupt
//
void ENC28Class::writeSnIR(SOCKET _s, uint8_t _flag)
{
    if (_s >= MAX_SOCK_NUM) return;

    // Clear interrupt flag
    if (_flag == SnIR::SEND_OK)
    {
        //socket[_s].myFlagSendOk = 0;
    }
}

// Read Rx pointer
//
uint16_t ENC28Class::readSnRX_RD(SOCKET _s)
{
    if (_s >= MAX_SOCK_NUM) return 0;

    uint16_t p = stack.getRxPointer();
    return p;
}

// Read Tx pointer
//
uint16_t ENC28Class::readSnTX_WR(SOCKET _s)
{
    if (_s >= MAX_SOCK_NUM) return 0;

    uint16_t p = stack.getTxPointer();
    return p;
}

// Write Rx pointer
//
void ENC28Class::writeSnRX_RD(SOCKET _s, uint16_t _addr)
{
    if (_s >= MAX_SOCK_NUM) return;

    stack.setRxPointer(_addr);
}

// Write Tx Pointer
//
void ENC28Class::writeSnTX_WR(SOCKET _s, uint16_t _addr)
{
    if (_s >= MAX_SOCK_NUM) return;

    stack.setTxPointer(_addr);
}

// Write Mode
//
void ENC28Class::writeSnMR(SOCKET _s, uint8_t _flag)
{
    if (_s >= MAX_SOCK_NUM) return;

    // 4.
    // Socket new instance
	
    // Pointers to buffers ends and starts
    if (_flag == SnMR::TCP)
    {
        // Call Open Func
        stack.open();

        // Enabling reception                
        stack.start();
    }
    else
    {
        // UDP and other protocols not implemented
    }
}

// Write server port
//
void ENC28Class::writeSnPORT(SOCKET _s, uint16_t _addr)
{
    if (_s >= MAX_SOCK_NUM) return;

    // 5.
    // Set listening port
    stack.setSourcePort(_addr);
}

//
// Server Methods
//
// Get Ip address
//
void ENC28Class::getIPAddress(uint8_t *_addr)
{
    stack.getIpAddress(_addr);
}

// Get gateway address
//
void ENC28Class::getGatewayIp(uint8_t *_addr)
{
    stack.getGatewayIp(_addr);
}

// Get subnet mask address
//
void ENC28Class::getSubnetMask(uint8_t *_addr)
{
    stack.getSubnetMask(_addr);
}

// Get MAC address
//
void ENC28Class::getMACAddress(uint8_t *_addr)
{
    stack.getMacAddress(_addr);
}

// Set MAC address
//
void ENC28Class::setMACAddress(uint8_t *_addr)
{
    // 2.
    // Ethernet begin
    stack.setMacAddress(_addr);

    // Init MAC Address
    //(unsigned char*)&
    stack.init(_addr);
}

// Set Ip address
//
void ENC28Class::setIPAddress(uint8_t *_addr)
{
    // 3.
    // Ethernet begin
    stack.setIpAddress(_addr);
}

// Set Gateway address
//
void ENC28Class::setGatewayIp(uint8_t *_addr)
{
    // 3.1.
    // Ethernet begin
    stack.setGatewayIp(_addr);
}

// Set Subnet mask address
//
void ENC28Class::setSubnetMask(uint8_t *_addr)
{
    // 3.2.
    // Ethernet begin
    stack.setSubnetMask(_addr);
}

// Not implemented
//
void ENC28Class::writeSnDIPR(SOCKET _s, uint8_t* _addr){}
void ENC28Class::writeSnDPORT(SOCKET _s, uint16_t _addr){}
void ENC28Class::read_data(SOCKET s, volatile uint8_t *src, volatile uint8_t *dst, uint16_t len){}
void ENC28Class::setRetransmissionTime(uint16_t _timeout){}
void ENC28Class::setRetransmissionCount(uint8_t _retry){}
