/*
 * Copyright (c) 2013 by Renato Aloi <renato.aloi@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 * 
 * 2013-12-26 :: REV 1 by Renato Aloi (missing DHCP part)
 */


#include "enc28.h"
#include <Arduino.h>

extern "C"
{
    #include "enc28j60.h"
    #include "checksum.h"
}

// DO NOT TURN ON ANY DEBUG MACRO
// when using EthernetSup
#undef DEBUG_REV1
#undef DEBUG_REV1_HEAVY

#define DEBUG_REV1_LIGHT

// See Rev 1 - Warn 1
static unsigned char macHR = 0;

// ENC28 controller instance
ENC28Class ENC28;

// Static variables
static unsigned     char        socketBuffer[MAX_BUFF_SIZE];
static volatile     SocketVal   socket[MAX_SOCK_NUM];

static unsigned     int         serverSourcePort    = 0;

static unsigned     char        _my_macaddr[]       = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned     char        _my_ipaddr[]        = { 0x00, 0x00, 0x00, 0x00 };
static unsigned     char        _my_gataddr[]       = { 0x00, 0x00, 0x00, 0x00 };
static unsigned     char        _my_subaddr[]       = { 0x00, 0x00, 0x00, 0x00 };

static TCP_PACKET               tcpPacket2;

// Static funcs declarations
static void readSocketPackets(void);
static void RecalcTxPointer(SOCKET i);
static void treatBackgroundProcesses(void);
static void InitSocket(SOCKET i);
static void ClearSocket(SOCKET i);

static void readBufferLittleEndian(unsigned char* buf, unsigned int size);
static void readSocketBufferLittleEndian(unsigned char* buf, unsigned int size, unsigned int start);
static void writeBufferBigEndian(unsigned char* buf, unsigned int size);
static void writeSocketBufferBigEndian(unsigned char* buf, unsigned int size, unsigned int start);

static uint8_t DealRXArrival(void);
static void SendSynResponse(SOCKET i);

// See Bug 1 Rev 1
//static uint16_t SendAckResponse(SOCKET i);
static uint16_t GetRequestPayload(SOCKET i);
static void SendAckResponse(SOCKET i);
static uint32_t GetRequestSeqNum(SOCKET i);
static uint32_t GetRequestAckNum(SOCKET i);
static uint16_t GetSourcePort(SOCKET i);

// New funcs - See Bug 2 - Rev 1
static void PrepareAckWithPayload(SOCKET i, unsigned int _payload);
static void SendAckWithPayload(SOCKET i, unsigned int _payload);
//
static void SendFinRequest(SOCKET i);

static void WaitForDMACopy(void);
static void SendBufferTx(void);

// Init Method
//
void ENC28Class::init(void)
{
    // 1.
    // Ethernet begin
    delay(300);
    
    // initialize I/O
    // ss as output:
    pinMode(ENC28J60_CONTROL_CS, OUTPUT);
    CSPASSIVE; 
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_SCK, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    digitalWrite(SPI_MOSI, LOW);
    digitalWrite(SPI_SCK, LOW);
    
    // initialize SPI interface
    // master mode and Fosc/2 clock:
    SPCR = (1<<SPE)|(1<<MSTR);
    SPSR |= (1<<SPI2X);
    
    // Initializating sockets
    for (unsigned int i = 0; i < MAX_SOCK_NUM; i++)
    {
        InitSocket(i);
    }
    
    // Init ENC28J60
    MACInit(); 
}

// Main method
// Returns socket status
//
uint8_t ENC28Class::readSnSR(SOCKET _s)
{
    if (_s >= MAX_SOCK_NUM) return 0;

    if (IsMACSendTx())
    {
        // Check if we have packets to treat
        //
        readSocketPackets();
        
        // Background Process
        //
        treatBackgroundProcesses();

        // Loop Counter
        socket[_s].loopCounter++;
    }
    
    // return pre-processed socket status
    return socket[_s].status;
}

// Command trigger
//
void ENC28Class::execCmdSn(SOCKET s, SockCMD _cmd)
{
    if (s >= MAX_SOCK_NUM) return;

    
    switch(_cmd)
    {
        case Sock_RECV:
            // Receive socket process -- must be first for performance stuff
            
            // Resizing buffer
            // ptrRdRx already incremented
            // no need to add 1 to size
            socket[s].sizeRx = (socket[s].endRx - socket[s].ptrRdRx);
            
            break;
        case Sock_SEND:
            // Send socket process -- must be first for performance stuff

#ifdef DEBUG_REV1_HEAVY
            Serial.print("Sock_SEND: ");
            Serial.println(millis());
#endif

            // Setting do send flag
            socket[s].myFlagDoSend = 1;

            // Clear Got Sync Ack flag
            socket[s].myFlagGotSyncAck = 0;

            // unlocking
            socket[s].myFlagLock = 0;
            
            break;
        case Sock_OPEN:
            // Open socket process
#ifdef DEBUG_REV1_HEAVY
            Serial.println("Sock_OPEN");
#endif
            
            // See Rev 1 - Warn 1
            // Now we can output Hardware version
			macHR = MACHardwareRevision();
			
			// Debugging version for compatibility test
#ifdef DEBUG_REV1
            Serial.print("Ethernet Hardware Rev.: ");
            Serial.println(macHR, DEC);
#endif 

#ifdef DEBUG_REV1_LIGHT
            Serial.print("Ethernet Hardware Rev.: ");
            Serial.println(macHR, DEC);
#endif 
            
            // See Rev 1 - Warn 1
            // Check version
			if (macHR < 6) 
			{
#ifdef DEBUG_REV1
                Serial.println("Version not compatible!");
#endif

#ifdef DEBUG_REV1_LIGHT
                Serial.println("Version not compatible!");
#endif
				// FAIL! Version not compatible
				// Need to be compatible with 
				// ENC28J60 ERRATA Rev.B7
				socket[s].status = SnSR::CLOSED;
			}
			else
			{
				// Update socket status 
				socket[s].status = SnSR::INIT;
			}
            
            break;
        case Sock_CLOSE:
            // Close socket process
#ifdef DEBUG_REV1_HEAVY
            Serial.println("Sock_CLOSE");
            Serial.print("Loop Counter: ");
            Serial.println(socket[s].loopCounter);
#endif
            // Setting CLOSED status
            socket[s].status = SnSR::CLOSED;

            // unlocking
            socket[s].myFlagLock = 0;
            
            break;
        case Sock_LISTEN:
            // Puts socket in a listening process
#ifdef DEBUG_REV1_HEAVY
            Serial.println("Sock_LISTEN");
#endif

            // Update socket status 
            socket[s].status = SnSR::LISTEN;

            // Release lock
            if (socket[s].myFlagLock) 
                socket[s].myFlagLock = 0;
            
            break;
        case Sock_CONNECT:
            // Connect socket process
#ifdef DEBUG_REV1_HEAVY
            Serial.println("Sock_CONNECT");
#endif
            
            break;
        case Sock_DISCON:
            // Disconnect socket process
#ifdef DEBUG_REV1_HEAVY
            Serial.println("Sock_DISCON");
#endif

            // Calling disconnect
            socket[s].myFlagDiscon = 1;

            socket[s].loopCounter = 0;

            // Release lock
            // we need to receive last
            // ACK before go to close wait
            if (socket[s].myFlagLock) socket[s].myFlagLock = 0;
            
            break;
        default:
#ifdef DEBUG_REV1_HEAVY
            Serial.println("Command not implemented!");
#endif
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
    if (s >= MAX_SOCK_NUM) return;

    socket[s].ptrWrTx = readSnTX_WR(s);
    socket[s].ptrWrTx += data_offset;
    
    // dont know why yet,
    // but only works this way
	// Rev1: TODO for another Rev.
    unsigned char data2[len];
    for (unsigned int m=0; m<len; m++)
    {
        data2[m]=data[m];
    }
    
    // save len because it is decreased
    // at SOCKETWriterBuffer func.
    // Old: dont know if really works
	// Rev1: Yep, it works indeed!
    unsigned int savedLen = len;
    
    // we dont have circular socket's buffer
    // one write instruction will do
	//
	// Rev1: Instead of circular buffer, we use ENC28J60's
	// Rev1: DMA buffer Copy to perform Zero-Copy operations.
	// Rev1: See DMACopyTo() and DMACopyFrom() functions.
	//
    //SOCKETWriteBuffer((unsigned char*)&data, len, socket[s].ptrWrTx);
    SOCKETWriteBuffer(data2, len, socket[s].ptrWrTx);
  
    // updating pointer
    socket[s].ptrWrTx += savedLen;
    writeSnTX_WR(s, socket[s].ptrWrTx);
}

// Receive data
//
void ENC28Class::recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
    if (s >= MAX_SOCK_NUM) return;

    // Always lock on reading
    socket[s].myFlagLock = 1;
    
    socket[s].ptrRdRx = readSnRX_RD(s);
    SOCKETReadBuffer(data, len, socket[s].ptrRdRx);
    if (!peek)
    {
        socket[s].ptrRdRx += len;
        writeSnRX_RD(s, socket[s].ptrRdRx);
    }
}

// Get TX Free size
//
uint16_t ENC28Class::getTXFreeSize(SOCKET s)
{
    if (s >= MAX_SOCK_NUM) return 0;

    return (socket[s].endTx - socket[s].ptrWrTx);
}

// Get RX received buffer size
// As received size is calculated by RX pointer location,
// it will decreased while reading
//
uint16_t ENC28Class::getRXReceivedSize(SOCKET s)
{
    if (s >= MAX_SOCK_NUM) return 0;

    return socket[s].sizeRx;
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

    if (socket[_s].myFlagSendOk)
    { 
#ifdef DEBUG_REV1_HEAVY
        Serial.print("SEND_OK: ");
        Serial.println(millis());
#endif

        // Advancing TX pointer
        RecalcTxPointer(_s);

        // Rev1: Lock because
        // we must avoid ACK arriving
        // when not in sending time
        socket[_s].myFlagLock = 1;

        // Return flag expected
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
        socket[_s].myFlagSendOk = 0;
    }
}

// Read Rx pointer
//
uint16_t ENC28Class::readSnRX_RD(SOCKET _s)
{
    if (_s >= MAX_SOCK_NUM) return 0;

    uint16_t p = SOCKETGetRxPointer();
    return p;
}

// Read Tx pointer
//
uint16_t ENC28Class::readSnTX_WR(SOCKET _s)
{
    if (_s >= MAX_SOCK_NUM) return 0;

    uint16_t p = SOCKETGetTxPointer();
    return p;
}

// Write Rx pointer
//
void ENC28Class::writeSnRX_RD(SOCKET _s, uint16_t _addr)
{
    if (_s >= MAX_SOCK_NUM) return;

    SOCKETSetRxPointer(_addr);
}

// Write Tx Pointer
//
void ENC28Class::writeSnTX_WR(SOCKET _s, uint16_t _addr)
{
    if (_s >= MAX_SOCK_NUM) return;

    SOCKETSetTxPointer(_addr);
}

// Write Mode
//
void ENC28Class::writeSnMR(SOCKET _s, uint8_t _flag)
{
    if (_s >= MAX_SOCK_NUM) return;

#ifdef DEBUG_REV1
    Serial.print("MODE Flag (");
    Serial.print(_flag == SnMR::TCP ? "TCP" : "OTHER");
    Serial.println(") OK!");
#endif

#ifdef DEBUG_REV1_LIGHT
    Serial.print("MODE Flag (");
    Serial.print(_flag == SnMR::TCP ? "TCP" : "OTHER");
    Serial.println(") OK!");
#endif
    // 4.
    // Socket new instance
    // When socket define hardware mode
    socket[_s].status   = SnSR::CLOSED;
	
	// Rev1: TODO Must create Issue to adjust
	// Rev1: threading sockets.
	// Rev1: W5100 lib does not thread processes, but...
    
    // Pointers to buffers ends and starts
    if (_flag == SnMR::TCP)
    {

        // TCP protocol
        if (_s == 0)
        {
            socket[_s].startRx = SOCKET1_RX_START;
            socket[_s].endRx   = SOCKET1_RX_END;
            socket[_s].startTx = SOCKET1_TX_START;
            socket[_s].endTx   = SOCKET1_TX_END;
            socket[_s].ptrRdRx   = socket[_s].startRx;
            socket[_s].ptrWrTx   = socket[_s].startTx;
        }
        else if (_s == 1)
        {
            socket[_s].startRx = SOCKET2_RX_START;
            socket[_s].endRx   = SOCKET2_RX_END;
            socket[_s].startTx = SOCKET2_TX_START;
            socket[_s].endTx   = SOCKET2_TX_END;
            socket[_s].ptrRdRx   = socket[_s].startRx;
            socket[_s].ptrWrTx   = socket[_s].startTx;
        }
        else
        {
            // Error max packets reached
        }

        // Call Open Func
        MACOpen();

        // wait for stabilization
        delay(10);

        // Enabling reception                
        MACEnableRecv();

        // wait for stabilization
        delay(10);  
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

    serverSourcePort  = _addr;

#ifdef DEBUG_REV1
    Serial.print("OPEN Port (");
    Serial.print(serverSourcePort);
    Serial.println(") OK!");
#endif

#ifdef DEBUG_REV1_LIGHT
    Serial.print("OPEN Port (");
    Serial.print(serverSourcePort);
    Serial.println(") OK!");
#endif
}

// Get Ip address
//
void ENC28Class::getIPAddress(uint8_t *_addr)
{
    for (int i = 0; i < 4; i++)
    {
        _addr[i] = _my_ipaddr[i];
    }
}

// Get gateway address
//
void ENC28Class::getGatewayIp(uint8_t *_addr)
{
    for (int i = 0; i < 4; i++)
    {
        _addr[i] = _my_gataddr[i];
    }
}

// Get subnet mask address
//
void ENC28Class::getSubnetMask(uint8_t *_addr)
{
    for (int i = 0; i < 4; i++)
    {
        _addr[i] = _my_subaddr[i];
    }
}

// Get MAC address
//
void ENC28Class::getMACAddress(uint8_t *_addr)
{
    for (int i = 0; i < 6; i++)
    {
        _addr[i] = _my_macaddr[i];
    }
}

// Set MAC address
//
void ENC28Class::setMACAddress(uint8_t *_addr)
{
    // 2.
    // Ethernet begin
    for (int i = 0; i < 6; i++)
    {
        _my_macaddr[i] = _addr[i];
    }
    MACInitMacAddr((unsigned char*)&_my_macaddr);
}

// Set Ip address
//
void ENC28Class::setIPAddress(uint8_t *_addr)
{
    // 3.
    // Ethernet begin
    
    for (int i = 0; i < 4; i++)
    {
        _my_ipaddr[i] = _addr[i];
    }
}

// Set Gateway address
//
void ENC28Class::setGatewayIp(uint8_t *_addr)
{
    // 3.1.
    // Ethernet begin
    
    for (int i = 0; i < 4; i++)
    {
        _my_gataddr[i] = _addr[i];
    }
}

// Set Subnet mask address
//
void ENC28Class::setSubnetMask(uint8_t *_addr)
{
    // 3.2.
    // Ethernet begin
    
    for (int i = 0; i < 4; i++)
    {
        _my_subaddr[i] = _addr[i];
    }
}

// Not implemented
//
void  ENC28Class::writeSnDIPR(SOCKET _s, uint8_t* _addr)
{
    
    
}

// Not implemented
//
void ENC28Class::writeSnDPORT(SOCKET _s, uint16_t _addr)
{
    
}

// Not implemented
//
void ENC28Class::read_data(SOCKET s, volatile uint8_t *src, volatile uint8_t *dst, uint16_t len)
{
    
}

// Not implemented
//
void ENC28Class::setRetransmissionTime(uint16_t _timeout)
{
  // not implemented
}

// Not implemented
//
void ENC28Class::setRetransmissionCount(uint8_t _retry)
{
  // not implemented
}

/***************************************************************************
 * STATIC LOCAL FUNCTIONS
 ***************************************************************************/

// Read Packets from receive buffer
// Treats already ARP and ICMP
//
static void readSocketPackets(void)
{
    // Rev 1
    // can we discard packet?
    // during BUG's correction
    // this technique worked at some point
    // but I think it is not necessary anymore
    unsigned char canDiscard = 1;

    if (MACGetPacketCount() > 0) 
    {
        // Deal with incoming packets
        // Returns true if TCP packet arrived
        // and is my IP address
        if (DealRXArrival())
        {
            // If we got here, means:
            // a. IP destination address is our IP address
            // b. Destination Port is serverPort (normally 80)
            // So, we already have Dest IP and Port confirmed for us!

            // Cycling through sockets to accommodate
            // arrived packet
            for (unsigned int i = 0; i < MAX_SOCK_NUM; i++)
            {
                if (socket[i].myFlagLock)
                {
                    canDiscard = 0;
                    break;
                }
                else
                {
                    if (socket[i].status == SnSR::LISTEN)
                    {
#ifdef DEBUG_REV1_HEAVY
                        Serial.println("SnSR::LISTEN");
#endif
                        // REV 1 - Revision on 
                        // reading and backgrounding processes
                        // ... in the end, the sending process 
                        // was the real pain in back ...

                        // Session treatment
                        // Check for final ACK for SYN handshaking
                        if (tcpPacket2.tcp.SourcePort == socket[i].sessionPort)
                        {
                            if (tcpPacket2.tcp.Flags.Bits.FlagACK
                                && tcpPacket2.tcp.Flags.Bits.FlagPSH

                                && !tcpPacket2.tcp.Flags.Bits.FlagFIN
                                )
                            {
                                // Going established
                                socket[i].myFlagEstablished = 1;

                                // Copy packet from
                                // ENC28J60's RX buffer to socket RX buffer
                                DMACopyTo(RX, socket[i].startRx, tcpPacket2.ip.TotalLength.Value + ETH_BUFF_SIZE);
                                WaitForDMACopy();

                                // stop searching sockets to attend
                                // this request will be deal at background time
                                break;
                            }
                            else if (tcpPacket2.tcp.Flags.Bits.FlagFIN 
                                    && tcpPacket2.tcp.Flags.Bits.FlagACK )
                            {
                                    
                                // Setting flag to send final ACK
                                socket[i].myFlagSendFinalACK = 1;
#ifdef DEBUG_REV1_HEAVY
                                Serial.println("FIN OUT OF REASON - SESSION");
#endif
                                // Copy packet from
                                // ENC28J60's RX buffer to socket RX buffer
                                DMACopyTo(RX, socket[i].startRx, TCP_BUFF_SIZE);
                                WaitForDMACopy();

                                socket[i].seqNumForSending = GetRequestAckNum(i);
                                socket[i].ackNumForSending = GetRequestSeqNum(i);

                                // Bind session port
                                socket[i].sessionPort = tcpPacket2.tcp.SourcePort;

                                // Put socket in waiting for close status
                                // we have 1 second to close it!
                                // the W5100 socket has this config
                                // hard coded
                                socket[i].status = SnSR::CLOSE_WAIT;

                                // We need to lock
                                // because we sending 
                                // a group of packets
                                if (!socket[i].myFlagLock) socket[i].myFlagLock = 1;

                                // stop searching sockets to attend
                                // this request will be deal at background time
                                break;
                                        
                                    
                            }
                        }


                        // Free treatment
                        // Receive a SYN and bind source port
                        else 
                        {
                            // Checking if is really a new session
                            // or just the client trying to open
                            // lot of source ports!
                            //
                            if (socket[i].sessionPort == 0)
                            {
                                if (tcpPacket2.tcp.Flags.Bits.FlagSYN)
                                {
                                    // Ok, actual port is different from session,
                                    // sending SYN and making this our new session port
                                    socket[i].myFlagNewSession = 1;

                                    // Bind session port
                                    socket[i].sessionPort = tcpPacket2.tcp.SourcePort;

                                    // Copy packet from
                                    // ENC28J60's RX buffer to socket RX buffer
                                    // With options 'cause we send
                                    // MMS option along with SYN/ACK
                                    DMACopyTo(RX, socket[i].startRx, TCP_BUFF_SIZE_W_OPT);
                                    WaitForDMACopy();

                                    // stop searching sockets to attend
                                    // this request will be deal at background time
                                    break;
                                }
                                
                                
                                else 
                                {
                                    if (!tcpPacket2.tcp.Flags.Bits.FlagRST)
                                    {
                                        // Setting flag to send final ACK
                                        socket[i].myFlagSendFinalACK = 1;
#ifdef DEBUG_REV1_HEAVY
                                        Serial.println("FIN OUT OF REASON");
#endif
                                        // Copy packet from
                                        // ENC28J60's RX buffer to socket RX buffer
                                        DMACopyTo(RX, socket[i].startRx, TCP_BUFF_SIZE);
                                        WaitForDMACopy();

                                        socket[i].seqNumForSending = GetRequestAckNum(i);
                                        socket[i].ackNumForSending = GetRequestSeqNum(i);

                                        // Bind session port
                                        socket[i].sessionPort = tcpPacket2.tcp.SourcePort;

                                        // Put socket in waiting for close status
                                        // we have 1 second to close it!
                                        // the W5100 socket has this config
                                        // hard coded
                                        socket[i].status = SnSR::CLOSE_WAIT;

                                        // We need to lock
                                        // because we sending 
                                        // a group of packets
                                        if (!socket[i].myFlagLock) socket[i].myFlagLock = 1;

                                        // stop searching sockets to attend
                                        // this request will be deal at background time
                                        break;
                                    }
                                }
                                
                                
                            }
                        }
                        
                    }
                    else if (socket[i].status == SnSR::ESTABLISHED)
                    {


                        // Session treatment
                        // Receive a PSH for session port
                        // Also treat control ACK's for sent packets
                        if (tcpPacket2.tcp.SourcePort == socket[i].sessionPort)
                        {
                            if (tcpPacket2.tcp.Flags.Bits.FlagACK
                                && !tcpPacket2.tcp.Flags.Bits.FlagPSH)
                            {
                                // Copy packet from
                                // ENC28J60's RX buffer to socket RX buffer
                                DMACopyTo(RX, socket[i].startRx, TCP_BUFF_SIZE);
                                WaitForDMACopy();

                                // Rev 1
                                // Sync ACK only when seq number 
                                // is greather than previous one
                                if (GetRequestAckNum(i) > socket[i].seqNumForSending)
                                {
                                    socket[i].seqNumForSending = GetRequestAckNum(i);
                                    socket[i].ackNumForSending = GetRequestSeqNum(i);

#ifdef DEBUG_REV1_HEAVY
                                    Serial.print("SnSR::ESTABLISHED: ");
                                    Serial.println(millis());
                                    Serial.print("ACK Num FS: ");
                                    Serial.println(socket[i].ackNumForSending);
                                    Serial.print("SEQ Num FS: ");
                                    Serial.println(socket[i].seqNumForSending);
#endif

                                    // Set My Flag to respond
                                    socket[i].myFlagGotSyncAck = 1;

                                    // Set Send OK flag
                                    socket[i].myFlagSendOk = 1;

                                    // Turn off waiting 
                                    // for sync ack
                                    socket[i].myFlagWaitForSyncAck = 0;

                                    // Clear last payload
                                    socket[i].previousPayloadForSending = 0;
                                }

                                // stop searching sockets to attend
                                // this request already been treated
                                break;
                            }
                        }
                        // Free treatment
                        // N/A
                        
                    }
                    else if (socket[i].status == SnSR::CLOSE_WAIT)
                    {
#ifdef DEBUG_REV1_HEAVY
                        Serial.println("SnSR::CLOSE_WAIT");
#endif
                        // Session treatment
                        // check for last ACK at session port
                        if (tcpPacket2.tcp.SourcePort == socket[i].sessionPort)
                        {
                            if (tcpPacket2.tcp.Flags.Bits.FlagFIN
                                && tcpPacket2.tcp.Flags.Bits.FlagACK
                                )
                            {
                                // Copy packet from
                                // ENC28J60's RX buffer to socket RX buffer
                                DMACopyTo(RX, socket[i].startRx, TCP_BUFF_SIZE);
                                WaitForDMACopy();

                                // See Rev 1 - Bug 1
                                // Setting SEQ/ACK numbers
                                // 
                                // When we got an ACK we dont need
                                // to calculate SEQ/ACK numbers,
                                // we just retrieve it from DMA buffer!
                                //
                                socket[i].seqNumForSending = GetRequestAckNum(i);
                                socket[i].ackNumForSending = GetRequestSeqNum(i);

                                // Set My Flag to respond
                                socket[i].myFlagSendCloseACK = 1;

                                // stop searching sockets to attend
                                // this request already been treated
                                break;
                            }
                        }

                        // Free treatment
                        // N/A
                    }
                    else // CLOSED 
                    {
#ifdef DEBUG_REV1_HEAVY
                        if (socket[i].status != SnSR::CLOSED)
                        {
                            Serial.print("Status = ");
                            Serial.println(socket[i].status);
                        }
                        else
                        {
                            Serial.println("SnSR::CLOSED");
                        }
#endif

                        // Session treatment
                        // Send RST to indicate this session is over!
                        // cant reproduce this BUG... 
                        // TODO to another REV

                        // Free treatment
                        // N/A

                    }
                }
            }
        }
        
        // Release packet
        if (canDiscard) MACDiscardRx();
    }
}



// Background tasks
// Some things cant be accomplished at time
// because of nature of we are doing
//
static void treatBackgroundProcesses(void)
{
    // Cycling through sockets to execute
    // pending tasks
    for (unsigned int i = 0; i < MAX_SOCK_NUM; i++)
    {
        if (socket[i].myFlagNewSession)
        {
#ifdef DEBUG_REV1
            Serial.println("myFlagNewSession");
            Serial.print("New Port: ");
            Serial.println(GetSourcePort(i));
            Serial.print("Socket: ");
            Serial.println(i);
#endif
            // reset flag
            socket[i].myFlagNewSession = 0;

            // treating flag
            // Send response
            SendSynResponse(i);

            // stop searching sockets to attend
            // this request already been treated
            break;
        }
        else if (socket[i].myFlagEstablished)
        {
#ifdef DEBUG_REV1
            Serial.println("myFlagEstablished");
#endif
            // reset flag
            socket[i].myFlagEstablished = 0;

            // First things, first
            // Configuring reading buffer
            // before go established

            // Clear RX size
            socket[i].sizeRx = 0;

            // Get client's payload
            socket[i].requestPayload = GetRequestPayload(i);

            // Bug 1 - Rev 1 Revisited
            // That's ok to send an ACK now
            // We just need to keep track of
            // SEQ and ACK numbers.
            // Already inverting ACK/SEQ numbers
            socket[i].seqNumForSending = GetRequestAckNum(i);
            socket[i].ackNumForSending = (uint32_t)(GetRequestSeqNum(i) + socket[i].requestPayload);
            socket[i].previousPayloadForSending = 0;

            // And sending ACK with no payload
            SendAckResponse(i);

            // Put socket in stablished
            // status
            socket[i].status = SnSR::ESTABLISHED;
            
            // Set pointers positions
            socket[i].ptrRdRx = socket[i].startRx + TCP_BUFF_SIZE;
            // Calc. endRx pointer
            if ((socket[i].ptrRdRx + socket[i].requestPayload) <= SOCKET_RX_END(i))
                socket[i].endRx   = (socket[i].ptrRdRx + socket[i].requestPayload);
            else
                socket[i].endRx   = SOCKET_RX_END(i);
                
            // Configuring socket reader RX pointer
            SOCKETSetRxPointer(socket[i].ptrRdRx);
            
            // Recalc Tx Pointer
            RecalcTxPointer(i);
            
            // we must lock for reading
            socket[i].myFlagLock = 1;
            
            // Release the kraken!
            // Rev 1:
            // Appears to be redunctant
            // but sizeRx is decreased and
            // sometimes we need the original
            // request payload's size.
            socket[i].sizeRx = socket[i].requestPayload;
            
            // stop searching sockets to attend
            // this request already been treated
            break;
        }
        // Old: Granting with myFlagSendOk
        // Old: we mess up with send already gone OK! (????)
        //
        // Rev1:
        // Changed everything to only one flag DoSend 
        //
        else if (socket[i].myFlagDoSend) 
        {
            unsigned int payloadForSending = 0;

#ifdef DEBUG_REV1_HEAVY
            Serial.print("myFlagDo(ing)Send: ");
            Serial.println(millis());
#endif
            // Clear Flag
            socket[i].myFlagDoSend = 0;

            // Do Send
            // Calc size TX
            payloadForSending = socket[i].ptrWrTx - (socket[i].startTx + TCP_BUFF_SIZE + 1);

            // Preparing ACK/SEQ numbers
            if (socket[i].previousPayloadForSending > 0)
            {
                socket[i].seqNumForSending += socket[i].previousPayloadForSending;
            }
            socket[i].previousPayloadForSending = payloadForSending;

            if (payloadForSending > 0)
            {
                PrepareAckWithPayload(i, payloadForSending);
                SendAckWithPayload(i, payloadForSending);
            }

            // And do few more loops
            // before set myFlagFoundNoSyncAck

            // Setting wait for 
            // sync ack flag 
            socket[i].myFlagWaitForSyncAck = 1;
            
            // Clear counter
            socket[i].loopCounter = 0;

            // stop searching sockets to attend
            // this request already been treated
            break;
            
            
        }
        else if (socket[i].myFlagWaitForSyncAck)
        {
            // not now, do more loops 
            // before consider it done
            if (socket[i].loopCounter > 50)
            {
#ifdef DEBUG_REV1_HEAVY
                Serial.print("myFlagFoundNoSyncAck: ");
                Serial.println(millis());
#endif
                // reset flag
                socket[i].myFlagWaitForSyncAck = 0;

                // Clear counter
                socket[i].loopCounter = 0;

                // Setting flag telling
                // server to send more data.
                // This means we did N loops
                // and found no synch ACK
                socket[i].myFlagFoundNoSyncAck = 1;

                // Set Send OK flag
                socket[i].myFlagSendOk = 1;
            }

            // stop searching sockets to attend
            // this request already been treated
            break;
        }
        else if (socket[i].myFlagSendFinalACK)
        {
#ifdef DEBUG_REV1_HEAVY
            Serial.println("myFlagSendFinalACK");
#endif
            // reset flag
            socket[i].myFlagSendFinalACK = 0;

            // Just send an ACK
            SendAckResponse(i);

            // And schedule a flag for
            // sending FIN/ACK along 
            // with this last ACK
            socket[i].myFlagSendFinalFIN = 1;

            // We must hold the lock
            // We dont want packets
            // arriving while we sending last ACK
            // and just after we will send FIN/ACK
            if (!socket[i].myFlagLock) 
                socket[i].myFlagLock = 1;

            // stop searching sockets to attend
            // this request already been treated
            break;
        }
        else if (socket[i].myFlagSendFinalFIN)
        {
#ifdef DEBUG_REV1_HEAVY
            Serial.println("myFlagSendFinalFIN");
#endif
            // reset flag
            socket[i].myFlagSendFinalFIN = 0;

            // Just send the FIN
            SendFinRequest(i);

            // Then release the lock
            if (socket[i].myFlagLock) socket[i].myFlagLock = 0;

            // stop searching sockets to attend
            // this request already been treated
            break;
        }
        else if (socket[i].myFlagDiscon && socket[i].myFlagGotSyncAck) 
        {
#ifdef DEBUG_REV1
            Serial.println("myFlagDiscon");
#endif
            // Put socket in waiting for close status
            // we have 1 second to close it!
            // the W5100 socket has this config
            // hard coded
            socket[i].status = SnSR::CLOSE_WAIT;

            // reset the flag
            socket[i].myFlagDiscon = 0;

            // Setting flag to send final ACK
            socket[i].myFlagSendFinalACK = 1;

            // We need to lock
            // because we sending 
            // a group of packets
            if (!socket[i].myFlagLock) socket[i].myFlagLock = 1;

            // stop searching sockets to attend
            // this request already been treated
            break;
        }
        else if (socket[i].myFlagSendCloseACK)
        {
#ifdef DEBUG_REV1_HEAVY
            Serial.print("myFlagSendCloseACK: ");
            Serial.println(millis());
#endif
            // reset the flag
            socket[i].myFlagSendCloseACK = 0;

            // SEQ and ACK numbers are ok already,
            // but we need to add 1 to ACK number
            socket[i].ackNumForSending++;

            // Just send the final ACK
            SendAckResponse(i);

            // Close socket for good
            socket[i].status = SnSR::CLOSED;

            // Release the session!
            // By releasing the port!
            ClearSocket(i);

            // release the lock 
            // if its the case
            if (socket[i].myFlagLock) socket[i].myFlagLock = 0;

            // stop searching sockets to attend
            // this request already been treated
            break;
        }
    }
}


// First arrival treatment
// Returns true if is a TCP packet
// and match my IP address
//
static uint8_t DealRXArrival(void)
{
    // Read buffer backwards because of
    // big endian integers
    readBufferLittleEndian(socketBuffer, ETH_BUFF_SIZE);
    
    // Load byte array into struct
    ETH_HEADER ethPacket = (ETH_HEADER&)socketBuffer;
    
    // Check if is an ARP or ICMP packet
    if (ethPacket.Type.Bytes.ByteH == ETHTYPE_ARP_H_V
        && ethPacket.Type.Bytes.ByteL == ETHTYPE_ARP_L_V)
    {
        // Load ARP struct
        readBufferLittleEndian(socketBuffer, ARP_BUFF_SIZE);
        ARP_PACKET arpPacket = (ARP_PACKET&)socketBuffer;
        
        // Check IP from ARP packet
        if (arpPacket.arp.DestIp1 == _my_ipaddr[0]
            && arpPacket.arp.DestIp2 == _my_ipaddr[1]
            && arpPacket.arp.DestIp3 == _my_ipaddr[2]
            && arpPacket.arp.DestIp4 == _my_ipaddr[3])
        {
            // Ok, ARP packet is for me.
            // If so, respond the ARP packet,
            //     advance the reader pointer and clear RX flag
            
            // Make Eth header
            // Inverting MAC adresses
            arpPacket.eth.DestMac1 = arpPacket.eth.SrcMac1;
            arpPacket.eth.DestMac2 = arpPacket.eth.SrcMac2;
            arpPacket.eth.DestMac3 = arpPacket.eth.SrcMac3;
            arpPacket.eth.DestMac4 = arpPacket.eth.SrcMac4;
            arpPacket.eth.DestMac5 = arpPacket.eth.SrcMac5;
            arpPacket.eth.DestMac6 = arpPacket.eth.SrcMac6;
            
            arpPacket.eth.SrcMac1 = _my_macaddr[0];
            arpPacket.eth.SrcMac2 = _my_macaddr[1];
            arpPacket.eth.SrcMac3 = _my_macaddr[2];
            arpPacket.eth.SrcMac4 = _my_macaddr[3];
            arpPacket.eth.SrcMac5 = _my_macaddr[4];
            arpPacket.eth.SrcMac6 = _my_macaddr[5];
            
            // Make ARP header
            // Fill OPTCODE with response value
            arpPacket.arp.OperationCode = high(ETH_ARP_OPCODE_REPLY_H_V)
                                            | low(ETH_ARP_OPCODE_REPLY_L_V);
            
            // Inverting MAC adresses
            arpPacket.arp.DestMac1 = arpPacket.arp.SrcMac1;
            arpPacket.arp.DestMac2 = arpPacket.arp.SrcMac2;
            arpPacket.arp.DestMac3 = arpPacket.arp.SrcMac3;
            arpPacket.arp.DestMac4 = arpPacket.arp.SrcMac4;
            arpPacket.arp.DestMac5 = arpPacket.arp.SrcMac5;
            arpPacket.arp.DestMac6 = arpPacket.arp.SrcMac6;
            
            arpPacket.arp.SrcMac1 = _my_macaddr[0];
            arpPacket.arp.SrcMac2 = _my_macaddr[1];
            arpPacket.arp.SrcMac3 = _my_macaddr[2];
            arpPacket.arp.SrcMac4 = _my_macaddr[3];
            arpPacket.arp.SrcMac5 = _my_macaddr[4];
            arpPacket.arp.SrcMac6 = _my_macaddr[5];
            
            // Inverting IP adresses
            arpPacket.arp.DestIp1 = arpPacket.arp.SrcIp1;
            arpPacket.arp.DestIp2 = arpPacket.arp.SrcIp2;
            arpPacket.arp.DestIp3 = arpPacket.arp.SrcIp3;
            arpPacket.arp.DestIp4 = arpPacket.arp.SrcIp4;
            
            arpPacket.arp.SrcIp1 = _my_ipaddr[0];
            arpPacket.arp.SrcIp2 = _my_ipaddr[1];
            arpPacket.arp.SrcIp3 = _my_ipaddr[2];
            arpPacket.arp.SrcIp4 = _my_ipaddr[3];
            
            writeBufferBigEndian((unsigned char*)&arpPacket, ARP_BUFF_SIZE);
            
            // Commiting send
            SendBufferTx();
        }
    }
    else if (ethPacket.Type.Bytes.ByteH == ETHTYPE_IP_H_V
        && ethPacket.Type.Bytes.ByteL == ETHTYPE_IP_L_V)
    {
        // Load IP struct
        readBufferLittleEndian(socketBuffer, IP_BUFF_SIZE);
        IP_PACKET ipPacket = (IP_PACKET&)socketBuffer;
        
        // Check for IPV4 Len
        if (ipPacket.ip.Version.Value == IP_HEADER_LEN_VER_V)
        {
            // Check if it is an IP packet and if destination
            // port is our port
                    
            // Check if Packet is for us
            if (ipPacket.ip.DestIp1 == _my_ipaddr[0]
                && ipPacket.ip.DestIp2 == _my_ipaddr[1]
                && ipPacket.ip.DestIp3 == _my_ipaddr[2]
                && ipPacket.ip.DestIp4 == _my_ipaddr[3])
            {
                // Check if is ICMP...
                if (ipPacket.ip.Protocol.Value == IP_PROTO_ICMP_V)
                {
                    // Load ICMP struct
                    readBufferLittleEndian(socketBuffer, ICMP_BUFF_SIZE);
                    ICMP_PACKET icmpPacket = (ICMP_PACKET&)socketBuffer;
                    
                    if (icmpPacket.icmp.Type == ICMP_TYPE_ECHOREQUEST_V)
                    {
                        //  If so, respond the ICMP packet,
                        //  advance the reader pointer and clear RX flag
                        
                        // Make Eth header
                        // Inverting MAC adresses
                        icmpPacket.eth.DestMac1 = icmpPacket.eth.SrcMac1;
                        icmpPacket.eth.DestMac2 = icmpPacket.eth.SrcMac2;
                        icmpPacket.eth.DestMac3 = icmpPacket.eth.SrcMac3;
                        icmpPacket.eth.DestMac4 = icmpPacket.eth.SrcMac4;
                        icmpPacket.eth.DestMac5 = icmpPacket.eth.SrcMac5;
                        icmpPacket.eth.DestMac6 = icmpPacket.eth.SrcMac6;
                        
                        icmpPacket.eth.SrcMac1 = _my_macaddr[0];
                        icmpPacket.eth.SrcMac2 = _my_macaddr[1];
                        icmpPacket.eth.SrcMac3 = _my_macaddr[2];
                        icmpPacket.eth.SrcMac4 = _my_macaddr[3];
                        icmpPacket.eth.SrcMac5 = _my_macaddr[4];
                        icmpPacket.eth.SrcMac6 = _my_macaddr[5];
                        
                        // Inverting IP adresses
                        icmpPacket.ip.DestIp1 = icmpPacket.ip.SrcIp1;
                        icmpPacket.ip.DestIp2 = icmpPacket.ip.SrcIp2;
                        icmpPacket.ip.DestIp3 = icmpPacket.ip.SrcIp3;
                        icmpPacket.ip.DestIp4 = icmpPacket.ip.SrcIp4;
                        
                        icmpPacket.ip.SrcIp1 = _my_ipaddr[0];
                        icmpPacket.ip.SrcIp2 = _my_ipaddr[1];
                        icmpPacket.ip.SrcIp3 = _my_ipaddr[2];
                        icmpPacket.ip.SrcIp4 = _my_ipaddr[3];
                        
                        // clear the 2 byte checksum
                        icmpPacket.ip.Checksum.Bytes.ByteH = 0;
                        icmpPacket.ip.Checksum.Bytes.ByteL = 0;
                        icmpPacket.ip.FragmentFlags.Bytes.ByteH = 0x40;
                        icmpPacket.ip.FragmentFlags.Bytes.ByteL = 0x00;
                        icmpPacket.ip.TimeToLive.Value = 64;
                        
                        
                        // calculate IP Header the checksum:
                        unsigned char *tmp = (unsigned char*)&icmpPacket;
                        unsigned char tmpHeader[IP_HEADER_LEN_V];
                        unsigned int counter1 = 0;
                        for(int k = ICMP_BUFF_SIZE - ETH_BUFF_SIZE - 1;
                            k > ICMP_BUFF_SIZE - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
                        {
                            tmpHeader[counter1] = tmp[k];
                            counter1++;
                        }
                        icmpPacket.ip.Checksum.Value = meu_checksum(tmpHeader, IP_HEADER_LEN_V);
                        
                        // updating type
                        icmpPacket.icmp.Type = ICMP_TYPE_ECHOREPLY_V;
                        
                        // Handling ICMP checksum
                        if (icmpPacket.icmp.Checksum.Bytes.ByteH > 255 - ICMP_TYPE_ECHOREQUEST_V)
                        {
                            icmpPacket.icmp.Checksum.Value++;
                        }
                        icmpPacket.icmp.Checksum.Bytes.ByteH += ICMP_TYPE_ECHOREQUEST_V;
                                                    
                        // Ok to send!
                        writeBufferBigEndian((unsigned char*)&icmpPacket, ICMP_BUFF_SIZE);
                        
                        // Commiting send
                        SendBufferTx();
                    }
                }
                else if (ipPacket.ip.Protocol.Value == IP_PROTO_TCP_V)
                {
                    // It is a Tcp packet,
                    // check if is for my ip
                    if (ipPacket.ip.DestIp1 == _my_ipaddr[0]
                        && ipPacket.ip.DestIp2 == _my_ipaddr[1]
                        && ipPacket.ip.DestIp3 == _my_ipaddr[2]
                        && ipPacket.ip.DestIp4 == _my_ipaddr[3]
                    )
                    {
                        // Rev 1 - Bug No Server Port
                        //
                        // Load TCP struct With Options
                        readBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE);
                        tcpPacket2 = (TCP_PACKET&)socketBuffer;
                        // Checking if port checks!
                        if (tcpPacket2.tcp.DestPort == serverSourcePort)
                        {
                            // so return true
                            return 1;
                        }
                        else
                        {
                            // Port not open for listening
#ifdef DEBUG_REV1
                            Serial.print("Port (");
                            Serial.print(tcpPacket2.tcp.DestPort);
                            Serial.println(") not open for listening!");
                            Serial.print("Try: ");
                            Serial.println(serverSourcePort);
#endif
                        }
                    }
                    else
                    {
#ifdef DEBUG_REV1
                        Serial.print("IP (");
                        Serial.print(ipPacket.ip.DestIp1);
                        Serial.print(".");
                        Serial.print(ipPacket.ip.DestIp2);
                        Serial.print(".");
                        Serial.print(ipPacket.ip.DestIp3);
                        Serial.print(".");
                        Serial.print(ipPacket.ip.DestIp4);
                        Serial.println(") not configured for this interface!");
#endif
                    }
                }
                else
                {
                    // Protocol not implemented
#ifdef DEBUG_REV1_HEAVY
                    Serial.print("Protocol not implemented!");
#endif
                }
            }
        }
    }
    
    // If we got here,
    // packet beeing discarded
    // nothing else to do, return false
    return 0;
}


// Send SYN/ACK response for SYN
//
static void SendSynResponse(SOCKET i)
{
    // Load TCP struct With Options
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE_W_OPT, socket[i].startRx);
    TCP_PACKET_W_OPT tcpPacket = (TCP_PACKET_W_OPT&)socketBuffer;

    // Send SYNACK back to client
    // Make Eth header
    // Inverting MAC adresses
    tcpPacket.eth.DestMac1 = tcpPacket.eth.SrcMac1;
    tcpPacket.eth.DestMac2 = tcpPacket.eth.SrcMac2;
    tcpPacket.eth.DestMac3 = tcpPacket.eth.SrcMac3;
    tcpPacket.eth.DestMac4 = tcpPacket.eth.SrcMac4;
    tcpPacket.eth.DestMac5 = tcpPacket.eth.SrcMac5;
    tcpPacket.eth.DestMac6 = tcpPacket.eth.SrcMac6;
    
    tcpPacket.eth.SrcMac1 = _my_macaddr[0];
    tcpPacket.eth.SrcMac2 = _my_macaddr[1];
    tcpPacket.eth.SrcMac3 = _my_macaddr[2];
    tcpPacket.eth.SrcMac4 = _my_macaddr[3];
    tcpPacket.eth.SrcMac5 = _my_macaddr[4];
    tcpPacket.eth.SrcMac6 = _my_macaddr[5];
    
    // Ip Total Length
    tcpPacket.ip.TotalLength.Value = IP_HEADER_LEN_V + TCP_HEADER_LEN_PLAIN_V + TCP_OPT_LEN_V;
    
    // Inverting IP adresses
    tcpPacket.ip.DestIp1 = tcpPacket.ip.SrcIp1;
    tcpPacket.ip.DestIp2 = tcpPacket.ip.SrcIp2;
    tcpPacket.ip.DestIp3 = tcpPacket.ip.SrcIp3;
    tcpPacket.ip.DestIp4 = tcpPacket.ip.SrcIp4;
    
    tcpPacket.ip.SrcIp1 = _my_ipaddr[0];
    tcpPacket.ip.SrcIp2 = _my_ipaddr[1];
    tcpPacket.ip.SrcIp3 = _my_ipaddr[2];
    tcpPacket.ip.SrcIp4 = _my_ipaddr[3];
    
    // id num
    tcpPacket.ip.Identification.Value = socket[i].idNum++;
    
    // clear the 2 byte Ip checksum
    tcpPacket.ip.Checksum.Bytes.ByteH = 0;
    tcpPacket.ip.Checksum.Bytes.ByteL = 0;
    
    // calculate IP Header the checksum:
    unsigned char *tmp = (unsigned char*)&tcpPacket;
    unsigned char tmpHeader[IP_HEADER_LEN_V];
    unsigned int counter1 = 0;
    for(int k = TCP_BUFF_SIZE_W_OPT - ETH_BUFF_SIZE - 1;
        k > TCP_BUFF_SIZE_W_OPT - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
    {
        tmpHeader[counter1] = tmp[k];
        counter1++;
    }
    tcpPacket.ip.Checksum.Value = meu_checksum(tmpHeader, IP_HEADER_LEN_V);
    
    // Clear flags
    tcpPacket.tcp.Flags.Value = 0;
    // Set SYNACK flag
    tcpPacket.tcp.Flags.Bits.FlagACK = 1;
    tcpPacket.tcp.Flags.Bits.FlagSYN = 1;
    
    // Inverting ports
    unsigned int tmpDest = tcpPacket.tcp.DestPort;
    tcpPacket.tcp.DestPort   = tcpPacket.tcp.SourcePort;
    tcpPacket.tcp.SourcePort = tmpDest;
    
    // Inverting seq and ack numbers
    // and adding 1 for computing ACK flag
    unsigned long tmpSeq = tcpPacket.tcp.SeqNum + 1;
    if (tcpPacket.tcp.AckNum != 0)
        tcpPacket.tcp.SeqNum = tcpPacket.tcp.AckNum;
    else
        tcpPacket.tcp.SeqNum = 3472621397ul;
    tcpPacket.tcp.AckNum = tmpSeq;
    
    // Adding MMS Option
    tcpPacket.opt.Kind = 2;
    tcpPacket.opt.Length = 4;
    tcpPacket.opt.MaxSegSize = 0x580;
    
    // DataOffset = 5 for ACK without options
    //tcpPacket.tcp.DataOffset.Bits.NibbleH = 5;
    // DataOffset = 6 for SYNACK with options
    tcpPacket.tcp.DataOffset.Bits.NibbleH = 6;
    
    // clear the 2 byte Tcp checksum
    tcpPacket.tcp.Checksum.Bytes.ByteH = 0;
    tcpPacket.tcp.Checksum.Bytes.ByteL = 0;
    
    // calculate TCP Header checksum
    tmp = (unsigned char*)&tcpPacket;
    unsigned char tmpTcpHeader[TCP_HEADER_LEN_W_OPT_V + TCP_CHECKSUM_LEN_V];
    counter1 = 0;
    for(int k = (TCP_BUFF_SIZE_W_OPT - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V) - 1);
        k > ((TCP_BUFF_SIZE_W_OPT - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V) - 1) -
                            (TCP_HEADER_LEN_W_OPT_V + TCP_CHECKSUM_LEN_V)); k--)
    {
        tmpTcpHeader[counter1++] = tmp[k];
    }
    tcpPacket.tcp.Checksum.Value =
                checksum(tmpTcpHeader, TCP_HEADER_LEN_W_OPT_V + TCP_CHECKSUM_LEN_V, CHECKSUMTCPID);

    // Writing send buffer!
    writeBufferBigEndian((unsigned char*)&tcpPacket, TCP_BUFF_SIZE_W_OPT);
    
    // Commiting send
    SendBufferTx();
}

// New func - See Bug 2 - Rev 1
// Prepare ACK with payload to send it through socket
//
static void PrepareAckWithPayload(SOCKET i, unsigned int _payload)
{
	// Load TCP struct
	readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, socket[i].startRx);
	tcpPacket2 = (TCP_PACKET&)socketBuffer;
	
	// ETH Part
	tcpPacket2.eth.DestMac1 = tcpPacket2.eth.SrcMac1;
	tcpPacket2.eth.DestMac2 = tcpPacket2.eth.SrcMac2;
	tcpPacket2.eth.DestMac3 = tcpPacket2.eth.SrcMac3;
	tcpPacket2.eth.DestMac4 = tcpPacket2.eth.SrcMac4;
	tcpPacket2.eth.DestMac5 = tcpPacket2.eth.SrcMac5;
	tcpPacket2.eth.DestMac6 = tcpPacket2.eth.SrcMac6;
	tcpPacket2.eth.SrcMac1 = _my_macaddr[0];
	tcpPacket2.eth.SrcMac2 = _my_macaddr[1];
	tcpPacket2.eth.SrcMac3 = _my_macaddr[2];
	tcpPacket2.eth.SrcMac4 = _my_macaddr[3];
	tcpPacket2.eth.SrcMac5 = _my_macaddr[4];
	tcpPacket2.eth.SrcMac6 = _my_macaddr[5];
	tcpPacket2.eth.Type.Value = 0x800;
	
    // See Rev 1 - Bug 1
	//unsigned int tmpTotalLen = tcpPacket2.ip.TotalLength.Value;
	
	// IP Part
	tcpPacket2.ip.Version.Value = 0x45;
	tcpPacket2.ip.DscpEcn = 0;
	tcpPacket2.ip.TotalLength.Value = TOTAL_HEADER_SIZE + _payload;
	tcpPacket2.ip.Identification.Value = socket[i].idNum++;
	tcpPacket2.ip.FragmentFlags.Bytes.ByteH = 0x40;
	tcpPacket2.ip.FragmentFlags.Bytes.ByteL = 0x00;
	tcpPacket2.ip.TimeToLive.Value = 64;
	tcpPacket2.ip.Protocol.Value = 0x6;
	tcpPacket2.ip.Checksum.Value = 0;
	tcpPacket2.ip.DestIp1 = tcpPacket2.ip.SrcIp1;
	tcpPacket2.ip.DestIp2 = tcpPacket2.ip.SrcIp2;
	tcpPacket2.ip.DestIp3 = tcpPacket2.ip.SrcIp3;
	tcpPacket2.ip.DestIp4 = tcpPacket2.ip.SrcIp4;
	tcpPacket2.ip.SrcIp1 = _my_ipaddr[0];
	tcpPacket2.ip.SrcIp2 = _my_ipaddr[1];
	tcpPacket2.ip.SrcIp3 = _my_ipaddr[2];
	tcpPacket2.ip.SrcIp4 = _my_ipaddr[3];
	
	// Inverting ports
	unsigned int tmpDest = tcpPacket2.tcp.DestPort;
	tcpPacket2.tcp.DestPort   = tcpPacket2.tcp.SourcePort;
	tcpPacket2.tcp.SourcePort = tmpDest;
	
    // See Rev 1 - Bug 1
    // SEQ/ACK numbers are treated 
    // outside the function
    tcpPacket2.tcp.SeqNum = socket[i].seqNumForSending;
    tcpPacket2.tcp.AckNum = socket[i].ackNumForSending;

	tcpPacket2.tcp.DataOffset.Bits.NibbleH = 0x5;
	tcpPacket2.tcp.Flags.Value = 0x0;
	tcpPacket2.tcp.Window = 0xFFFF;
	tcpPacket2.tcp.Checksum.Value = 0;
	tcpPacket2.tcp.UrgentPoint = 0;
	
	// Resolving flags and checksums
	tcpPacket2.tcp.Flags.Bits.FlagACK = 1;
    // See Rev 1 - Bug 2 (Issues)
	//tcpPacket2.tcp.Flags.Bits.FlagPSH = 1;
	
	// calculate IP Header the checksum:
	unsigned char *tmpInverter = (unsigned char*)&tcpPacket2;
	unsigned char tmpHeader[IP_HEADER_LEN_V];
	unsigned int counter2 = 0;
	for(int k = TCP_BUFF_SIZE - ETH_BUFF_SIZE - 1;
	k > TCP_BUFF_SIZE - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
	{
		tmpHeader[counter2] = tmpInverter[k];
		counter2++;
	}
	tcpPacket2.ip.Checksum.Value = meu_checksum(tmpHeader, IP_HEADER_LEN_V);
	
	// Getting data to send from socket's TX buffer
	unsigned char buffWriter[_payload];
	SOCKETReadBuffer((unsigned char*)buffWriter, _payload, socket[i].startTx + TCP_BUFF_SIZE + 1);
	
	// calculate TCP Header the checksum:
	unsigned char *tmpTemp = (unsigned char*)&tcpPacket2;
	unsigned char tmp[TCP_BUFF_SIZE + _payload];
	for (unsigned int i = 0; i < _payload; i++)
	{
		tmp[i] = buffWriter[(_payload - 1) - i];
	}
	for (unsigned int i = 0; i < TCP_BUFF_SIZE; i++)
	{
		tmp[(_payload + i)] = tmpTemp[i]; //(TCP_BUFF_SIZE - 1) - i];
	}
	unsigned char tmpTcpHeader[TCP_HEADER_LEN_PLAIN_V + TCP_CHECKSUM_LEN_V + _payload];
	counter2 = 0;
	for(int k = (((TCP_BUFF_SIZE) - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V) - 1) + _payload);
	k >= (((TCP_BUFF_SIZE + _payload) - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V))
	- (TCP_CHECKSUM_LEN_V + TCP_HEADER_LEN_PLAIN_V + _payload)); k--)
	{
		tmpTcpHeader[counter2] = tmp[k];
		counter2++;
		if (!k) break;
	}
	tcpPacket2.tcp.Checksum.Value = checksum(tmpTcpHeader,
	TCP_HEADER_LEN_PLAIN_V + TCP_CHECKSUM_LEN_V + _payload, CHECKSUMTCPID);
	
	// Write TCP Header to socket Tx Buffer
	writeSocketBufferBigEndian((unsigned char*)&tcpPacket2, TCP_BUFF_SIZE, socket[i].startTx);
}

// New func - See Bug 2 - Rev 1
// Send PSH data though socket
//
static void SendAckWithPayload(SOCKET i, unsigned int _payload)
{
	// DMA Copy from socket buffer to ENC28J60 TX buffer
	// and send it!
	// Header part
	DMACopyFrom(TX, socket[i].startTx, TCP_BUFF_SIZE);
	
	// Wait for copy
	WaitForDMACopy();
	
	// Data Part
	unsigned char buffWriter[_payload];
	SOCKETReadBuffer((unsigned char*)buffWriter, _payload, socket[i].startTx + TCP_BUFF_SIZE + 1);
	
	// Write data part
	MACWriteTXBufferOffset((unsigned char*)&buffWriter, _payload, TCP_BUFF_SIZE);
	
	// Commiting send
	SendBufferTx();
}

// Send ACK in response for PSH or FIN
//
//static uint16_t SendAckResponse(SOCKET i)
// See Bug 1 Rev 1
static void SendAckResponse(SOCKET i)
{
    // Load TCP struct 
    //readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, socket[i].startRx);
    //tcpPacket2 = (TCP_PACKET&)socketBuffer;
    //unsigned int ret_payload = tcpPacket2.ip.TotalLength.Value - TOTAL_HEADER_SIZE;
    // Bug 1 Solve part 1
    // deprecated -- see GetRequestPayload(SOCKET i)
    
    //unsigned int ret_payload = GetRequestPayload(i);
    
    // ETH Part
    tcpPacket2.eth.DestMac1 = tcpPacket2.eth.SrcMac1;
    tcpPacket2.eth.DestMac2 = tcpPacket2.eth.SrcMac2;
    tcpPacket2.eth.DestMac3 = tcpPacket2.eth.SrcMac3;
    tcpPacket2.eth.DestMac4 = tcpPacket2.eth.SrcMac4;
    tcpPacket2.eth.DestMac5 = tcpPacket2.eth.SrcMac5;
    tcpPacket2.eth.DestMac6 = tcpPacket2.eth.SrcMac6;
    tcpPacket2.eth.SrcMac1 = _my_macaddr[0];
    tcpPacket2.eth.SrcMac2 = _my_macaddr[1];
    tcpPacket2.eth.SrcMac3 = _my_macaddr[2];
    tcpPacket2.eth.SrcMac4 = _my_macaddr[3];
    tcpPacket2.eth.SrcMac5 = _my_macaddr[4];
    tcpPacket2.eth.SrcMac6 = _my_macaddr[5];
    tcpPacket2.eth.Type.Value = 0x800;
    
    // IP Part
    tcpPacket2.ip.Version.Value = 0x45;
    tcpPacket2.ip.DscpEcn = 0;
    tcpPacket2.ip.TotalLength.Value = TOTAL_HEADER_SIZE; // + ret_payload;
    tcpPacket2.ip.Identification.Value = socket[i].idNum++;
    tcpPacket2.ip.FragmentFlags.Bytes.ByteH = 0x40;
    tcpPacket2.ip.FragmentFlags.Bytes.ByteL = 0x00;
    tcpPacket2.ip.TimeToLive.Value = 64;
    tcpPacket2.ip.Protocol.Value = 0x6;
    tcpPacket2.ip.Checksum.Value = 0;
    tcpPacket2.ip.DestIp1 = tcpPacket2.ip.SrcIp1;
    tcpPacket2.ip.DestIp2 = tcpPacket2.ip.SrcIp2;
    tcpPacket2.ip.DestIp3 = tcpPacket2.ip.SrcIp3;
    tcpPacket2.ip.DestIp4 = tcpPacket2.ip.SrcIp4;
    tcpPacket2.ip.SrcIp1 = _my_ipaddr[0];
    tcpPacket2.ip.SrcIp2 = _my_ipaddr[1];
    tcpPacket2.ip.SrcIp3 = _my_ipaddr[2];
    tcpPacket2.ip.SrcIp4 = _my_ipaddr[3];
    
    // Inverting ports
    unsigned int tmpDest = tcpPacket2.tcp.DestPort;
    tcpPacket2.tcp.DestPort   = tcpPacket2.tcp.SourcePort;
    tcpPacket2.tcp.SourcePort = tmpDest;
    
    // See Rev 1 - Bug 1
    // Getting ACK/SEQ numbers from
    // sockets array
    tcpPacket2.tcp.SeqNum = socket[i].seqNumForSending;
    tcpPacket2.tcp.AckNum = socket[i].ackNumForSending;

    tcpPacket2.tcp.DataOffset.Bits.NibbleH = 0x5;
    tcpPacket2.tcp.Flags.Value = 0x0;
    tcpPacket2.tcp.Window = 0xFFFF;
    tcpPacket2.tcp.Checksum.Value = 0;
    tcpPacket2.tcp.UrgentPoint = 0;
    
    // Resolving flags and checksums
    tcpPacket2.tcp.Flags.Bits.FlagACK = 1;
    
    // calculate IP Header the checksum:
    unsigned char *tmpInverter = (unsigned char*)&tcpPacket2;
    unsigned char tmpHeader[IP_HEADER_LEN_V];
    unsigned int counter2 = 0;
    for(int k = TCP_BUFF_SIZE - ETH_BUFF_SIZE - 1;
        k > TCP_BUFF_SIZE - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
    {
        tmpHeader[counter2] = tmpInverter[k];
        counter2++;
    }
    tcpPacket2.ip.Checksum.Value = meu_checksum(tmpHeader, IP_HEADER_LEN_V);
    
    // calculate TCP Header the checksum:
    unsigned char *tmpTemp = (unsigned char*)&tcpPacket2;
    unsigned char tmpTcpHeader[TCP_HEADER_LEN_PLAIN_V + TCP_CHECKSUM_LEN_V];
    counter2 = 0;
    for(int k = (((TCP_BUFF_SIZE) - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V) - 1));
        k >= (((TCP_BUFF_SIZE) - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V))
                    - (TCP_CHECKSUM_LEN_V + TCP_HEADER_LEN_PLAIN_V)); k--)
    {
        tmpTcpHeader[counter2] = tmpTemp[k];
        counter2++;
        if (!k) break;
    }
    tcpPacket2.tcp.Checksum.Value = checksum(tmpTcpHeader,
                TCP_HEADER_LEN_PLAIN_V + TCP_CHECKSUM_LEN_V, CHECKSUMTCPID);
    
    // Put headers into TX buffer
    writeBufferBigEndian((unsigned char*)&tcpPacket2, TCP_BUFF_SIZE);
    
    // Commiting send
    SendBufferTx();
    
    // See Bug 1 Rev1
    //return ret_payload;
}

// Send Fin request
//
static void SendFinRequest(SOCKET i)
{
    // Load TCP struct 
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, socket[i].startRx);
    tcpPacket2 = (TCP_PACKET&)socketBuffer;
    
    // ETH Part
    tcpPacket2.eth.DestMac1 = tcpPacket2.eth.SrcMac1;
    tcpPacket2.eth.DestMac2 = tcpPacket2.eth.SrcMac2;
    tcpPacket2.eth.DestMac3 = tcpPacket2.eth.SrcMac3;
    tcpPacket2.eth.DestMac4 = tcpPacket2.eth.SrcMac4;
    tcpPacket2.eth.DestMac5 = tcpPacket2.eth.SrcMac5;
    tcpPacket2.eth.DestMac6 = tcpPacket2.eth.SrcMac6;
    tcpPacket2.eth.SrcMac1 = _my_macaddr[0];
    tcpPacket2.eth.SrcMac2 = _my_macaddr[1];
    tcpPacket2.eth.SrcMac3 = _my_macaddr[2];
    tcpPacket2.eth.SrcMac4 = _my_macaddr[3];
    tcpPacket2.eth.SrcMac5 = _my_macaddr[4];
    tcpPacket2.eth.SrcMac6 = _my_macaddr[5];
    tcpPacket2.eth.Type.Value = 0x800;
    
    // IP Part
    tcpPacket2.ip.Version.Value = 0x45;
    tcpPacket2.ip.DscpEcn = 0;
    tcpPacket2.ip.TotalLength.Value = TOTAL_HEADER_SIZE;
    tcpPacket2.ip.Identification.Value = socket[i].idNum++;
    tcpPacket2.ip.FragmentFlags.Bytes.ByteH = 0x40;
    tcpPacket2.ip.FragmentFlags.Bytes.ByteL = 0x00;
    tcpPacket2.ip.TimeToLive.Value = 64;
    tcpPacket2.ip.Protocol.Value = 0x6;
    tcpPacket2.ip.Checksum.Value = 0;
    tcpPacket2.ip.DestIp1 = tcpPacket2.ip.SrcIp1;
    tcpPacket2.ip.DestIp2 = tcpPacket2.ip.SrcIp2;
    tcpPacket2.ip.DestIp3 = tcpPacket2.ip.SrcIp3;
    tcpPacket2.ip.DestIp4 = tcpPacket2.ip.SrcIp4;
    tcpPacket2.ip.SrcIp1 = _my_ipaddr[0];
    tcpPacket2.ip.SrcIp2 = _my_ipaddr[1];
    tcpPacket2.ip.SrcIp3 = _my_ipaddr[2];
    tcpPacket2.ip.SrcIp4 = _my_ipaddr[3];
    
    // Inverting ports
    unsigned int tmpDest = tcpPacket2.tcp.DestPort;
    tcpPacket2.tcp.DestPort   = tcpPacket2.tcp.SourcePort;
    tcpPacket2.tcp.SourcePort = tmpDest;
    
    // Rev 1
    // ACK/SEQ Num
    tcpPacket2.tcp.SeqNum = socket[i].seqNumForSending;
    tcpPacket2.tcp.AckNum = socket[i].ackNumForSending;

    tcpPacket2.tcp.DataOffset.Bits.NibbleH = 0x5;
    tcpPacket2.tcp.Flags.Value = 0x0;
    tcpPacket2.tcp.Window = 0xFFFF;
    tcpPacket2.tcp.Checksum.Value = 0;
    tcpPacket2.tcp.UrgentPoint = 0;
    
    // Resolving flags and checksums
    tcpPacket2.tcp.Flags.Bits.FlagACK = 1;
    tcpPacket2.tcp.Flags.Bits.FlagFIN = 1;
    
    // calculate IP Header the checksum:
    unsigned char *tmpInverter = (unsigned char*)&tcpPacket2;
    unsigned char tmpHeader[IP_HEADER_LEN_V];
    unsigned int counter2 = 0;
    for(int k = TCP_BUFF_SIZE - ETH_BUFF_SIZE - 1;
        k > TCP_BUFF_SIZE - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
    {
        tmpHeader[counter2] = tmpInverter[k];
        counter2++;
    }
    tcpPacket2.ip.Checksum.Value = meu_checksum(tmpHeader, IP_HEADER_LEN_V);
    
    // calculate TCP Header the checksum:
    unsigned char *tmpTemp = (unsigned char*)&tcpPacket2;
    unsigned char tmpTcpHeader[TCP_HEADER_LEN_PLAIN_V + TCP_CHECKSUM_LEN_V];
    counter2 = 0;
    for(int k = (((TCP_BUFF_SIZE) - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V) - 1));
        k >= (((TCP_BUFF_SIZE) - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V))
                    - (TCP_CHECKSUM_LEN_V + TCP_HEADER_LEN_PLAIN_V)); k--)
    {
        tmpTcpHeader[counter2] = tmpTemp[k];
        counter2++;
        if (!k) break;
    }
    tcpPacket2.tcp.Checksum.Value = checksum(tmpTcpHeader,
                TCP_HEADER_LEN_PLAIN_V + TCP_CHECKSUM_LEN_V, CHECKSUMTCPID);
    
    // Put headers into TX buffer
    writeBufferBigEndian((unsigned char*)&tcpPacket2, TCP_BUFF_SIZE);
    
    // Commiting send
    SendBufferTx();
}


// Get Payload from Socket Buffer
//
static uint16_t GetRequestPayload(SOCKET i)
{
    // Load TCP struct 
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, socket[i].startRx);
    tcpPacket2 = (TCP_PACKET&)socketBuffer;
    
    unsigned int ret_payload = tcpPacket2.ip.TotalLength.Value - TOTAL_HEADER_SIZE;
    
    return ret_payload;
}

// Get Source Port from Socket Buffer
//
static uint16_t GetSourcePort(SOCKET i)
{
    // Load TCP struct 
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, socket[i].startRx);
    tcpPacket2 = (TCP_PACKET&)socketBuffer;
    
    return tcpPacket2.tcp.SourcePort;
}

// See Rev 1 - Bug 1
// Get SEQ number from Socket Buffer
//
static uint32_t GetRequestSeqNum(SOCKET i)
{
    // Load TCP struct 
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, socket[i].startRx);
    tcpPacket2 = (TCP_PACKET&)socketBuffer;
    
    unsigned long ret = (unsigned long)(tcpPacket2.tcp.SeqNum);
    return ret;
}

// See Rev 1 - Bug 1
// Get ACK number from Socket Buffer
//
static uint32_t GetRequestAckNum(SOCKET i)
{
    // Load TCP struct 
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, socket[i].startRx);
    tcpPacket2 = (TCP_PACKET&)socketBuffer;
    
    unsigned long ret = (unsigned long)(tcpPacket2.tcp.AckNum);
    return ret;
}

// Wait for DMA Copy finish
//
static void WaitForDMACopy(void)
{
    while(!IsDMACopyDone());
}

// Send Buffer and waits for response
//
static void SendBufferTx(void)
{
    MACSendTx();

    unsigned long timerSendTrigger = millis();
    
    while(!IsMACSendTx())
    {
        if (timerSendTrigger + 100 < millis())
        {
            break;
        }
    }
}




// Put Tx pointer 54 positions forward
// to avoid TCP Head area
//
static void RecalcTxPointer(SOCKET i)
{
    // After confirming sending, recalc tx pointer
    // Set TX pointer position
    socket[i].ptrWrTx = socket[i].startTx + TCP_BUFF_SIZE + 1;
    
    // Configuring socket writer TX pointer
    SOCKETSetTxPointer(socket[i].ptrWrTx);
}

// Init SocketVal struct
//
static void InitSocket(SOCKET i)
{
    socket[i].ptrRdRx   = 0;
    socket[i].ptrWrTx   = 0;
    
    socket[i].startRx   = 0;
    socket[i].endRx     = 0;
    socket[i].startTx   = 0;
    socket[i].endTx     = 0;
    
    socket[i].idNum     = 322;
    
    socket[i].status = SnSR::CLOSED;
    socket[i].sizeRx = 0;
    

    socket[i].myFlagNewSession = 0;
    socket[i].myFlagGotSyncAck = 0;
    socket[i].myFlagEstablished = 0;
    socket[i].myFlagFoundNoSyncAck = 0;
    socket[i].myFlagSendOk = 0;
    socket[i].myFlagSendFinalACK = 0;
    socket[i].myFlagSendFinalFIN = 0;
    socket[i].myFlagSendCloseACK = 0;
    socket[i].myFlagDoSend = 0;
    socket[i].myFlagWaitForSyncAck = 0;
    socket[i].myFlagDiscon = 0;
    socket[i].myFlagLock = 0;

    socket[i].requestPayload = 0;
    socket[i].previousPayloadForSending = 0;
    socket[i].sessionPort = 0;


    // See Rev 1 - Bug 2
    socket[i].ackNumForSending = 0;
    socket[i].seqNumForSending = 0;

    socket[i].loopCounter = 0;
}

// Clear session part from SocketVal struct
//
static void ClearSocket(SOCKET i)
{
    socket[i].idNum     = 322;
    
    //socket[i].status = SnSR::CLOSED;
    socket[i].sizeRx = 0;
    

    socket[i].myFlagNewSession = 0;
    socket[i].myFlagGotSyncAck = 0;
    socket[i].myFlagEstablished = 0;
    socket[i].myFlagFoundNoSyncAck = 0;
    socket[i].myFlagSendOk = 0;
    socket[i].myFlagSendFinalACK = 0;
    socket[i].myFlagSendFinalFIN = 0;
    socket[i].myFlagSendCloseACK = 0;
    socket[i].myFlagDoSend = 0;
    socket[i].myFlagWaitForSyncAck = 0;
    socket[i].myFlagDiscon = 0;
    socket[i].myFlagLock = 0;
    

    socket[i].requestPayload = 0;
    socket[i].previousPayloadForSending = 0;
    socket[i].sessionPort = 0;


    // See Rev 1 - Bug 2
    socket[i].ackNumForSending = 0;
    socket[i].seqNumForSending = 0;

    socket[i].loopCounter = 0;
    
    
}

// Read and invert ENC28's RX buffer
// This method mirror the buffer to swap
// big endian integers in little endian
//
static void readBufferLittleEndian(unsigned char* buf, unsigned int size)
{
    unsigned char tmpBuf[size];
    
    MACReadRXBuffer(tmpBuf, size);
    
    for (unsigned int k = 0; k < size; k++)
    {
        buf[k] = tmpBuf[size - k - 1];
    }
}

// Read and invert Socket RX buffer
// This method mirror the buffer to swap
// big endian integers in little endian
//
static void readSocketBufferLittleEndian(unsigned char* buf, unsigned int size, unsigned int start)
{
    unsigned char tmpBuf[size];
    
    SOCKETReadBuffer(tmpBuf, size, start);
    
    for (unsigned int k = 0; k < size; k++)
    {
        buf[k] = tmpBuf[size - k - 1];
    }
}

// Read and invert Socket RX buffer
// This method mirror the buffer to swap
// big endian integers in little endian
//
static void writeSocketBufferBigEndian(unsigned char* buf, unsigned int size, unsigned int start)
{
    unsigned char tmpBuf[size];
    
    for (unsigned int k = 0; k < size; k++)
    {
        tmpBuf[k] = buf[size - k - 1];
    }
    
    SOCKETWriteBuffer(tmpBuf, size, start);
}


// Write and invert TX buffer
// This method mirror the buffer to swap 
// little endian integers back into big endian
//
static void writeBufferBigEndian(unsigned char* buf, unsigned int size)
{
    unsigned char tmpBuf[size];
    
    for (unsigned int k = 0; k < size; k++)
    {
        tmpBuf[k] = buf[size - k - 1];
    }
    
    MACWriteTXBuffer(tmpBuf, size);
}

