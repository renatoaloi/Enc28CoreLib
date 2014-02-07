/*
 * Copyright (c) 2013 by Renato Aloi <renato.aloi@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 * 
 * 2013-12-26 :: REV 1 by Renato Aloi (missing DHCP part)
 * 2014-01-28 :: REV 2 by Renato Aloi (radical organization & clean up)
 */

#include "tcpStack.h"
#include <Arduino.h>

extern "C"
{
    #include "enc28j60.h"
}


static unsigned     int         serverSourcePort    = 0;
static unsigned     char        _my_macaddr[]       = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static unsigned     char        _my_ipaddr[]        = { 0x00, 0x00, 0x00, 0x00 };
static unsigned     char        _dest_ipaddr[]      = { 0x00, 0x00, 0x00, 0x00 };
static unsigned     char        _my_gataddr[]       = { 0x00, 0x00, 0x00, 0x00 };
static unsigned     char        _my_subaddr[]       = { 0x00, 0x00, 0x00, 0x00 };


static void readBufferLittleEndian(unsigned char*, unsigned int);
static void writeBufferBigEndian(unsigned char*, unsigned int);
static uint16_t checksum(uint8_t*, uint16_t, uint8_t);
static uint16_t checksum2(uint16_t, uint16_t);
static void readSocketBufferLittleEndian(unsigned char*, unsigned int, unsigned int);
static void writeSocketBufferBigEndian(unsigned char*, unsigned int, unsigned int);
static uint16_t tcp_sum_calc(uint16_t, uint8_t*, uint8_t*, uint8_t, uint16_t);

// Init Method
//
void TcpStackClass::init(unsigned char* __my_macaddr)
{
    _my_macaddr[0] = __my_macaddr[0];
    _my_macaddr[1] = __my_macaddr[1];
    _my_macaddr[2] = __my_macaddr[2];
    _my_macaddr[3] = __my_macaddr[3];
    _my_macaddr[4] = __my_macaddr[4];
    _my_macaddr[5] = __my_macaddr[5];

	// Init MAC Address
    MACInitMacAddr(__my_macaddr);
}

void TcpStackClass::begin(void)
{
	// Init ENC28J60
    MACInit(); 
}

void TcpStackClass::open(void)
{
	// Call Open Func
    MACOpen();

    // wait for stabilization
    delay(10);
}

void TcpStackClass::start(void)
{
	// Enabling reception                
    MACEnableRecv();

    // wait for stabilization
    delay(10);  
}

char TcpStackClass::getMosiPin(void)
{
	return SPI_MOSI;
}

char TcpStackClass::getSckPin(void)
{
	return SPI_SCK;
}

char TcpStackClass::getMisoPin(void)
{
	return SPI_MISO;
}

char TcpStackClass::getControlCsPin(void)
{
	return ENC28J60_CONTROL_CS;
}

void TcpStackClass::setCsPinPassive(void)
{
	CSPASSIVE; 
}

uint16_t TcpStackClass::getRxPointer(void)
{
	return SOCKETGetRxPointer();
}

uint16_t TcpStackClass::getTxPointer(void)
{
	return SOCKETGetTxPointer();
}

void TcpStackClass::setRxPointer(uint16_t _addr)
{
	SOCKETSetRxPointer(_addr);
}

void TcpStackClass::setTxPointer(uint16_t _addr)
{
	SOCKETSetTxPointer(_addr);
}

uint8_t TcpStackClass::getPacketCount()
{
	return MACGetPacketCount();
}

void TcpStackClass::setSourcePort(uint16_t _port)
{
	serverSourcePort = _port;
}

void TcpStackClass::getMacAddress(uint8_t* _mac)
{
	for (int i = 0; i < 6; i++)
    {
        _mac[i] = _my_macaddr[i];
    }
}

void TcpStackClass::setMacAddress(uint8_t* _mac)
{
	for (int i = 0; i < 6; i++)
    {
        _my_macaddr[i] = _mac[i];
    }
}

void TcpStackClass::getIpAddress(uint8_t* _ip)
{
	for (int i = 0; i < 4; i++)
    {
        _ip[i] = _my_ipaddr[i];
    }
}

void TcpStackClass::setIpAddress(uint8_t* _ip)
{
	for (int i = 0; i < 4; i++)
    {
        _my_ipaddr[i] = _ip[i];
    }
}

void TcpStackClass::getGatewayIp(uint8_t* _gateway)
{
	for (int i = 0; i < 4; i++)
    {
        _gateway[i] = _my_gataddr[i];
    }
}

void TcpStackClass::setGatewayIp(uint8_t* _gateway)
{
	for (int i = 0; i < 4; i++)
    {
        _my_gataddr[i] = _gateway[i];
    }
}

void TcpStackClass::getSubnetMask(uint8_t* _subnet)
{
	for (int i = 0; i < 4; i++)
    {
        _subnet[i] = _my_subaddr[i];
    }
}

void TcpStackClass::setSubnetMask(uint8_t* _subnet)
{
	for (int i = 0; i < 4; i++)
    {
        _my_subaddr[i] = _subnet[i];
    }
}

uint8_t TcpStackClass::getHardwareRevision()
{
	return MACHardwareRevision();
}

void TcpStackClass::discardPacket()
{
	MACDiscardRx();
}



TCP_PACKET TcpStackClass::getTcpPacket()
{
    unsigned char socketBuffer[TCP_BUFF_SIZE];
    readBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE);
    return (TCP_PACKET&)socketBuffer;
}

void TcpStackClass::readSocketBuffer(uint8_t* _buf, uint16_t _size, uint16_t _start)
{
    SOCKETReadBuffer(_buf, _size, _start);
}


void TcpStackClass::sendProcessing(const uint8_t *data, uint16_t len, uint16_t _txPointer)
{
    // we dont have circular socket's buffer
    // one write instruction will do
    //
    // Rev1: Instead of circular buffer, we use ENC28J60's
    // Rev1: DMA buffer Copy to perform Zero-Copy operations.
    // Rev1: See DMACopyTo() and DMACopyFrom() functions.
    //
    // Rev2: Eliminating all RAM buffers. Taking full advantage 
    // Rev2: of ENC28J60's hardware's buffer
    // Rev2: Objective is save Arduino's RAM
    // 
    // Rev2 Extended: Remember! TX Buffer has a Control byte! 
    // Rev2 Extended: It must be skiped at checksum!
    //
    SOCKETWriteBuffer((uint8_t *)data, len, _txPointer);
}



// First arrival treatment
// Returns true if is a TCP packet
// and match my IP address
//
uint8_t TcpStackClass::dealRXArrival()
{
	unsigned char socketBuffer[MAX_BUFF_SIZE];
	unsigned char *tmpBuffer; //[MAX_BUFF_SIZE];
	unsigned char tmpHeader[IP_HEADER_LEN_V];
	unsigned int  counter1;

	ETH_HEADER ethPacket;
	ARP_PACKET arpPacket;
	IP_PACKET ipPacket;
	ICMP_PACKET icmpPacket;
	TCP_PACKET tcpPacket2;

	// Read buffer backwards because of
    // big endian integers
	readBufferLittleEndian(socketBuffer, ETH_BUFF_SIZE);

    // Load byte array into struct
    ethPacket = (ETH_HEADER&)socketBuffer;
    
    // Check if is an ARP or ICMP packet
    if (ethPacket.Type.Bytes.ByteH == ETHTYPE_ARP_H_V
        && ethPacket.Type.Bytes.ByteL == ETHTYPE_ARP_L_V)
    {
        // Load ARP struct
        readBufferLittleEndian(socketBuffer, ARP_BUFF_SIZE);
        arpPacket = (ARP_PACKET&)socketBuffer;
        
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
        ipPacket = (IP_PACKET&)socketBuffer;
        
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
                    icmpPacket = (ICMP_PACKET&)socketBuffer;
                    
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
                        tmpBuffer = (unsigned char*)&icmpPacket;
                        counter1 = 0;
                        for(int k = ICMP_BUFF_SIZE - ETH_BUFF_SIZE - 1;
                            k > ICMP_BUFF_SIZE - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
                        {
                            tmpHeader[counter1] = tmpBuffer[k];
                            counter1++;
                        }
                        icmpPacket.ip.Checksum.Value = checksum(tmpHeader, IP_HEADER_LEN_V, 0);
                        
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
                        }
                    }
                    else
                    {
                    	// Not my IP
                    }
                }
                else
                {
                    // Protocol not implemented
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
void TcpStackClass::sendSynResponse(uint16_t _startx)
{
    unsigned char socketBuffer[TCP_BUFF_SIZE_W_OPT];
    unsigned char *tmpBuffer;
    unsigned char tmpHeader[IP_HEADER_LEN_V];
    unsigned char tmpTcpHeader[TCP_HEADER_LEN_W_OPT_V + TCP_CHECKSUM_LEN_V];
    unsigned int counter1 = 0;
    unsigned int tmpDest;
    unsigned long tmpSeq;

    TCP_PACKET_W_OPT tcpPacket;

    // Load TCP struct With Options
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE_W_OPT, _startx);
    tcpPacket = (TCP_PACKET_W_OPT&)socketBuffer;

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
    tcpPacket.ip.Identification.Value = (uint16_t)micros();
    
    // clear the 2 byte Ip checksum
    tcpPacket.ip.Checksum.Bytes.ByteH = 0;
    tcpPacket.ip.Checksum.Bytes.ByteL = 0;
    
    // calculate IP Header the checksum:
    tmpBuffer = (unsigned char*)&tcpPacket;
    for(int k = TCP_BUFF_SIZE_W_OPT - ETH_BUFF_SIZE - 1;
        k > TCP_BUFF_SIZE_W_OPT - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
    {
        tmpHeader[counter1] = tmpBuffer[k];
        counter1++;
    }
    tcpPacket.ip.Checksum.Value = checksum(tmpHeader, IP_HEADER_LEN_V, 0);
    
    // Clear flags
    tcpPacket.tcp.Flags.Value = 0;
    // Set SYNACK flag
    tcpPacket.tcp.Flags.Bits.FlagACK = 1;
    tcpPacket.tcp.Flags.Bits.FlagSYN = 1;
    
    // Inverting ports
    tmpDest = tcpPacket.tcp.DestPort;
    tcpPacket.tcp.DestPort   = tcpPacket.tcp.SourcePort;
    tcpPacket.tcp.SourcePort = tmpDest;
    
    // Inverting seq and ack numbers
    // and adding 1 for computing ACK flag
    tmpSeq = tcpPacket.tcp.SeqNum + 1;
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
    tmpBuffer = (unsigned char*)&tcpPacket;
    
    counter1 = 0;
    for(int k = (TCP_BUFF_SIZE_W_OPT - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V) - 1);
        k > ((TCP_BUFF_SIZE_W_OPT - (IP_BUFF_SIZE - TCP_CHECKSUM_LEN_V) - 1) -
                            (TCP_HEADER_LEN_W_OPT_V + TCP_CHECKSUM_LEN_V)); k--)
    {
        tmpTcpHeader[counter1++] = tmpBuffer[k];


    }

    tcpPacket.tcp.Checksum.Value =
                checksum(tmpTcpHeader, TCP_HEADER_LEN_W_OPT_V + TCP_CHECKSUM_LEN_V, CHECKSUMTCPID);

    // Writing send buffer!
    writeBufferBigEndian((unsigned char*)&tcpPacket, TCP_BUFF_SIZE_W_OPT);
    
    // Commiting send
    SendBufferTx();
}


// Send ACK in response for PSH or FIN
//
void TcpStackClass::sendPshResponse(TCP_PACKET tcpPacket2, uint32_t _seqNum, uint32_t _ackNum)
{
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
    tcpPacket2.ip.Identification.Value = (uint16_t)micros();
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
    tcpPacket2.tcp.SeqNum = _seqNum;
    tcpPacket2.tcp.AckNum = _ackNum;

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
    tcpPacket2.ip.Checksum.Value = checksum(tmpHeader, IP_HEADER_LEN_V, 0);
    
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

// New func - See Rev 2
// Prepare Send - Fill hardware Buffer before sending
//
void TcpStackClass::prepareSend(uint16_t _startrx, uint16_t _payload, uint16_t _starttx, uint32_t _seqNum, uint32_t _ackNum)
{
    unsigned char socketBuffer[TCP_BUFF_SIZE];
    unsigned char tmpHeader[IP_HEADER_LEN_V];
    unsigned int tmpDest;
    unsigned char *tmpInverter;
    unsigned int counter2;

    TCP_PACKET tcpPacket2;

    // Load TCP struct With Options
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, _startrx);
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
    tcpPacket2.ip.TotalLength.Value = TOTAL_HEADER_SIZE + _payload;
    tcpPacket2.ip.Identification.Value = (uint16_t)micros();
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

    _dest_ipaddr[0] = tcpPacket2.ip.DestIp1;
    _dest_ipaddr[1] = tcpPacket2.ip.DestIp2;
    _dest_ipaddr[2] = tcpPacket2.ip.DestIp3;
    _dest_ipaddr[3] = tcpPacket2.ip.DestIp4;
    
    // Inverting ports
    tmpDest = tcpPacket2.tcp.DestPort;
    tcpPacket2.tcp.DestPort   = tcpPacket2.tcp.SourcePort;
    tcpPacket2.tcp.SourcePort = tmpDest;
    
    // See Rev 1 - Bug 1
    // SEQ/ACK numbers are treated 
    // outside the function
    tcpPacket2.tcp.SeqNum = _seqNum; //socket[i].getSeqNum();
    tcpPacket2.tcp.AckNum = _ackNum; //socket[i].getAckNum();

    tcpPacket2.tcp.DataOffset.Bits.NibbleH = 0x5;
    tcpPacket2.tcp.Flags.Value = 0x0;
    tcpPacket2.tcp.Window = 0xFFFF;
    tcpPacket2.tcp.Checksum.Value = 0;
    tcpPacket2.tcp.UrgentPoint = 0;
    
    // Resolving flags and checksums
    tcpPacket2.tcp.Flags.Bits.FlagACK = 1;
    tcpPacket2.tcp.Flags.Bits.FlagPSH = 1;
    
    // calculate IP Header the checksum:
    tmpInverter = (unsigned char*)&tcpPacket2;
    counter2 = 0;
    for(int k = TCP_BUFF_SIZE - ETH_BUFF_SIZE - 1;
    k > TCP_BUFF_SIZE - ETH_BUFF_SIZE - IP_HEADER_LEN_V - 1; k--)
    {
        tmpHeader[counter2] = tmpInverter[k];
        counter2++;
    }
    tcpPacket2.ip.Checksum.Value = checksum(tmpHeader, IP_HEADER_LEN_V, 0);

    // Not now
    tcpPacket2.tcp.Checksum.Value = 0; //0xB98A;

    // Write TCP Header to socket Tx Buffer
    writeSocketBufferBigEndian((unsigned char*)&tcpPacket2, TCP_BUFF_SIZE, _starttx);
}

// New func - See Bug 2 - Rev 1
// Send PSH data though socket
// Must be called only AFTER prepareSend() function execution
// depends on _dest_ipaddr to be filled on prepareSend() func.
//
// Rev2: This func was totally remade to avoid RAM buffers
// Rev2: checksum now reads directly from ENC28J60's hardware
//
// Rev2: Must execute data copy from socket buffer to TX buffer twice!
// Rev2: Second time must advance TX pointer +1 after end.
// Rev2: Added cleaning part before fill TX hardware buffer, to avoid last byte junky.
//
void TcpStackClass::sendPush(uint16_t _starttx, uint16_t _payload)
{
    // DMA Copy from socket buffer to ENC28J60 TX buffer
    // and send it!

    uint16_t ck2;
    unsigned char buff = 0;
    uint16_t size = (TCP_HEADER_SIZE + _payload);
    uint8_t padding = ((size % 2 == 0) ? 0 : 1);

    // Cleaning Data Part
    for (int i = 0; i < _payload + 1; i++)
    {
        MACWriteTXBufferOffset2(&buff, 1, TCP_BUFF_SIZE + i, 1);
    }
    
    // DMA Copying TCP Header part
    DMACopyFrom(TX, _starttx, TCP_BUFF_SIZE);
    
    // Wait for copy
    while(!IsDMACopyDone());

    // Moving Data Part from
    // one ENC28J60 memory address to 
    // to another.
    //
    // Rev2: All RAM buffers were terminated! For good!
    // Rev2: No more socketWriter buffer, 
    // Rev2: just byte-to-byte hardware copy!
    //
    for (int i = 0; i < _payload; i++)
    {
        SOCKETReadBuffer(&buff, 1, _starttx + TCP_BUFF_SIZE + i);
        // Write data part
        MACWriteTXBufferOffset2(&buff, 1, TCP_BUFF_SIZE + i, (i!=0));
    }

    // TCP Checksum (DMA Copy Based)
    // no RAM buffer used!
    ck2 = tcp_sum_calc(size, _my_ipaddr, _dest_ipaddr, padding, _starttx + IP_BUFF_SIZE);

    // Filling checksum fields
    buff = (ck2>>8) & 0xFF;
    MACWriteTXBufferOffset2(&buff, 1, TCP_BUFF_SIZE - 3, 1);
    buff = (ck2) & 0xFF;
    MACWriteTXBufferOffset2(&buff, 1, TCP_BUFF_SIZE - 2, 1);

    // Data Part AGAIN!
    // This is necessary to put 
    // TX pointer at correct location
    // REV2 - we need to move TX pointer
    //        one byte forward, after
    //        payload len
    for (int i = 0; i < _payload + 1; i++)
    {
        SOCKETReadBuffer(&buff, 1, _starttx + TCP_BUFF_SIZE + i);
        // Write data part
        MACWriteTXBufferOffset2(&buff, 1, TCP_BUFF_SIZE + i, (i!=0));
    }

    // Commiting send
    SendBufferTx();
}

// Send ACK in response
//
void TcpStackClass::sendAckResponse(TCP_PACKET tcpPacket2, uint32_t _seqNum, uint32_t _ackNum)
{
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
    tcpPacket2.ip.Identification.Value = (uint16_t)micros();
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
    
    // Updating ACK/SEQ numbers
    tcpPacket2.tcp.SeqNum = _seqNum;
    tcpPacket2.tcp.AckNum = _ackNum;

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
    tcpPacket2.ip.Checksum.Value = checksum(tmpHeader, IP_HEADER_LEN_V, 0);
    
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

// Send Fin request
//
void TcpStackClass::sendFinRequest(uint16_t _startx, uint8_t _internalStatus)
{
    // Load TCP struct 
    uint8_t socketBuffer[TCP_BUFF_SIZE];
    readSocketBufferLittleEndian(socketBuffer, TCP_BUFF_SIZE, _startx);
    TCP_PACKET tcpPacket2 = (TCP_PACKET&)socketBuffer;
    
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
    tcpPacket2.ip.Identification.Value = (uint16_t)micros();
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
    unsigned long tmpSeqNum = tcpPacket2.tcp.SeqNum;
    tcpPacket2.tcp.SeqNum = tcpPacket2.tcp.AckNum;
    tcpPacket2.tcp.AckNum = tmpSeqNum;

    // If is FIN_WAIT status, 
    // we need to add 1 to ack num
    if ((_internalStatus & SOCKET_FIN_WAIT) == SOCKET_FIN_WAIT)
    {
        tcpPacket2.tcp.AckNum++;
    }

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
    tcpPacket2.ip.Checksum.Value = checksum(tmpHeader, IP_HEADER_LEN_V, 0);
    
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

// Send Buffer and waits for response
//
void TcpStackClass::SendBufferTx(void)
{
    unsigned long timerSendTrigger = millis();

    // Sending buffer
    MACSendTx();

    // Waiting for buffer to be sent.
    while(!IsMACSendTx())
    {
        if (timerSendTrigger + 1000 < millis())
        {
            //Serial.println(F("ERRO ENVIO!"));
            //delay(10);

            /*
            for (int i = 0; i < MAX_SOCK_NUM; i++)
            {
                socket[i].setStatus(SnSR::CLOSED);
                socket[i].setInternalStatus(SOCKET_CLOSED);
            }

            // Init MAC Address
            MACInitMacAddr(_my_macaddr);
            // Init ENC28J60
            MACInit(); 
            // Call Open Func
            MACOpen();
            // wait for stabilization
            delay(10);
            // Enabling reception                
            MACEnableRecv();
            // wait for stabilization
            delay(10);  
            */

            break;
        }
    }
}























// Read and invert ENC28's RX buffer
// This method mirror the buffer to swap
// big endian integers in little endian ones
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



// The Ip checksum is calculated over the ip header only starting
// with the header length field and a total length of 20 bytes
// unitl ip.dst
// You must set the IP checksum field to zero before you start
// the calculation.
// len for ip is 20.
//
// For UDP/TCP we do not make up the required pseudo header. Instead we 
// use the ip.src and ip.dst fields of the real packet:
// The udp checksum calculation starts with the ip.src field
// Ip.src=4bytes,Ip.dst=4 bytes,Udp header=8bytes + data length=16+len
// In other words the len here is 8 + length over which you actually
// want to calculate the checksum.
// You must set the checksum field to zero before you start
// the calculation.
// len for udp is: 8 + 8 + data length
// len for tcp is: 4+4 + 20 + option len + data length
//
// For more information on how this algorithm works see:
// http://www.netfor2.com/checksum.html
// http://www.msc.uky.edu/ken/cs471/notes/chap3.htm
// The RFC has also a C code example: http://www.faqs.org/rfcs/rfc1071.html
static uint16_t checksum(uint8_t *buf, uint16_t len, uint8_t type)
{
    // type 0=ip 
    //      1=udp
    //      2=tcp
    uint32_t sum = 0;
    uint16_t tmp = 0;

    //if(type==0){
    //        // do not add anything
    //}
    if(type == 1)
    {
        sum += IP_PROTO_UDP_V; // protocol udp
        // the length here is the length of udp (data+header len)
        // =length given to this function - (IP.scr+IP.dst length)
        sum += len - 8; // = real tcp len
    }
    if(type == 2)
    {
        sum += IP_PROTO_TCP_V; 
        // the length here is the length of tcp (data+header len)
        // =length given to this function - (IP.scr+IP.dst length)
        sum += len - 8; // = real tcp len
    }

    // build the sum of 16bit words
    while(len > 1)
    {
        tmp = 0xFFFF & (*(buf)<<8|*(buf+1));
        sum += tmp;
        buf += 2;
        len -= 2;
    }
    // if there is a byte left then add it (padded with zero)
    if (len)
    {
        sum += (0xFF & *buf)<<8;
    }
    // now calculate the sum over the bytes in the sum
    // until the result is only 16bit long
    while (sum>>16)
    {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    // build 1's complement:
    return((uint16_t) sum ^ 0xFFFF);
}

static uint16_t tcp_sum_calc(uint16_t len_tcp, uint8_t* src_addr, uint8_t* dest_addr, uint8_t padding, uint16_t buff_pointer)
{
    uint8_t buf[2];
    uint16_t prot_tcp = 6;
    uint16_t padd = 0;
    uint16_t word16;
    uint32_t sum;  
    uint16_t i, j;  

    // Find out if the length of data is even or odd number. If odd,
    // add a padding byte = 0 at the end of packet
    if (padding&1==1){
        padd=1;
    }
    
    //initialize sum to zero
    sum=0;
    
    // make 16 bit words out of every two adjacent 8 bit words and 
    // calculate the sum of all 16 bit words
    for (i=0;i<len_tcp+padd;i=i+2)
    {
        // Skipping control byte
        j = 0;
        if (i > 19) j = 1;

        if (i < (len_tcp - 1))
        {
            SOCKETReadBuffer(buf, 2, buff_pointer + i + j);
        }
        else //if (i < len_tcp + padd)
        {
            SOCKETReadBuffer(buf, 1, buff_pointer + i + j);
            buf[1] = 0;
        }
        word16 =((buf[0]<<8)&0xFF00)+(buf[1]&0xFF);
        sum = sum + (unsigned long)word16;
    }   
    // add the TCP pseudo header which contains:
    // the IP source and destinationn addresses,
    for (i=0;i<4;i=i+2){
        word16 =((src_addr[i]<<8)&0xFF00)+(src_addr[i+1]&0xFF);
        sum=sum+word16; 
    }
    for (i=0;i<4;i=i+2){
        word16 =((dest_addr[i]<<8)&0xFF00)+(dest_addr[i+1]&0xFF);
        sum=sum+word16;     
    }
    // the protocol number and the length of the TCP packet
    sum = sum + prot_tcp + len_tcp;
    // keep only the last 16 bits of the 32 bit calculated sum and add the carries
    while (sum>>16)
        sum = (sum & 0xFFFF)+(sum >> 16);
    // Take the one's complement of sum
    sum = ~sum;
    return ((uint16_t) sum);
}