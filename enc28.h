/*
 * Copyright (c) 2013 by Renato Aloi <renato.aloi@gmail.com>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef	ENC28_H_INCLUDED
#define	ENC28_H_INCLUDED

#include "enc28socket.h"



class ENC28Class {

public:
    void init();

    /**
    * @brief	This function is being used for copy the data form Receive buffer of the chip to application buffer.
    * 
    * It calculate the actual physical address where one has to read
    * the data from Receive buffer. Here also take care of the condition while it exceed
    * the Rx memory uper-bound of socket.
    */
   void read_data(SOCKET s, volatile uint8_t * src, volatile uint8_t * dst, uint16_t len);
   
   /**
    * @brief	 This function is being called by send() and sendto() function also. 
    * 
    * This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
    * register. User should read upper byte first and lower byte later to get proper value.
    */
   void send_data_processing(SOCKET s, const uint8_t *data, uint16_t len);
   /**
    * @brief A copy of send_data_processing that uses the provided ptr for the
    *        write offset.  Only needed for the "streaming" UDP API, where
    *        a single UDP packet is built up over a number of calls to
    *        send_data_processing_ptr, because TX_WR doesn't seem to get updated
    *        correctly in those scenarios
    * @param ptr value to use in place of TX_WR.  If 0, then the value is read
    *        in from TX_WR
    * @return New value for ptr, to be used in the next call
    */
 // FIXME Update documentation
   void send_data_processing_offset(SOCKET s, uint16_t data_offset, const uint8_t *data, uint16_t len);
 
   /**
    * @brief	This function is being called by recv() also.
    * 
    * This function read the Rx read pointer register
    * and after copy the data from receive buffer update the Rx write pointer register.
    * User should read upper byte first and lower byte later to get proper value.
    */
   void recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek = 0);
 
    void setGatewayIp(uint8_t *_addr);
    void getGatewayIp(uint8_t *_addr);
 
    void setSubnetMask(uint8_t *_addr);
    void getSubnetMask(uint8_t *_addr);
 
    void setMACAddress(uint8_t * addr);
    void getMACAddress(uint8_t * addr);
 
    void setIPAddress(uint8_t * addr);
    void getIPAddress(uint8_t * addr);
 
    void setRetransmissionTime(uint16_t timeout);
    void setRetransmissionCount(uint8_t _retry);
 
   void execCmdSn(SOCKET s, SockCMD _cmd);
   
   uint16_t getTXFreeSize(SOCKET s);
   uint16_t getRXReceivedSize(SOCKET s);
   

   
    // ENC28 Socket registers
    // ----------------------
public:
    uint8_t  readSnMR(SOCKET _s);                       // Mode
    uint8_t  readSnSR(SOCKET _s);                       // Status
    uint8_t  readSnIR(SOCKET _s);                       // Interrupt
    uint16_t readSnRX_RD(SOCKET _s);                    // RX Read Pointer
    uint16_t readSnTX_WR(SOCKET _s);                    // TX Write Pointer
    
    void    writeSnMR(SOCKET _s, uint8_t _flag);        // Mode
    void    writeSnIR(SOCKET _s, uint8_t _flag);        // Interrupt
    void    writeSnPORT(SOCKET _s, uint16_t _addr);      // Source Port
    void    writeSnRX_RD(SOCKET _s, uint16_t _addr);    // RX Read Pointer
    void    writeSnTX_WR(SOCKET _s, uint16_t _addr);    // TX Write Pointer
    
    void    writeSnDIPR(SOCKET _s, uint8_t* _addr);
    void    writeSnDPORT(SOCKET _s, uint16_t _addr);

public:
    static const uint16_t SSIZE = 1024;  // 8k total SRAM from ENC28J60
    static const uint16_t RSIZE = 1024;  // 4k for RX/TX buffers
                                        // 4k for RX/TX socket buffers
                                        // where:
                                        // 1k for each socket
                                        //    1024 bytes for socket RX
                                        //    1024 bytes for socket TX
                                        // (1024 x 2) x 2 sockets = 4k
    
};

extern ENC28Class ENC28;

#endif