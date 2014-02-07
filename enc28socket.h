#include <stdint.h>

#ifndef ENC28SOCKET_H_INCLUDED
#define ENC28SOCKET_H_INCLUDED

#define MAX_SOCK_NUM             1
#define CHECKSUMTCPID            2

#define ETH_BUFF_SIZE            14
#define ARP_BUFF_SIZE            42
#define IP_BUFF_SIZE             34
#define ICMP_BUFF_SIZE           98
#define TCP_BUFF_SIZE_W_OPT      58
#define TCP_BUFF_SIZE            54
#define MAX_BUFF_SIZE            98

#define TOTAL_HEADER_SIZE        (TCP_BUFF_SIZE - ETH_BUFF_SIZE)
#define TCP_HEADER_SIZE          (TCP_BUFF_SIZE - IP_BUFF_SIZE)


// ******* ETH VALUES *******
#define ETHTYPE_ARP_H_V          0x08
#define ETHTYPE_ARP_L_V          0x06
#define ETHTYPE_IP_H_V           0x08
#define ETHTYPE_IP_L_V           0x00
#define ETH_HEADER_LEN_V         0x0E

// ******* ARP VALUES *******
#define ETH_ARP_OPCODE_REPLY_H_V 0x00
#define ETH_ARP_OPCODE_REPLY_L_V 0x02

// ******* IP VALUES *******
#define IP_PROTO_ICMP_V		 0x01
#define IP_PROTO_TCP_V		 0x06
#define IP_PROTO_UDP_V		 0x11
#define IP_V4_V			 0x40
#define IP_HEADER_LEN_VER_V      0x45
#define IP_HEADER_LEN_V		 0x14

// ******* ICMP VALUES *******
#define ICMP_TYPE_ECHOREQUEST_V  0x08
#define ICMP_TYPE_ECHOREPLY_V    0x00

// ******* TCP VALUES *******
#define TCP_OPT_LEN_V            0x04
#define TCP_FLAGS_FIN_V		 0x01 //
#define TCP_FLAGS_SYN_V		 0x02 //
#define TCP_FLAGS_RST_V          0x04 //
#define TCP_FLAGS_RSTACK_V       0x14 //
#define TCP_FLAGS_PUSH_V         0x08 //
#define TCP_FLAGS_ACK_V		 0x10 //
#define TCP_FLAGS_FINACK_V	 0x11 //
#define TCP_FLAGS_SYNACK_V 	 0x12 //
#define TCP_FLAGS_PSHACK_V       0x18 //
#define TCP_SEQ_NUM_INI_HH_V     0xC0
#define TCP_SEQ_NUM_INI_HL_V     0xC7
#define TCP_SEQ_NUM_INI_LH_V     0x56
#define TCP_SEQ_NUM_INI_LL_V     0xEF
#define TCP_HEADER_LEN_PLAIN_V   0x14
#define TCP_HEADER_LEN_W_OPT_V   0x18
#define TCP_CHECKSUM_LEN_V       0x08





typedef uint8_t SOCKET;


union FIELD_16
{
    struct
    {
        unsigned char ByteL;
        unsigned char ByteH;
    } Bytes;
    unsigned int Value;
    
} ;

union FIELD_8
{
    struct
    {
        unsigned char NibbleL   : 4;
        unsigned char NibbleH  : 4;
    } Bits;
    unsigned char Value;
} ;

typedef struct
{
    union FIELD_16 Type;
    unsigned char SrcMac6;
    unsigned char SrcMac5;
    unsigned char SrcMac4;
    unsigned char SrcMac3;
    unsigned char SrcMac2;
    unsigned char SrcMac1;
    unsigned char DestMac6;
    unsigned char DestMac5;
    unsigned char DestMac4;
    unsigned char DestMac3;
    unsigned char DestMac2;
    unsigned char DestMac1;
    
} ETH_HEADER;

typedef struct
{
    unsigned char DestIp4;
    unsigned char DestIp3;
    unsigned char DestIp2;
    unsigned char DestIp1;
    unsigned char DestMac6;
    unsigned char DestMac5;
    unsigned char DestMac4;
    unsigned char DestMac3;
    unsigned char DestMac2;
    unsigned char DestMac1;
    unsigned char SrcIp4;
    unsigned char SrcIp3;
    unsigned char SrcIp2;
    unsigned char SrcIp1;
    unsigned char SrcMac6;
    unsigned char SrcMac5;
    unsigned char SrcMac4;
    unsigned char SrcMac3;
    unsigned char SrcMac2;
    unsigned char SrcMac1;
    unsigned int OperationCode;
    unsigned char ProtocolAddressLen;
    unsigned char HardwareAddressLen;
    union FIELD_16 ProtocolType;
    unsigned int HardwareType;
    
} ARP_HEADER;

typedef struct
{
    unsigned long Reserved14;
    unsigned long Reserved13;
    unsigned long Reserved12;
    unsigned long Reserved11;
    unsigned long Reserved10;
    unsigned long Reserved9;
    unsigned long Reserved8;
    unsigned long Reserved7;
    unsigned long Reserved6;
    unsigned long Reserved5;
    unsigned long Reserved4;
    unsigned long Reserved3;
    unsigned long Reserved2;
    unsigned long Reserved1;
    
    unsigned int Reserved;
    unsigned int Identifier;
    union FIELD_16 Checksum;
    unsigned char Code;
    unsigned char Type;
    
} ICMP_HEADER;

typedef struct
{
    unsigned char DestIp4;
    unsigned char DestIp3;
    unsigned char DestIp2;
    unsigned char DestIp1;
    unsigned char SrcIp4;
    unsigned char SrcIp3;
    unsigned char SrcIp2;
    unsigned char SrcIp1;
    union FIELD_16 Checksum;
    union FIELD_8 Protocol;
    union FIELD_8 TimeToLive;
    
    union FIELD_16 FragmentFlags;
    
    union FIELD_16 Identification;
    union FIELD_16 TotalLength;
    unsigned char DscpEcn;
    union FIELD_8 Version;
    
} IP_HEADER;

#define FIN     (0x01)		// FIN Flag as defined in RFC
#define SYN     (0x02)		// SYN Flag as defined in RFC
#define RST     (0x04)		// Reset Flag as defined in RFC
#define PSH     (0x08)		// Push Flag as defined in RFC
#define ACK     (0x10)		// Acknowledge Flag as defined in RFC
#define URG     (0x20)		// Urgent Flag as defined in RFC
typedef struct
{
    unsigned int UrgentPoint;
    union FIELD_16 Checksum;
    unsigned int Window;
    
    union
    {
        struct
        {
            unsigned char FlagFIN    : 1;
            unsigned char FlagSYN    : 1;
            unsigned char FlagRST    : 1;
            unsigned char FlagPSH    : 1;
            unsigned char FlagACK    : 1;
            unsigned char FlagURG    : 1;
            unsigned char Reserved2  : 2;
        } Bits;
        unsigned char Value;
    } Flags;
    
    union FIELD_8 DataOffset;
    unsigned long AckNum;
    unsigned long SeqNum;
    unsigned int  DestPort;
    unsigned int  SourcePort;
    
} TCP_HEADER;

#define TCP_OPTIONS_END_OF_LIST     (0x00u)		// End of List TCP Option Flag
#define TCP_OPTIONS_NO_OP           (0x01u)		// No Op TCP Option
#define TCP_OPTIONS_MAX_SEG_SIZE    (0x02u)		// Maximum segment size TCP flag
typedef struct
{
    unsigned int         MaxSegSize;
    unsigned char        Length;
    unsigned char        Kind;
    
} TCP_OPTIONS;

typedef struct
{
    ARP_HEADER  arp;
    ETH_HEADER eth;

} ARP_PACKET;

typedef struct
{
    ICMP_HEADER  icmp;
    IP_HEADER ip;
    ETH_HEADER eth;

} ICMP_PACKET;

typedef struct
{
    IP_HEADER ip;
    ETH_HEADER eth;

} IP_PACKET;

typedef struct
{
    TCP_HEADER  tcp;
    IP_HEADER ip;
    ETH_HEADER eth;

} TCP_PACKET;

typedef struct
{
    TCP_OPTIONS  opt;
    TCP_HEADER  tcp;
    IP_HEADER ip;
    ETH_HEADER eth;

} TCP_PACKET_W_OPT;


class SnMR {
public:
    static const uint8_t CLOSE  = 0x00;
    static const uint8_t TCP    = 0x01;
    static const uint8_t UDP    = 0x02;
    static const uint8_t IPRAW  = 0x03;
    static const uint8_t MACRAW = 0x04;
    static const uint8_t PPPOE  = 0x05;
    static const uint8_t ND     = 0x20;
    static const uint8_t MULTI  = 0x80;
};

enum SockCMD {
    Sock_OPEN      = 0x01,
    Sock_LISTEN    = 0x02,
    Sock_CONNECT   = 0x04,
    Sock_DISCON    = 0x08,
    Sock_CLOSE     = 0x10,
    Sock_SEND      = 0x20,
    Sock_SEND_MAC  = 0x21,
    Sock_SEND_KEEP = 0x22,
    Sock_RECV      = 0x40
};

class SnIR {
public:
    static const uint8_t SEND_OK = 0x10;
    static const uint8_t TIMEOUT = 0x08;
    static const uint8_t RECV    = 0x04;
    static const uint8_t DISCON  = 0x02;
    static const uint8_t CON     = 0x01;
};

class SnSR {
public:
    static const uint8_t CLOSED      = 0x00;
    static const uint8_t INIT        = 0x13;
    static const uint8_t LISTEN      = 0x14;
    static const uint8_t SYNSENT     = 0x15;
    static const uint8_t SYNRECV     = 0x16;
    static const uint8_t ESTABLISHED = 0x17;
    static const uint8_t FIN_WAIT    = 0x18;
    static const uint8_t CLOSING     = 0x1A;
    static const uint8_t TIME_WAIT   = 0x1B;
    static const uint8_t CLOSE_WAIT  = 0x1C;
    static const uint8_t LAST_ACK    = 0x1D;
    static const uint8_t UDP         = 0x22;
    static const uint8_t IPRAW       = 0x32;
    static const uint8_t MACRAW      = 0x42;
    static const uint8_t PPPOE       = 0x5F;
};

class IPPROTO {
public:
    static const uint8_t IP   = 0;
    static const uint8_t ICMP = 1;
    static const uint8_t IGMP = 2;
    static const uint8_t GGP  = 3;
    static const uint8_t TCP  = 6;
    static const uint8_t PUP  = 12;
    static const uint8_t UDP  = 17;
    static const uint8_t IDP  = 22;
    static const uint8_t ND   = 77;
    static const uint8_t RAW  = 255;
};


#endif