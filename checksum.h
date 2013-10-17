
#include <inttypes.h>

#define IP_PROTO_ICMP_V		 0x01
#define IP_PROTO_TCP_V		 0x06
#define IP_PROTO_UDP_V		 0x11

#ifndef CHECKSUM_H
#define CHECKSUM_H
uint16_t meu_checksum(uint8_t *header, uint16_t len);
uint16_t    checksum(uint8_t *buf, uint16_t len, uint8_t type);
//void        fillChecksum(uint8_t *buf);

#endif