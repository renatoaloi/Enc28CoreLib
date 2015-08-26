Enc28CoreLib
============

AUG 2015
-----------
Deprecated: Merged into EtherEncLib
https://github.com/renatoaloi/EtherEncLib/
-----------


ENC28J60 TCP Stack Library for Atmel megaAVR series

This Enc28j60 library differ from others, including EtherEncLib.h, because
of where its inspiration came from.

All libraries for ENC28J60 hardware are based on Pascal Stang's implementation.

Enc28CoreLib is based on original Microchip implementation for its PIC plataform,
developed by Howard Schlunder.

The good news is Microchip's implementation do Zero-Copy for buffering HTML data
through ENC28J60's DMA Copy. This approach takes advantage from chip's 8K buffer
and hardware assisted DMA coping data between addresses.

Unfortunately, the real implementation for PIC do not fit in ATMega328 hardware. 

But I managed to accommodate Microchip's concept inside W5100 library, as a hack.
This way, we could change between W5100 and ENC28J60 hardware without changes in
Sketch's code: only changing between library cores.

2013 Dec
Renato Aloi
