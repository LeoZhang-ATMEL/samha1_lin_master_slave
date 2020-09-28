/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _LIN_COMMON_H    /* Guard against multiple inclusion */
#define _LIN_COMMON_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include <stdint.h>
/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRANSMIT,
    RECEIVE,
    ERROR // Used for LIN Master
}lin_packet_type_t;

typedef union {
    struct {
        uint8_t SYNC; /* Only used for Master */
        uint8_t PID;
        uint8_t data[8 + 1]; // Data + Checksum
        lin_packet_type_t type;
        uint8_t length; /* Data length, less than 8 */
    };
    uint8_t rawPacket[13];
}lin_packet_t;

typedef union {
    struct {
        unsigned ID0: 1;
        unsigned ID1: 1;
        unsigned ID2: 1;
        unsigned ID3: 1;
        unsigned ID4: 1;
        unsigned ID5: 1;
        unsigned P0: 1;
        unsigned P1: 1;
    };
    uint8_t rawPID;
} lin_pid_t;

uint8_t LIN_calcParity(uint8_t CMD);
uint8_t LIN_getChecksum(uint8_t* data, uint8_t length);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _LIN_COMMON_H */

/* *****************************************************************************
 End of File
 */
