#ifndef CAN_API_H
#define CAN_API_H

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdlib.h>

/* Message Identifiers */
#define CAN_ID_GLOBAL            ((uint16_t) 0x00)
#define CAN_ID_PANIC             ((uint16_t) 0x01)

#define CAN_ID_BRAKE_LIGHT       ((uint16_t) 0x0B)
#define CAN_ID_THROTTLE          ((uint16_t) 0x0C)
#define CAN_ID_AIR_CONTROL_CRITICAL       ((uint16_t) 0x0D)
#define CAN_ID_AIR_CONTROL_SENSE          ((uint16_t) 0x0E)
#define CAN_ID_DASHBOARD         ((uint16_t) 0x0F)
#define CAN_ID_BMS_CORE        ((uint16_t) 0x10)

// #define CAN_ID_BMS_TEMP          ((uint16_t) 0x12)
#define CAN_ID_STEERING_WHEEL    ((uint16_t) 0x12)
#define CAN_ID_BMS_VOLT          ((uint16_t) 0x13)
#define CAN_ID_BMS_DISCHARGE     ((uint16_t) 0x14)
#define CAN_ID_MOTORCONTROLLER   ((uint16_t) 0x15)
#define CAN_ID_STEERING_WHEEL    ((uint16_t) 0x12)

#define CAN_ID_MC_POSITION       ((uint16_t) 0xA5)
#define CAN_ID_MC_CURRENT        ((uint16_t) 0xA6)
#define CAN_ID_MC_VOLTAGE        ((uint16_t) 0xA7)
#define CAN_ID_MC_COMMAND        ((uint16_t) 0xC0)

#define CAN_ID_BRAKE_PRESSURE    ((uint16_t) 0x18)

#define CAN_ID_TUTORIAL6         ((uint16_t) 0x25)


/* Message Lengths */
#define CAN_LEN_GLOBAL          ((uint16_t) 8)
#define CAN_LEN_PANIC           ((uint16_t) 1)

#define CAN_LEN_BRAKE_LIGHT     ((uint16_t) 7)
#define CAN_LEN_THROTTLE        ((uint16_t) 4)
#define CAN_LEN_AIR_CONTROL_CRITICAL     ((uint16_t) 4)
#define CAN_LEN_AIR_CONTROL_SENSE        ((uint16_t) 6)
#define CAN_LEN_DASHBOARD       ((uint16_t) 3)
#define CAN_LEN_BMS_CORE        ((uint16_t) 7)

// #define CAN_LEN_BMS_TEMP        ((uint16_t) 8)
#define CAN_LEN_BMS_VOLT        ((uint16_t) 8)
#define CAN_LEN_BMS_DISCHARGE   ((uint16_t) 8)
#define CAN_LEN_MOTORCONTROLLER ((uint16_t) 8)
#define CAN_LEN_STEERING_WHEEL  ((uint16_t) 1)

#define CAN_LEN_MC_POSITION     ((uint16_t) 8)
#define CAN_LEN_MC_CURRENT      ((uint16_t) 8)
#define CAN_LEN_MC_VOLTAGE      ((uint16_t) 8)
#define CAN_LEN_MC_COMMAND      ((uint16_t) 8)

#define CAN_LEN_BRAKE_PRESSURE  ((uint16_t) 2)

#define CAN_LEN_TUTORIAL6       ((uint16_t) 1)

/* Masks */
#define CAN_MSK_GLOBAL            ((uint16_t) 0x00)
#define CAN_MSK_SINGLE            ((uint16_t) 0xFF)
#define CAN_MSK_DOUBLE            ((uint16_t) 0xFE)


/* Modes of Operation */
#define CAN_ENABLED      0x00
#define CAN_DISABLED     0x01
#define CAN_LISTEN       0x02


/* Error Definitions */
#define CAN_ERR_MOB_BUSY      0
#define CAN_ERR_NO_RX_FLAG    1
#define CAN_ERR_DLCW          2
#define CAN_ERR_UNKNOWN       3


/* Function Prototypes */

// Initialize CAN system based on MODE of operation
uint8_t CAN_init ( uint8_t mode );

// Transmit a message on the CAN bus
uint8_t CAN_transmit ( uint8_t mob, uint16_t ident, uint8_t msg_length, uint8_t msg[] );

// Quick check to see if transmitted message sent correclty
uint8_t CAN_transmit_success ( uint8_t mob );

// Wait to receive a specific CAN message
uint8_t CAN_wait_on_receive ( uint8_t mob, uint16_t ident, uint8_t msg_length, uint16_t mask );

// Read a message that has been received
uint8_t CAN_read_received ( uint8_t mob, uint8_t msg_length, uint8_t *msg );


#endif
