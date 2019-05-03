/*
 * AIR CAN Demo RX
 *
 * This code is a demo of receiving CAN for testing the CAN network in the 2019
 * roadkill harness.
 *
 * @author Alex Hoppe '19
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "can_api.h"

#define LED1_PIN    PC4
#define LED2_PIN    PC5

#define LED_DDR     DDRC
#define LED_PORT    PORTC

/*----- Interrupt(s) -----*/
ISR(CAN_INT_vect) {
    LED_PORT ^= _BV(LED1_PIN);
    CANPAGE = (0 << MOBNB0); // Switch to MOb 0, the one we're listening on.
    if(bit_is_set(CANSTMOB, RXOK)) {
        volatile uint8_t msg = CANMSG;      //grab the first byte of the CAN message

        if(msg == 0xFF) {
            LED_PORT |= _BV(LED2_PIN);
        } else {
            LED_PORT &= ~_BV(LED2_PIN);
        }

        //Setup to Receive Again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(0, CAN_ID_TUTORIAL6, CAN_LEN_TUTORIAL6, 0xFF);
    }
}

int main (void) {
    
    /* Set the data direction register so the led pin is output */
    LED_DDR |= _BV(LED1_PIN) | _BV(LED2_PIN);    
        
    sei();

    /* Initialize CAN */
    CAN_init(CAN_ENABLED);
    
    CAN_wait_on_receive(0, CAN_ID_TUTORIAL6, CAN_LEN_TUTORIAL6, 0xFF);

    while (1) {
        _delay_ms(10);
    }                              
}
