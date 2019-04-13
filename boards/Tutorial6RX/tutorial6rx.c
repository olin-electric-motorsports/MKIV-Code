/*
 * Tutorial 6 RX
 *
 * This board is a demo of receiving CAN. It receives a CAN message on 0x25 and if it's 
 * 0xFF, it turns on the LED, otherwise it turns it off.
 *
 * All reference info is pulled from
 * https://www.nongnu.org/avr-libc/user-manual/index.html
 * http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8209-8-bit%20AVR%20ATmega16M1-32M1-64M1_Datasheet.pdf
 *
 * @author Alex Hoppe '19
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "can_api.h"

#define LED1_PIN    PB0
#define LED2_PIN    PB1
#define LED3_PIN    PB2

#define PINOUT_1    PB6
#define PINOUT_2    PB7

/*----- Interrupt(s) -----*/
ISR(CAN_INT_vect) {    
    CANPAGE = (0 << MOBNB0); // Switch to MOb 0, the one we're listening on.
    if(bit_is_set(CANSTMOB, RXOK)) {
        volatile uint8_t msg = CANMSG;      //grab the first byte of the CAN message

        if(msg == 0xFF) {
            PORTB |= _BV(LED2_PIN);
        } else {
            PORTB &= ~_BV(LED2_PIN);
        }

        //Setup to Receive Again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(0, CAN_ID_TUTORIAL6, CAN_LEN_TUTORIAL6, 0xFF);
    }
}

int main (void) {
    
    /* Set the data direction register so the led pin is output */
    DDRB |= _BV(LED1_PIN) | _BV(LED2_PIN) | _BV(LED3_PIN) | _BV(PINOUT_2);    
        
    /* Set up input pin with pull-up resistor */
    DDRB &= ~_BV(PINOUT_1);     /* Sanity check pin to input */
    PORTB |= _BV(PINOUT_1);     /* Set output high (if DDR is set to input, 
                                 * this sets a pull-up resistor */
    sei();

    /* Initialize CAN */
    CAN_init(CAN_ENABLED);
    
    CAN_wait_on_receive(0, CAN_ID_TUTORIAL6, CAN_LEN_TUTORIAL6, 0xFF);

    while (1) {
        _delay_ms(250);
    }                              
}
