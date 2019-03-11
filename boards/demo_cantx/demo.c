/*
 * Demo of start button 
 *
 * This program is designed for Olin Unfiltered at Candidates' Weekends.
 * The button blinks an LED
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

#define CAN_ID_DEMO     0x25
#define CAN_LEN_DEMO    1

uint8_t msg[1];

ISR(PCINT0_vect) {
    /* If button is pressed, turn the light on */
    if (bit_is_clear(PINB, PINOUT_1)) {
        PORTB |= _BV(LED2_PIN);
    } else {
        PORTB &= ~_BV(LED2_PIN);
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
    
    /* set up pin change interrupts */    
    PCICR |= _BV(PCIE0);
    PCMSK0 |= _BV(PINOUT_1);

    /* Initialize CAN */
    CAN_init(CAN_ENABLED);

    while (1) {
        PORTB ^= _BV(LED3_PIN);
        
        /* If button is pressed, turn the light on */
        if (bit_is_clear(PINB, PINOUT_1)) {
            msg[0] = 0x00;
        } else {
            msg[0] = 0xFF;
        }
        CAN_transmit(0, CAN_ID_DEMO, CAN_LEN_DEMO, msg);
        
        _delay_ms(250);
    }                               
}
