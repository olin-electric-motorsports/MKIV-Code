/*
 * Tutorial 0
 *
 * The goal of this program is to show a very basic way to turn on a light to 
 * show simple I/O and bit manipulation.
 *
 * @author Alex Hoppe '19
 */

#include <avr/io.h>

/* Definitions based on board pins */
#define LED_PIN     PB0     
#define PINOUT_1    PB6


int main (void) {                       /* This part of the code runs once */

    /* Set up output pins (various ways) */

    // DDRB = DDRB | 0b00000001 // Using | and binary port access
    // DDRB = DDRB | _BV(PB0);  // Using | and pre-defined name         
    DDRB |= _BV(LED_PIN);       // Using |= and pin name macro (most legible)
    

    /* Set up input pin with pull-up resistor */
    DDRB &= ~_BV(PINOUT_1);     /* Set pin to input (unnecessary, pins
                                 * default to input */

    PORTB |= _BV(PINOUT_1);     /* Set output high (if DDR is set to input, 
                                 * this sets a pull-up resistor */




    while (1) {                     /* This part of the code runs forever */
        
        // if (PINB & _BV(PINOUT_1)) {             /* Alternate way of writing */
        if (bit_is_set(PINB, PINOUT_1)) {       // If the Pinout1 pin is high
            PORTB &= ~_BV(LED_PIN);             // Turn LED off
        } else {                                // Otherwise
            PORTB |= _BV(LED_PIN);              // Turn it on
        }
    }                                   
}
