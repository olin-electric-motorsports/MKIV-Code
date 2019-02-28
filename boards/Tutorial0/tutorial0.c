/*
 * Tutorial 0
 *
 * The goal of this program is to show a very basic way to turn on a light to 
 * show simple I/O and bit manipulation.
 *
 * @author Alex Hoppe '19
 */

#include <avr/io.h>


int main (void) { /* This part of the code runs once */

    /* Set up output pins (various ways) */

    // DDRB = DDRB | 0b00000001     // Using | and binary port access
    DDRB = DDRB | _BV(PB0);         // Using | and pre-defined name         
    

    /* Set up input pin with pull-up resistor */
    DDRB &= ~_BV(PB6);          /* Set pin to input (unnecessary, pins
                                 * default to input */

    PORTB |= _BV(PB6);          /* Set output high (if DDR is set to input, 
                                 * this sets a pull-up resistor */


    while (1) { /* This part of the code runs forever */
        
        // if (PINB & _BV(PB6)) {           /* Alternate way of writing */
        if (bit_is_set(PINB, PB6)) {        // If the Pinout1 pin is high
            PORTB &= ~_BV(PB0);             // Turn LED off
        } else {                            // Otherwise
            PORTB |= _BV(PB0);              // Turn it on
        }
    }                                   
}
