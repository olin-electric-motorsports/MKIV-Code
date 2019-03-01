/*
 * Tutorial 2 
 *
 * The goal of this program is to demonstrate the use of pin-change interrupts 
 * to free up the main loop for blocking code, like blinking LEDs with delays.
 *
 * All reference info is pulled from
 * https://www.nongnu.org/avr-libc/user-manual/index.html
 *
 * @author Alex Hoppe '19
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LED1_PIN    PB0
#define LED2_PIN    PB1
#define LED3_PIN    PB2

#define PINOUT_1    PB6
#define PINOUT_2    PB7

/* Interrupt service routine for pin change interrupts on PCINT[0:7] */
ISR(PCINT0_vect) {
    
    /* Since any of the pins in PCI0 could've triggered this interrupt, test if
     * PB6 is pulled low. If so, set the LED on. Otherwise, turn it off */

    if (bit_is_set(PINB, PINOUT_1)) {           // If the Pinout1 pin is high
            PORTB &= ~_BV(LED1_PIN);             // Turn LED off
        } else {                                // Otherwise
            PORTB |= _BV(LED1_PIN);              // Turn it on
    }
}    

int main (void) {
    
    /* Set the data direction register so the led pin is output */
    DDRB |= _BV(LED1_PIN) | _BV(LED2_PIN) | _BV(LED3_PIN);    
        
    /* Set up input pin with pull-up resistor */
    DDRB &= ~_BV(PINOUT_1);     /* Sanity check pin to input */
    PORTB |= _BV(PINOUT_1);     /* Set output high (if DDR is set to input, 
                                 * this sets a pull-up resistor */

    /* Set up pin change interrupts on PCINT6 (PB6, PINOUT_1) */
    sei();                      /* Set global enable interrupt flag */
    PCICR |= _BV(PCIE0);        /* Enable 0th bank of pin change interrupts */
    PCMSK0 |= _BV(PCINT6);      /* Enable PCINT6 to trigger interrupts*/

    while (1) {
        
        /* Toggle LED2_pin each cycle */
        PORTB ^= _BV(LED2_PIN);
        _delay_ms(500);
    }                               
}
