/*
 * Tutorial 3 
 *
 * The goal of this program is to demonstrate the use of timer interrupts to 
 * remove delay-based events from the main loop and have them run
 * asynchronously. This program will blink an LED based on TC0
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

/* Interrupt service routine for timer 0 compare match A interrupt */
ISR(TIMER0_COMPA_vect) {
    
    /* Toggle LED 3 */
    PORTB ^= _BV(LED3_PIN);
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

    
    /* Set up TC0 in ctc mode, with OCR0A interrupt enabled and at 2 Hz */    
    TCCR0A |= _BV(WGM01);       /* Set WGM[2:0] to 0b010: CTC mode (WGM2 is 
                                 * TCCR0B) */
    
    TCCR0B |= _BV(CS02) | _BV(CS00);    /* Set up prescaler for TC0 to clk_io
                                         * divided by 1024: 
                                         * 4MHz/1024 = 3.90625 kHz 
                                         * CS0[2:0] = 0b101  
                                         * (table 15-10) in datasheet */

    TIMSK0 |= _BV(OCIE0A);      /* Set interrupt enable for output channel A
                                 * interrupt on compare match */

    OCR0A = 195;                /* Set the match register to 195 (maximum 
                                 * period), 3.90625 kHz / 195 = 20.032051 Hz */

    
    while (1) {
        
        /* Toggle LED2_pin each cycle */
        PORTB ^= _BV(LED2_PIN);
        _delay_ms(500);
    }                               
}
