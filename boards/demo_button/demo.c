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

#define LED1_PIN    PB0
#define LED2_PIN    PB1
#define LED3_PIN    PB2

#define PINOUT_1    PB6
#define PINOUT_2    PB7

/* Increment the match value up and down to change PWM duty cycle */
uint8_t pwm_match_value = 16;
uint8_t ascending = 1;

/* Interrupt service routine for timer 0 compare match A interrupt
 * Effectively manually doing PWM on PINOUT_2 */
ISR(TIMER0_COMPA_vect) {
    
    PORTB ^= _BV(LED2_PIN);

    /* turn on pinout 2 */
    PORTB |= _BV(PINOUT_2);

    if (pwm_match_value == 32) {
        ascending = 0;
    }
    if (pwm_match_value == 0) {
        ascending = 1;
    }
    
    /* increment or decrement value accordingly */
    if (ascending) {
        pwm_match_value ++;
    } else {
        pwm_match_value --;
    }
    
    OCR0B = pwm_match_value;
}

ISR(TIMER0_COMPB_vect) {
    PORTB &= ~_BV(PINOUT_2);   
}

int main (void) {
    
    /* Set the data direction register so the led pin is output */
    DDRB |= _BV(LED1_PIN) | _BV(LED2_PIN) | _BV(LED3_PIN) | _BV(PINOUT_2);    
        
    /* Set up input pin with pull-up resistor */
    DDRB &= ~_BV(PINOUT_1);     /* Sanity check pin to input */
    PORTB |= _BV(PINOUT_1);     /* Set output high (if DDR is set to input, 
                                 * this sets a pull-up resistor */

    sei();

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
    TIMSK0 |= _BV(OCIE0B);


    OCR0A = 64;                 /* Set the match register to 195 (maximum 
                                 * period), 3.90625 kHz / 195 = 20.032051 Hz */
    OCR0B = pwm_match_value; 


    while (1) {
        
        /* If button is pressed, turn the light on */
        if (bit_is_clear(PINB, PINOUT_1)) {
            PORTB |= _BV(PINOUT_2);
            TIMSK0 &= ~_BV(OCIE0B); /* Disable interrupt to turn off LED */
        } else {
            TIMSK0 |= _BV(OCIE0B);
        }
    }                               
}
