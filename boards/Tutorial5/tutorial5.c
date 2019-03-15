/*
 * Tutorial 5 
 *
 * The goal of this program is to use timers and global flag variables to show
 * how to schedule two different chunks of code to happen atomically in the 
 * main loop, based on timing, other interrupts, or any other trigger.
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


/* Define global flag variable and different flag locations */
#define SINGLE  0b00000001 /* Could also be written 0x01, 1, etc. */
#define BLINK   0b00000010

uint8_t gFlag = 0;  /* None of the flags are enabled at the start */


/* Define separate global counter variables for each task */
uint8_t gCounterSingle = 0;
const uint8_t single_match = 20;

uint8_t gCounterBlink = 0;
const uint8_t blink_match = 100;

/* Interrupt service routine for timer 0 compare match A interrupt, at 20Hz */
ISR(TIMER0_COMPA_vect) {
    /* We'll have to update the two timers independently here */

    /* If we've counted up 20 times, then reset the counter and execute code */
    if (gCounterSingle == single_match) {
        gCounterSingle = 0;
        
        /* Set the flag bit for SINGLE */
        gFlag |= SINGLE;
        
    /* If it's not the 20th time, just increment the counter by one */
    } else {
        gCounterSingle++;
    }

    
    /* If we've counted up 100 times, then reset the counter and execute code */
    if (gCounterBlink == blink_match) {
        gCounterBlink = 0;
    
        /* Set the flag bit for BLINK */
        gFlag |= BLINK;

    /* If it's not the 100th time, just increment the counter by one */
    } else {
        gCounterBlink++;
    }
}

void setup_timer_20Hz(void) {
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
}

int main (void) {
    
    /* Set global enable interrupt flag */
    sei();
    
    /* Set the data direction register so the led pin is output */
    DDRB |= _BV(LED1_PIN) | _BV(LED2_PIN) | _BV(LED3_PIN);    
    
    /* Set up timer in a helper function */
    setup_timer_20Hz();

    while (1) {
    
        /* If the SINGLE flag has been set */
        if (gFlag & SINGLE) {
            /* Reset the SINGLE flag */
            gFlag &= ~SINGLE;
            /* One long blink */
            PORTB |= _BV(LED2_PIN);
            _delay_ms(300);
            PORTB &= ~_BV(LED2_PIN);
        }
        
        /* If the BLINK flag has been set */
        if (gFlag & BLINK) {
            /* Reset the BLINK flag */
            gFlag &= ~BLINK;
            /* Two tiny blinks */
            PORTB |= _BV(LED1_PIN);
            _delay_ms(100);
            PORTB &= ~_BV(LED1_PIN);
            _delay_ms(100);
            PORTB |= _BV(LED1_PIN);
            _delay_ms(100);
            PORTB &= ~_BV(LED1_PIN);
        }

        /* Otherwise just do nothing until one of the flags is set! */

    }                               
}
