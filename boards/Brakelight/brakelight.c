/*
Header:
	Code for the Brakelight/BSPD/Shutdown Sense Board lcoated
	in the LV-Box enclosure
Author:
	@author Jerry Goss '19
*/

/*----- RJ45 LEDs -----*/
//Green: //TODO
//Orange:

/*----- Includes -----*/
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

/*----- Macro Definitions -----*/
/* Shutdown */
#define GLOBAL_SHUTDOWN         0x0
// for gTimerFlag
//TODO

#define UPDATE_STATUS 0

// uint8_t msg[1];
volatile uint8_t gTimerFlag = 0x01; // Timer Flag

/* pins */

#define BRAKE_GATE_PIN          PD1
#define BRAKE_GATE_PIN_BANK     PIND

/* LEDs */
#define LED1                    PD6
#define LED1_PORT               PORTD
#define LED2                    PD7
#define LED2_PORT               PORTD
#define LED3                    PB2
#define LED3_PORT               PORTB

#define EXT_LED1                PB23
#define EXT_LED2                PB24
#define EXT_LED_PORT            PORTB

/* CAN Positions */
#define CAN_BRAKE_ANALOG_MSB	0
#define CAN_BRAKE_ANALOG_LSB	1
#define CAN_BRAKE_GATE        2
#define CAN_BSPD              3
#define CAN_TSMS            	4
#define CAN_ESTOP	            5
#define CAN_GLVMS		          6

/* FLAGS */
#define FLAG_BRAKE	 0b00000001
//TODO

/* MOBs */
#define MOB_BRAKELIGHT		    0
// TODO


/* Functions to Create */
//CAN ISR Interrupt
//CAN transmit
//Timer ISR
//init_Timer function
//Check flags
//init ADC
//main loop

void CAN_ISR(void) {
	//TODO
}

void CAN_TRANS(void) {
	//TODO
}

void Timer_ISR(void) {
	//TODO
}

void init_Timer(void) {
	//TODO
}

void init_ADC(void) {
	//TODO
}

void Check_flags(void) {
	//TODO

}

void setup_timer_100Hz(void) {
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
    OCR0A = 39;                /* Set the match register to 195 (maximum
                                 * period), 3.90625 kHz / 39 = 100.16 Hz */
}

ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    */
    gTimerFlag |= _BV(UPDATE_STATUS);
}

/* Interrupt service routine for pin change interrupts on PCINT[16:23] */
ISR(PCINT2_vect) {
	/* Any of the listed pins could've been triggered. Check if PD1 was pulled low. */

	if (bit_is_set(PIND, BRAKE_GATE_PIN)) {       // if the BRAKE_GATE_PIN is high
		LED1_PORT |= _BV(LED1);                     // turn LED1 on
		}
	else {                                        // if it the BRAKE_GATE_PIN is low
		LED1_PORT &= ~_BV(LED1);                    // turn LED2 off
	}
}


int main(void) {
//init functions
//set pins as outputs/inputs
//CAN listen


	//initialize pin change interrupts on PD1
	sei();
	PCICR |= _BV(PCIE2);          /*Enable 2nd bank of pin change interrupts */
	PCMSK2 |= _BV(PCINT17);       /* Enable PCINT17 to trigger interrupts */

	while(1) {
	//timer if statement
	//do stuff
	///TODO
	}
}
