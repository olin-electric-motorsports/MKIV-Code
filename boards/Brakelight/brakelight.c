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
#define GLOBAL_SHUTDOWN         0
// for gTimerFlag
//TODO

#define UPDATE_STATUS  0b00000001

// uint8_t msg[1];

uint8_t gStatusMessage[7];

volatile uint8_t gFlag = 0;

/* pins */

#define BRAKE_GATE_PIN          PD1
#define BRAKE_GATE_PIN_BANK     PIND
#define BRAKE_LIGHT_DDR         DDRD
#define BSPD_PIN                PB7
#define BSPD_PIN_BANK           PINB
#define BSPD_DDR                DDRB
#define TSMS_PIN                PB6
#define TSMS_PIN_BANK           PINB
#define TSMS_DDR                DDRB
#define E_STOP_PIN              PB5
#define E_STOP_PIN_BANK         PINB
#define E_STOP_DDR              DDRB

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
#define CAN_BRAKE_GATE          2
#define CAN_BSPD                3
#define CAN_TSMS            	4
#define CAN_ESTOP	            5
#define CAN_GLVMS		        6

/* FLAGS */
#define FLAG_BRAKE	            0


/* MOBs */
#define MOB_BRAKELIGHT		    0


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
    gFlag |= UPDATE_STATUS;
}

/* Interrupt service routine for pin change interrupts on PCINT[0:7] */
ISR(PCINT0_vect) {
    /* If BSPD pin is reading high, send CAN panic */
    gStatusMessage[CAN_BSPD] = (bit_is_clear(BSPD_PIN_BANK, BSPD_PIN) ? 0xFF : 0);
    gStatusMessage[CAN_TSMS] = (bit_is_clear(TSMS_PIN_BANK, TSMS_PIN) ? 0xFF : 0);
    gStatusMessage[CAN_ESTOP] = (bit_is_clear(E_STOP_PIN_BANK, E_STOP_PIN) ? 0xFF : 0);
}

ISR(PCINT1_vect){
    //TODO
}

/* Interrupt service routine for pin change interrupts on PCINT[16:23] */
ISR(PCINT2_vect) {
  	/* Any of the listed pins could've been triggered. Check if PD1 was pulled low. */

  	if (bit_is_set(BRAKE_GATE_PIN_BANK, BRAKE_GATE_PIN)) {       // if the BRAKE_GATE_PIN is high
    		LED1_PORT |= _BV(LED1);                     // turn LED1 on
    		}
    	else {                                        // if it the BRAKE_GATE_PIN is low
    		LED1_PORT &= ~_BV(LED1);                    // turn LED1 off
	}
    gStatusMessage[CAN_GLVMS] = (bit_is_clear(E_STOP_PIN_BANK, E_STOP_PIN_BANK) ? 0xFF : 0);
    gStatusMessage[CAN_BRAKE_GATE] = (bit_is_clear(BRAKE_GATE_PIN_BANK, BRAKE_GATE_PIN) ? 0xFF : 0);

}

int main(void) {
//init functions
//set pins as outputs/inputs
//CAN listen

    /* set data direction register so that brake light pin is the output  */
    BRAKE_LIGHT_DDR |= _BV(LED1) | _BV(LED2) | _BV(LED3);

  	//initialize pin change interrupts on PD1
  	sei();
    PCICR |= _BV(PCIE2);          /*Enable 2nd bank of pin change interrupts */
  	PCMSK2 |= _BV(PCINT17);       /* Enable PCINT17 to trigger interrupts */

    PCICR |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT7);

    CAN_init(CAN_ENABLED);

    setup_timer_100Hz();

    while(1) {
        //timer if statement
        if (gFlag & UPDATE_STATUS) {
            gFlag &= ~UPDATE_STATUS;      // reset flag state

            // CAN_BRAKE_ANALOG_MSB pos 0
            gStatusMessage[0] = 0;
            // CAN_BRAKE_ANALOG_LSB pos 1
            gStatusMessage[1] = 0;

            CAN_transmit(0, CAN_ID_BRAKE_LIGHT, CAN_LEN_BRAKE_LIGHT, gStatusMessage);
        }
    }
}
