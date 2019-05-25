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

#define SS_BSPD                 PB7
#define SS_TSMS                 PB6
#define SS_LEFT_ESTOP           PB5
#define SS_GLVMS                PD0
#define Brakelight              PD1

/* LEDs */
#define LED1                    PD6
#define LED2                    PD7
#define LED3                    PB2
#define EXT_LED1                PB3
#define EXT_LED2                PB4

/* CAN Message Positions */
#define CAN_BRAKE_ANALOG_MSB	0
#define CAN_BRAKE_ANALOG_LSB	1
#define CAN_BRAKE_GATE		2
#define CAN_BSPD		3
#define CAN_TSMS		4
#define CAN_ESTOP		5
#define CAN_GLVMS		6

/* CAN Message Objects */
#define MOB_PANIC		0
#define MOB_BRAKELIGHT		1

/* Fault Codes */
#define FAULT_CODE_BSPD 	0x10


#define UPDATE_STATUS         0

uint8_t gStatusMessage[7] = {0xFF, 0, 0, 0, 0, 0, 0};

volatile uint8_t gTimerFlag = 0x01;

void initTimer(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);
    OCR0A = 0x27;
}

ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    */
    gTimerFlag =  _BV(UPDATE_STATUS);
}

/* Interrupt service routine for pin change interrupts on PCINT[0:7] */
ISR(PCINT2_vect) {

    gStatusMessage[CAN_GLVMS] = (bit_is_set(PIND, SS_GLVMS) ? 0x00 : 0xFF);
    gStatusMessage[CAN_BRAKE_GATE] = (bit_is_set(PIND, Brakelight) ? 0xFF : 0x00);

}

ISR(PCINT0_vect){
    gStatusMessage[CAN_BSPD] = (bit_is_set(PINB, SS_BSPD) ? 0x00 : 0xFF);
    gStatusMessage[CAN_TSMS] = (bit_is_set(PINB, SS_TSMS) ? 0x00 : 0xFF);
    gStatusMessage[CAN_ESTOP] = (bit_is_set(PINB, SS_LEFT_ESTOP) ? 0x00 : 0xFF);
}

void initADC(void) {
    //Get the Analog to Digital Converter started (ADC)
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);

    //Enable interal reference voltage
    ADCSRB &= _BV(AREFEN);

    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);

    //Reads by default from ADC0 (pin 11)
    //This line is redundant. The timer
    ADMUX |= _BV(0x00);
}

void readBrakePressure(void){
	/*
	 * Read values from the brake pressure sensor
	 * to send over CAN to the rest of the vehicle
	 */

	ADMUX = _BV(REFS0);
	ADMUX |= 10; // ADC10
	ADCSRA |= _BV(ADSC);
	loop_until_bit_is_clear(ADCSRA,ADSC);
	uint16_t brakeValue = ADC; 

	gStatusMessage[CAN_BRAKE_ANALOG_MSB] = (uint8_t) (brakeValue >> 2);
	gStatusMessage[CAN_BRAKE_ANALOG_LSB] = (uint8_t) brakeValue;

}

int main(void) {

    //Set up LEDs
    DDRD |= _BV(LED1) | _BV(LED2);
    DDRB |= _BV(LED3) | _BV(EXT_LED1) | _BV(EXT_LED2);
    DDRC |= _BV(PC0); //Cooling pump LSD

    //Set up Shutdown Sense Pins
    DDRD &= ~_BV(SS_GLVMS);
    DDRB &= ~_BV(SS_BSPD) & ~_BV(SS_TSMS) & ~_BV(SS_LEFT_ESTOP);

  	//initialize pin change interrupts on PD1
  	sei();
    initTimer();

    PCICR |= _BV(PCIE0) | _BV(PCIE2);
    PCMSK0 |= _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);
    PCMSK2 |= _BV(PCINT16) | _BV(PCINT17);

    initADC();
    CAN_init(CAN_ENABLED);

    // CAN_wait_on_receive(0, CAN_ID_BRAKE_PRESSURE, CAN_LEN_BRAKE_PRESSURE, 0xFF);

    while(1) {
        // PORTD ^= _BV(LED2);
        PORTB ^= _BV(EXT_LED2);
        if(bit_is_set(gTimerFlag,UPDATE_STATUS)){
            PORTD ^= _BV(EXT_LED2);
            gTimerFlag &= ~_BV(UPDATE_STATUS);
            readBrakePressure(); 
            CAN_transmit(0, CAN_ID_BRAKE_LIGHT, CAN_LEN_BRAKE_LIGHT, gStatusMessage);
        }
    }
}
