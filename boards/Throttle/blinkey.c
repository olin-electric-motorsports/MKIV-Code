/*
  Code for the OEM MKIV Throttle Board
  Author: @awenstrup
*/

//Import Statements
#include <avr/io.h>
#include <util/delay.h>

//Macro Definitinos

//Pin and Port Macros

#define PLED1 PC6
#define PLED1_PORT PORTC
#define PLED2 PB3
#define PLED2_PORT PORTB
#define PLED3 PB4
#define PLED3_PORT PORTB

//********************Global variables***************

//********************Functions*************************

//Initializers
void initTimer(void) {
    // Set up 8-bit timer in CTC mode
    TCCR0A = _BV(WGM01);

    // Set clock prescaler to (1/256) - page 89
    TCCR0B = 0b100;

    // Enable Match A interupts - page 90
    TIMSK0 |= _BV(OCIE0A);

    //Makes the timer reset everytime it hits 255
    // - page 90
    OCR0A = 0xFF;
}

void setPledOut(void) {
    DDRC |= _BV(PLED1);
    DDRB |= _BV(PLED2);
    DDRB |= _BV(PLED3);
}

void setLightsHigh(void) {
    PLED1_PORT |= _BV(PLED1);
    PLED2_PORT |= _BV(PLED2);
    PLED3_PORT |= _BV(PLED3);
}

void setLightsLow(void) {
    PLED1_PORT &= ~_BV(PLED1);
    PLED2_PORT &= ~_BV(PLED2);
    PLED3_PORT &= ~_BV(PLED3);
}

int main(void) {
  initTimer();
  setPledOut();

  while(1) {
      setLightsHigh();
      _delay_ms(500);
      setLightsLow();
      _delay_ms(500);
  }
}
