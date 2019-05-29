/*----- Includes -----*/
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"

/*----- MACROS -----*/
#define LED1        PC4
#define LED2        PC5
#define LED_PORT    PORTC

#define RJ45_LED1    PB0
#define RJ45_LED2    PB1
#define RJ45_LED_PORT    PORTB

#define PRECHARGE_CTRL        PB4
#define PRECHARGE_PORT        PORTB

#define MOB_AIR_CRIT        0
#define MOB_AIR_SENSE        1

/*----- gFlag -----*/
#define UPDATE_STATUS       0
#define FLAG_AIRPLUS_AUX    1
#define FLAG_AIRMINUS_AUX   2


volatile uint8_t gFlag = 0x00; // Global Flag
volatile uint8_t LEDtimer = 0x00;

uint8_t msg[8] = {0,0,0,0,0,0,0,0};

ISR(TIMER0_COMPA_vect) {
   /*
   Timer/Counter0 compare match A
   If the clock frequency is 4MHz then this is called 16 times per second
   MATH: (4MHz/1024)/255 = ~16
   */

    gFlag |= _BV(UPDATE_STATUS);
    LEDtimer++;
    if(LEDtimer > 100){
        LEDtimer = 0;
        LED_PORT ^= _BV(LED2);
    }
}

/*
ISR(CAN_INT_vect) {
   CANPAGE = (MOB_AIR_CRIT << MOBNB0); // Switch to MOb 0, the one we're listening on.
   if(bit_is_set(CANSTMOB, RXOK)) {
       volatile uint8_t msg = CANMSG;      //grab the first byte of the CAN message

       if(msg == 0xFF) {
           PORTB |= _BV(LED2_PIN);
       } else {
           PORTB &= ~_BV(LED2_PIN);
       }

       m//Setup to Receive Again
       CANSTMOB = 0x00;
       CAN_wait_on_receive(0, CAN_ID_TUTORIAL6, CAN_LEN_TUTORIAL6, 0xFF);
   }
}
*/

ISR(PCINT1_vect) {
   if(bit_is_set(PINC,PC6)){
     gFlag |= _BV(FLAG_AIRPLUS_AUX);
    } else {
     gFlag &= ~_BV(FLAG_AIRPLUS_AUX);
    }
    if(bit_is_set(PINC,PC7)){
     gFlag |= _BV(FLAG_AIRMINUS_AUX);
    } else {
     gFlag &= ~_BV(FLAG_AIRMINUS_AUX);
    }

}


void initTimer(void) {
   TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
   TCCR0B = 0x05;          // clkio/1024 prescaler
   TIMSK0 |= _BV(OCIE0A);  // Every 1024 cycles, OCR0A increments
   OCR0A = 0x27; //dec 39  // until 0xff, 255, which then calls for
                           // the TIMER0_COMPA_vect interrupt
                // currently running at 100Hz
}


int main (void) {
   initTimer();
   sei(); //Inititiates interrupts for the ATMega
   CAN_init(CAN_ENABLED);
   //CAN_wait_on_receive(0, CAN_ID_TUTORIAL6, CAN_LEN_TUTORIAL6, 0xFF);

   // Enable interrupt
   PCICR |= _BV(PCIE0) | _BV(PCIE1);


   //Sets these pins at outputs
   DDRB |= _BV(PRECHARGE_CTRL) | _BV(RJ45_LED1) | _BV(RJ45_LED2);
   DDRC |= _BV(LED1) | _BV(LED2);


   while(1) {
       if(bit_is_set(gFlag,UPDATE_STATUS)){
           gFlag &= ~_BV(UPDATE_STATUS);

           // if(bit_is_set(gFlag,FLAG_AIRPLUS_AUX)){
           //     LED_PORT |=  _BV(LED1);
	           msg[0] = 0x0F;
	           CAN_transmit(MOB_AIR_CRIT,CAN_ID_AIR_CONTROL_CRITICAL,CAN_LEN_AIR_CONTROL_CRITICAL,msg);
	           _delay_ms(2000);
	           msg[0] = 0xFF;
						 CAN_transmit(MOB_AIR_CRIT,CAN_ID_AIR_CONTROL_CRITICAL,CAN_LEN_AIR_CONTROL_CRITICAL,msg);
	           _delay_ms(4000);
						 msg[0] = 0x00;
	           CAN_transmit(MOB_AIR_CRIT,CAN_ID_AIR_CONTROL_CRITICAL,CAN_LEN_AIR_CONTROL_CRITICAL,msg);
	           _delay_ms(5000);

           // } else {
           //     msg[0] = 0x00;
           //     LED_PORT &= ~_BV(LED1);
           // }




       }
   }
}
