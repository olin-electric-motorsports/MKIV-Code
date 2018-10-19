/*----- Includes -----*/
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/*----- MACROS -----*/
// Macros are essentially variables for variables
// Everywhere it says LED1, the compiler reads PB0
#define LED1        PB0
#define LED2        PB1
#define LED3        PB2
#define LED_PORT    PORTB

/*----- Flags -----*/
// for gFlag
// gFlag is essentially a means of making code more readable
// which allows the coder to store up to 8 (indexed at 0)
// state variables.
#define UPDATE_STATUS           0


/*----- Global Variables -----*/
volatile uint8_t gFlag = 0x00;  // Global Flag

/*----- Timer Counters ----- */
uint8_t clock_prescale = 0x00;

/*----- Functions to Work on -----*/
// Shutdown sense
// ADC (Analog Digital Converter)
// Printing over UART
// THE DATASHEET IS YOUR BEST FRIEND
// hint:Look at other code on the car...



ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    If the clock frequency is 4MHz then this is called 16 times per second
    MATH: (4MHz/1024)/255 = ~16
    */

    clock_prescale ++;
    if(clock_prescale>15) {
        gFlag |= _BV(UPDATE_STATUS);
        clock_prescale = 0;
    }
}

/*----- Functions -----*/
void initTimer(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);  // Every 1024 cycles, OCR0A increments 
    OCR0A = 0xFF;           // until 0xff, 255, which then calls for
                            // the TIMER0_COMPA_vect interrupt
}


int main (void) {
    initTimer();
    sei(); //Inititiates interrupts for the ATMega

    // Set interrupt registers
    PCICR |= _BV(PCIE0);

    //Data Direction Register x
    //Sets these pins at outputs
	DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2);
	
    //Turns on LED2
    LED_PORT |= _BV(LED2);

	while(1) {
		if(bit_is_set(gFlag,UPDATE_STATUS)){
			gFlag &= ~_BV(UPDATE_STATUS);
            //enjoy the small light show
			LED_PORT ^= _BV(LED1);
			LED_PORT ^= _BV(LED3);
			LED_PORT ^= _BV(LED2);			
		}
	}
}