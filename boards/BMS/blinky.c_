/*----- Includes -----*/
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


int main (void) {

    //Data Direction Register X
    //Decides input or output
	DDRD |= _BV(PD5) | _BV(PD6) | _BV(PD7);
    
    DDRC |= _BV(PC7);
    PORTC |= _BV(PC7);
    
    uint8_t ctr = 1;

	while(1) {
        //LEDs flip once per second
		// PORTD ^= _BV(PD5);
		// PORTD ^= _BV(PD6);
		// PORTD ^= _BV(PD7);			
		
        PORTD = ctr << 5;
		ctr = ctr + 1 % 8;

		PORTC ^= _BV(PC7);
        _delay_ms(1000);

	}
}
