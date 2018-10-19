/*----- Includes -----*/
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


int main (void) {

    //Data Direction Register X
    //Decides input or output
	DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2);
	PORTB |= _BV(PB1);

	while(1) {
        //LEDs flip once per second
		PORTB ^= _BV(PB0);
		PORTB ^= _BV(PB1);
		PORTB ^= _BV(PB2);			
		
        _delay_ms(1000);

	}
}