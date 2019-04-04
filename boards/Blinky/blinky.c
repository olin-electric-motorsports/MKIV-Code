/*----- Includes -----*/
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


int main (void) {

    //Data Direction Register X
    //Decides input or output
	DDRB |= _BV(PB5) | _BV(PB6) | _BV(PB7);
	PORTB |= _BV(PB5);

	while(1) {
        //LEDs flip once per second
		PORTB ^= _BV(PB5);
		PORTB ^= _BV(PB6);
		PORTB ^= _BV(PB7);			
		
        _delay_ms(1000);

	}
}