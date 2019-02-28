/* Tutorial 1 helper file
 *
 * Helper file to define blink_once function
 *
 * @author Alex Hoppe '19
 */

#include <avr/io.h>
#include <util/delay.h>

void blink_once(int pin) {      
    PORTB |= _BV(pin);  // Turn one LED on
    _delay_ms(200);         // wait 200 ms
    PORTB |= _BV(pin);  // Turn LED off
    _delay_ms(200);         // wait 200 ms
}


