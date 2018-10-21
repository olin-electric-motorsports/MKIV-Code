/*----- Includes -----*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"
#include <string.h>
#include <stdio.h>

/*---Macro Definitions---*/

//LED Programming Lights//
#define PORT_LED     PORTD
#define LED1        PD5
#define LED2        PD6
#define LED3        PD7


int main (void) {

		// Initialize UART peripheral
		LOG_init();

		//uint8_t var = 0;
		//char disp_string[64];
		//char static_msg[] = "static example msg";

    //Data Direction Register X
    //Decides input or output
	// DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2);
	// PORTB |= _BV(PB1);
	//Sets Pins as OUTPUT
		DDRD |= _BV(LED2) | _BV(LED3) | _BV(LED1);

	while(1) {
        //LEDs flip once per second
    // LOG_println(static_msg, strlen(static_msg));
		//sprintf(disp_string,"hello world");
		//LOG_println(disp_string, strlen(disp_string));
		LOG_init();
		char init_success_msg[] = "init good!";
		LOG_println(init_success_msg, strlen(init_success_msg));
		PORT_LED |= _BV(LED1);
		PORT_LED |= _BV(LED2);
		PORT_LED |= _BV(LED3);

		_delay_ms(10);

		PORT_LED ^= _BV(LED1);
		PORT_LED ^= _BV(LED2);
		PORT_LED ^= _BV(LED3);

    _delay_ms(10);

	}
}
