/*
 * Tutorial 7: UART and Analog Input
 * 
 * This tutorial shows to do UART output, with ADC input as the data source.
 * To read the UART data, install picocom with sudo apt install picocom.
 * Run it with: picocom /dev/ttyUSB0. To escape, hit ctl-a ctl-x.
 *
 * You may have to give yourself permissions to use /dev/ttyUSB0 with
 * sudo adduser $USER dialout 
 * and then logging out and back in.
 * 
 * @author Alex Hoppe
 */


#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include "log_uart.h"


// To hold onto a string for UART, you need a character buffer to put it into
char uart_buf[64];

// There's some setup we need to do for the ADC
void ADC_init(void) {
    /* Get the Analog to Digital Converter started (ADC)
     * Set ADC Enable, and set AD Prescaler to 0x101
     * Divides clock frequency by 32 for AD clock */ 
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);

    //Enable interal reference voltage
    ADCSRB &= _BV(AREFEN);

    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);
}

// This function starts a conversion and then blocks program execution
// until the ADC returns a 10-bit value (0-1023)
uint16_t ADC_read(uint8_t adc_channel) { 
    
    // Set the ADC channel
    ADMUX &= 0b11100000;
    ADMUX |= adc_channel;
    
    //Trigger a conversion by setting the AD Start Conversion bit
    ADCSRA |= _BV(ADSC);

    //Block program until ADC is done
    loop_until_bit_is_clear(ADCSRA, ADSC);

    return (uint16_t) ADC;
}


int main(void) {

    /* Enable the peripherals we want to use */
    sei();
    LOG_init();
    ADC_init();
    
    /* This is where we'll store the voltage measurement */
    uint16_t analog_voltage = 0;

    while(1) {

        /* Read from Pinout 2, PB7, on ADC4 */
        analog_voltage = ADC_read(4);

        /* Use sprintf to copy some text into our UART buffer
         * and also render the voltage number as text. */
        sprintf(uart_buf, "Voltage is %d", analog_voltage);
        
        /* Write the UART buffer to the UART peripheral */
        LOG_println(uart_buf, strlen(uart_buf));

        _delay_ms(500);
    }
}
