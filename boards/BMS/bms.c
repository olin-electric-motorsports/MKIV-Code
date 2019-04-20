/*
 * BMS DEMO TX
 *
 * CAN transmitting code from Tutorial 6. Transmits a CAN message on 0x25 with a 0
 * or 0xFF to turn on or off an LED based on a button state.
 *
 * @author Alex Hoppe '19
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "can_api.h"
#include "log_uart.h"

#define LED1_PIN    PD5
#define LED2_PIN    PD6
#define LED3_PIN    PD7

#define LED_PORT    PORTD
#define LED_DDR     DDRD

#define RELAY_PIN   PC7
#define RELAY_PORT  PORTC
#define RELAY_DDR   DDRC

#define TRANSMIT_STATUS 0b00000001

volatile uint8_t gFlag = 0;
uint8_t gStatusMessage[7];

uint8_t gCounterTransmit = 0;
const uint8_t transmit_match = 10;

char uart_buf[64];

// Scale down transmit task
ISR(TIMER0_COMPA_vect) {
    if (gCounterTransmit == transmit_match) {
        gCounterTransmit = 0;

        gFlag |= TRANSMIT_STATUS;
        LED_PORT ^= _BV(LED1_PIN);

    } else {
        gCounterTransmit++;
    }
}

void setup_timer_100Hz(void) {
    /* Set up TC0 in ctc mode, with OCR0A interrupt enabled and at 2 Hz */
    TCCR0A |= _BV(WGM01);       /* Set WGM[2:0] to 0b010: CTC mode (WGM2 is
                                 * TCCR0B) */
    TCCR0B |= _BV(CS02) | _BV(CS00);    /* Set up prescaler for TC0 to clk_io
                                         * divided by 1024:
                                         * 4MHz/1024 = 3.90625 kHz
                                         * CS0[2:0] = 0b101
                                         * (table 15-10) in datasheet */
    TIMSK0 |= _BV(OCIE0A);      /* Set interrupt enable for output channel A
                                 * interrupt on compare match */
    OCR0A = 39;                /* Set the match register to 195 (maximum
                                 * period), 3.90625 kHz / 39 = 100.16 Hz */
}

int main (void) {

    /* Set the data direction register so the led pin is output */
    LED_DDR |= _BV(LED1_PIN) | _BV(LED2_PIN) | _BV(LED3_PIN);

    sei();
    /* Initialize CAN */
    CAN_init(CAN_ENABLED);

    setup_timer_100Hz();

    LOG_init();

    while (1) {
        LED_PORT ^= _BV(LED3_PIN);
        // Transmit status task
        if (gFlag & TRANSMIT_STATUS) {
            gFlag &= ~TRANSMIT_STATUS;

            LED_PORT ^= _BV(LED2_PIN);

            // Actually build up a CAN message
            //Report relay status
            gStatusMessage[0] = bit_is_set(RELAY_PORT, RELAY_PIN) ? 0xFF : 0;
            // TODO: temperature
            gStatusMessage[1] = 100;
            // TODO: state of charge
            gStatusMessage[2] = 12;
            // Report BMS ok for BMS light
            gStatusMessage[3] = 0xFF;
            // Report regen status
            gStatusMessage[4] = 0;
            // Report current limiting
            gStatusMessage[5] = 0;
            // Report cell balancing status
            gStatusMessage[6] = 0;

            CAN_transmit(0, CAN_ID_BMS_CORE, CAN_LEN_BMS_CORE, gStatusMessage);

        }

    }
}
