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
#include "ltc6811.h"

volatile uint8_t FLAGS = 0x00;

#define LED1_PIN    PD5
#define LED2_PIN    PD6
#define LED3_PIN    PD7

#define LED_PORT    PORTD
#define LED_DDR     DDRD

#define RELAY_PIN   PC7
#define RELAY_PORT  PORTC
#define RELAY_DDR   DDRC

#define TRANSMIT_STATUS     0b00000001

#define UNDER_VOLTAGE       0b00000100
#define OVER_VOLTAGE        0b00001000
#define SOFT_OVER_VOLTAGE   0b00010000

volatile uint8_t gFlag = 0;
uint8_t gStatusMessage[7];

uint8_t gCounterTransmit = 0;
const uint8_t transmit_match = 10;

char uart_buf[64];

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 42000;//35900; // Over voltage threshold ADC Code
const uint16_t UV_THRESHOLD = 25000; //20000;// Under voltage threshold ADC Code

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

// READ VOLTAGES /////////////////////////////////////////////////////////////////////////////////////////
int8_t read_all_voltages(void) // Start Cell ADC Measurement
{
    int8_t error = 0;

    // Set up ADC
    wakeup_sleep(TOTAL_IC);
    o_ltc6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT); //TODO write o_ltc6811_adcv()
    o_ltc6811_pollAdc(); // TODO write o_ltc6811_pollAdc()

    error = o_ltc6811_rdcv(0,TOTAL_IC, raw_cell_voltages); //Parse ADC measurements //TODO write o_ltc6811_rdcv

    // 12 16-bit ints
    for (int i = 0; i < TOTAL_IC; i++) {
        char tmp_msg[108] = "";
        sprintf(tmp_msg, "v%d,%u,%u,%u,%u,%u,"
                         "%u,%u,%u,%u,"
                         "%u,%u",
                          i,
                          error,
                          raw_cell_voltages[i][0],
                          raw_cell_voltages[i][1],
                          raw_cell_voltages[i][2],
                          raw_cell_voltages[i][3],
                          raw_cell_voltages[i][4],
                          raw_cell_voltages[i][6],
                          raw_cell_voltages[i][7],
                          raw_cell_voltages[i][8],
                          raw_cell_voltages[i][9],
                          raw_cell_voltages[i][10]);

        LOG_println(tmp_msg, strlen(tmp_msg));
    }

    // Do value checking
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
        for (uint8_t cell = 0; cell < CELL_CHANNELS; cell++) {

            uint16_t cell_value = raw_cell_voltages[ic][cell];

            if (cell_value > OV_THRESHOLD) {
                FLAGS |= OVER_VOLTAGE;
            } else if (cell_value > SOFT_OV_THRESHOLD) {
                FLAGS |= SOFT_OVER_VOLTAGE;

                // :( sad side-effects
                if (FLAGS & AIRS_CLOSED) {
                    // IC and Cell are 1-indexed for discharge
                    enable_discharge(ic + 1, cell + 1);
                }
            } else if (cell_value < UV_THRESHOLD) {
                FLAGS |= UNDER_VOLTAGE;
            } else {
                FLAGS &= ~(OVER_VOLTAGE | UNDER_VOLTAGE | SOFT_OVER_VOLTAGE);
            }
        }
    }

    return error;
}


int main (void) {

    /* Set the data direction register so the led pin is output */
    LED_DDR |= _BV(LED1_PIN) | _BV(LED2_PIN) | _BV(LED3_PIN);

    sei();
    /* Initialize CAN */
    CAN_init(CAN_ENABLED);

    setup_timer_100Hz();

    LOG_init();

    /* Write the relay on for testing as of now */
    RELAY_DDR |= _BV(RELAY_PIN);
    RELAY_PORT |= _BV(RELAY_PIN);

    // Test out our wakeup code
    // ltc6811_init(&PORTB, PB6);
    SPI_init(SPI_FOSC_DIV_4, SPI_MODE_1_1, &PORTB, PB6);

    _delay_us(1000);


    while (1) {
        // LED_PORT ^= _BV(LED3_PIN);
        // Transmit status task
        if (gFlag & TRANSMIT_STATUS) {
            gFlag &= ~TRANSMIT_STATUS;

            // Test LTC6820
            // wakeup_sleep(TOTAL_IC);
            uint8_t dat;
            SPI_start();
            SPI_transfer(0xFF, &dat);
            // _delay_us(100);
            SPI_end();


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
