/*
 *  2019 BMS code
 *
 * @author Alex Hoppe '19
 * @author Vienna Scheyer '21
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "can_api.h"
#include "log_uart.h"
#include "ltc6811.h"

<<<<<<< HEAD
volatile uint8_t FLAGS = 0x00;

=======
// BMS core hardware defines
>>>>>>> 0e9995555ebd041b47fb1f48efa8a00495e8fbd0
#define LED1_PIN    PD5
#define LED2_PIN    PD6
#define LED3_PIN    PD7

#define LED_PORT    PORTD
#define LED_DDR     DDRD

#define RELAY_PIN   PC7
#define RELAY_PORT  PORTC
#define RELAY_DDR   DDRC

<<<<<<< HEAD
#define TRANSMIT_STATUS     0b00000001

#define UNDER_VOLTAGE       0b00000100
#define OVER_VOLTAGE        0b00001000
#define SOFT_OVER_VOLTAGE   0b00010000
=======
// gFlag register pre-defines
#define TRANSMIT_STATUS     0b00000001
#define UNDER_VOLTAGE       0b00000100
#define OVER_VOLTAGE        0b00001000
#define UNDER_TEMP          0b00010000
#define OVER_TEMP           0b00100000
>>>>>>> 0e9995555ebd041b47fb1f48efa8a00495e8fbd0

volatile uint8_t gFlag = 0;
uint8_t gStatusMessage[7];

// Timer prescale vars and constants
uint8_t gCounterTransmit = 0;
const uint8_t transmit_match = 100;

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 42000;//35900; // Over voltage threshold ADC Code
const uint16_t UV_THRESHOLD = 25000; //20000;// Under voltage threshold ADC Code

// I2C MUX addresses from peripheral board
const uint8_t MUX1_ADDR = 0x90;
const uint8_t MUX2_ADDR = 0x92;
const uint8_t MUX3_ADDR = 0x94;

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
<<<<<<< HEAD
int8_t read_all_voltages(void) // Start Cell ADC Measurement
{
    int8_t error = 0;

    // Set up ADC
    wakeup_sleep(TOTAL_IC);
    o_ltc6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT); //TODO write o_ltc6811_adcv()
    o_ltc6811_pollAdc(); // TODO write o_ltc6811_pollAdc()

    error = o_ltc6811_rdcv(0,TOTAL_IC, raw_cell_voltages); //Parse ADC measurements //TODO write o_ltc6811_rdcv

    // 12 16-bit ints
=======
uint8_t read_all_voltages(void) // Start Cell ADC Measurement
{
    int8_t error = 0;

    wakeup_sleep(TOTAL_IC);
    // Start a cell conversion with dicharge disabled on all cells
    ltc6811_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);
    // wait until ADC is finished(SEE LTC 6804 Datasheet Pg. 24)
    _delay_ms(3);
    //Read back and parse out ADC measurements
    error = ltc6811_rdcv(TOTAL_IC, cell_voltages);

    // TODO very slow, comment out after inspection ****************************
    // UART out cell voltages as 10  16-bit ints
>>>>>>> 0e9995555ebd041b47fb1f48efa8a00495e8fbd0
    for (int i = 0; i < TOTAL_IC; i++) {
        char tmp_msg[108] = "";
        sprintf(tmp_msg, "v%d,%u,%u,%u,%u,%u,"
                         "%u,%u,%u,%u,"
                         "%u,%u",
                          i,
                          error,
<<<<<<< HEAD
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
=======
                          cell_voltages[i][0],
                          cell_voltages[i][1],
                          cell_voltages[i][2],
                          cell_voltages[i][3],
                          cell_voltages[i][4],
                          cell_voltages[i][6],
                          cell_voltages[i][7],
                          cell_voltages[i][8],
                          cell_voltages[i][9],
                          cell_voltages[i][10]);
>>>>>>> 0e9995555ebd041b47fb1f48efa8a00495e8fbd0

        LOG_println(tmp_msg, strlen(tmp_msg));
    }

    // Do value checking
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
<<<<<<< HEAD
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
=======
        for (uint8_t cell = 0; cell < NUM_CELLS; cell++) {

            //Skip cells that are 0, i.e. cell 6 and 12
            if ((cell == 5) || (cell == 11)) continue;

            uint16_t cell_value = cell_voltages[ic][cell];

            if (cell_value > OV_THRESHOLD) {
                gFlag |= OVER_VOLTAGE;
            } else if (cell_value < UV_THRESHOLD) {
                gFlag |= UNDER_VOLTAGE;
            } else {
                gFlag &= ~(OVER_VOLTAGE | UNDER_VOLTAGE);
>>>>>>> 0e9995555ebd041b47fb1f48efa8a00495e8fbd0
            }
        }
    }

    return error;
}

<<<<<<< HEAD
=======
/* On each peripheral board, this function sets the MUX channel over I2C, then
 * starts an ADC conversion for aux voltage 1, GPIO1.
 * Then it reads back the aux voltage data for all of the boards
 * Then it parses those into the temp_sensor_voltages array.
 */
uint8_t read_all_temperatures(void)
{
    uint8_t error = 0;

    const uint8_t MUX_CHANNELS = 8;

    // Disable all MUXes
    mux_disable(TOTAL_IC, MUX1_ADDR);
    mux_disable(TOTAL_IC, MUX2_ADDR);
    mux_disable(TOTAL_IC, MUX3_ADDR);

    // First four thermistors are in MUX3
    for (uint8_t i = 0; i < 4; i++) {

        mux_set_channel(TOTAL_IC, MUX3_ADDR, i);
        _delay_us(5);                             //Spec is 1600ns from stop cond.
        ltc6811_adax(MD_7KHZ_3KHZ, AUX_CH_GPIO1); //start ADC measurement for GPIO CH 1
        _delay_us(500);                           //only need to delay 500uS for GPIO1 conversion
        error = ltc6811_rdaux(0,TOTAL_IC, _aux_voltages); //Parse all ADC measurements back

        // Grab aux voltages into the temp voltages array
        for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
            //First four are out of order, 0123 are 2_2, 2_1, 1_2, and 1_1 respectively
            temp_sensor_voltages[ic][3-i] = _aux_voltages[ic][0]; //Store temperatures
        }
    }

    // Disable MUX3, now iterate through MUX2 (for modules 3-6)
    mux_disable(TOTAL_IC, MUX3_ADDR);

    for (uint8_t i = 0; i < MUX_CHANNELS; i++) {

        mux_set_channel(TOTAL_IC, MUX2_ADDR, i);
        _delay_us(5);                             //Spec is 1600ns from stop cond.
        ltc6811_adax(MD_7KHZ_3KHZ, AUX_CH_GPIO1); //start ADC measurement for GPIO CH 1
        _delay_us(500);                           //only need to delay 500uS for GPIO1 conversion
        error = ltc6811_rdaux(0, TOTAL_IC, _aux_voltages); //Parse all ADC measurements back

        // Grab aux voltages into the temp voltages array
        for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
            //Sensors are in reverse order
            temp_sensor_voltages[ic][MUX_CHANNELS-1-i] = _aux_voltages[ic][0]; //Store temperatures
        }
    }

    // Disable MUX2, now iterate through MUX1 (for modules 7-10)
    mux_disable(TOTAL_IC, MUX2_ADDR);

    for (uint8_t i = 0; i < MUX_CHANNELS; i++) {

        mux_set_channel(TOTAL_IC, MUX1_ADDR, i);
        _delay_us(5);                             //Spec is 1600ns from stop cond.
        ltc6811_adax(MD_7KHZ_3KHZ, AUX_CH_GPIO1); //start ADC measurement for GPIO CH 1
        _delay_us(500);                           //only need to delay 500uS for GPIO1 conversion
        error = ltc6811_rdaux(0, TOTAL_IC, _aux_voltages); //Parse all ADC measurements back

        // Grab aux voltages into the temp voltages array
        for (uint8_t ic = 0; ic < TOTAL_IC; ic++) {
            //Sensors are in reverse order, 0123 are 2_2, 2_1, 1_2, and 1_1 respectively
            temp_sensor_voltages[ic][MUX_CHANNELS-1-i] = _aux_voltages[ic][0]; //Store temperatures
        }
    }

    for( int ic = 0; ic < TOTAL_IC; ic++) {
        char temp_msg[128] = "";
        sprintf(temp_msg, "t%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", ic, error,
                        temp_sensor_voltages[ic][0],
                        temp_sensor_voltages[ic][1],
                        temp_sensor_voltages[ic][2],
                        temp_sensor_voltages[ic][3],
                        temp_sensor_voltages[ic][4],
                        temp_sensor_voltages[ic][5],
                        temp_sensor_voltages[ic][6],
                        temp_sensor_voltages[ic][7],
                        temp_sensor_voltages[ic][8],
                        temp_sensor_voltages[ic][9]
                        );

        LOG_println(temp_msg, strlen(temp_msg));
    }


    // if (over_temp == 0) {
    //     //upon successful execution clear flags
    //     FLAGS &= ~OVER_TEMP;
    // }

    return error;
}

>>>>>>> 0e9995555ebd041b47fb1f48efa8a00495e8fbd0

int main (void) {

    /* Set the data direction register so the led pin is output */
    LED_DDR |= _BV(LED1_PIN) | _BV(LED2_PIN) | _BV(LED3_PIN);

    sei();
    /* Initialize CAN */
    CAN_init(CAN_ENABLED);

    setup_timer_100Hz();

    LOG_init();

    SPI_init(SPI_FOSC_DIV_4, SPI_MODE_1_1, &PORTB, PB6);
    // MISO_iso pin is input because of SPI module. Write high for pull-up
    PORTB |= _BV(PB0);

    //Perform an initial check before we set the relay
    RELAY_DDR |= _BV(RELAY_PIN);
    read_all_voltages();
    //read_all_temperatures();
    // If we've got no faults
    if (!(gFlag & (OVER_TEMP | UNDER_TEMP | OVER_VOLTAGE | UNDER_VOLTAGE))) {
        RELAY_PORT |= _BV(RELAY_PIN);
    }

    while (1) {
        // LED_PORT ^= _BV(LED3_PIN);
        // Transmit status task
        if (gFlag & TRANSMIT_STATUS) {
            gFlag &= ~TRANSMIT_STATUS;

            read_all_voltages();
            read_all_temperatures();

            LED_PORT ^= _BV(LED2_PIN);

            // Actually build up a CAN message
            //Report relay status
            gStatusMessage[0] = bit_is_set(RELAY_PORT, RELAY_PIN) ? 0xFF : 0;
            // TODO: temperature
            gStatusMessage[1] = 100;
            // TODO: state of charge
            gStatusMessage[2] = 12;
            if ((gFlag & OVER_VOLTAGE) || (gFlag & UNDER_VOLTAGE) || (gFlag & OVER_TEMP) || (gFlag & UNDER_TEMP)) {
                RELAY_PORT &= ~_BV(RELAY_PIN);
                gStatusMessage[3] = 0xFF;
            } else {
                // Report BMS ok for BMS light
                gStatusMessage[3] = 0x00;
            }
            // Report regen status
            gStatusMessage[4] = 0;
            // Report current limiting
            gStatusMessage[5] = 0;
            // Report cell balancing status
            gStatusMessage[6] = 0;

            CAN_transmit(0, CAN_ID_BMS_CORE, CAN_LEN_BMS_CORE, gStatusMessage);

        }

        // if ((gFlag & OVER_VOLTAGE) || (gFlag & UNDER_VOLTAGE) || (gFlag & OVER_TEMP) || (gFlag & UNDER_TEMP)) {
        //     RELAY_PORT &= ~_BV(RELAY_PIN);
        //     gStatusMessage
        // }

    }
}
