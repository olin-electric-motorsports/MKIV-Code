/*
 * OEM LTC6811 library header
 *
 * Contains function declarations for abstracting out the LTC6811 interface
 * Much of this is pulled from the Linear Technology Arduino example code found
 * in the LTSketchbook, available online.
 * The sketch in question is the DC2259_test.ino sketch.
 *
 * @author Alex Hoppe '19
 * @author Vienna Scheyer '21
 */

#ifndef LTC6811_H
#define LTC6811_H

#include <util/delay.h>
#include <avr/pgmspace.h>
#include "spi.h"
#include "ltc6811_defs.h"

#define TOTAL_IC    1
#define NUM_CELLS   12 //TODO shouldn't this be 10?

// Cell voltages read back from the peripheral board
// LSbit = 0.0001V, i.e. 65535 -> 6.5535V
// |  cell_voltages[0][0]| cell_voltages[0][1] |  cell_voltages[0][2]|    .....     |  cell_voltages[0][11]|  cell_voltages[1][0] | cell_voltages[1][1]|  .....   |
// |---------------------|---------------------|---------------------|--------------|----------------------|----------------------|--------------------|----------|
// |IC1 Cell 1           |IC1 Cell 2           |IC1 Cell 3           |    .....     |  IC1 Cell 12         |IC2 Cell 1            |IC2 Cell 2          | .....    |
uint16_t raw_cell_voltages[TOTAL_IC][NUM_CELLS];

void ltc6811_init(volatile uint8_t* cs_port, uint8_t cs_pin);
// Half-duplex SPI write, then read
void SPI_write_then_read(uint8_t *tx_data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len);

//Generic wakeup command to wake the ltc6813 from sleep
void wakeup_sleep(uint8_t total_ic);

void ltc6811_adcv(
        uint8_t MD, //ADC Mode
        uint8_t DCP, //Discharge Permit
        uint8_t CH //Cell Channels to be measured
        );

//This function will block operation until the ADC has finished it's conversion
uint32_t ltc6811_pollAdc(void);

/*
 * Reads and parses the ltc6811 cell voltage registers.
 */
int8_t ltc6811_rdcv(
        uint8_t total_ic, // the number of ICs in the system
        uint16_t cell_codes[][NUM_CELLS] // Array of the parsed cell codes
        //TODO should this be [TOTAL_IC][NUM_CELLS]
        );

void _ltc6811_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                        uint8_t total_ic, //the number of ICs in the daisy chain
                        uint8_t *data //An array of the unparsed cell codes
                    );

void mux_disable(uint8_t total_ic, uint8_t i2c_adress);

void mux_set_channel(uint8_t total_ic, uint8_t i2c_address, uint8_t channel);

uint16_t pec15_calc(uint8_t len,    //Number of bytes that will be used to calculate a PEC
                    uint8_t *data); //Array of data that will be used to calculate a PEC


#endif

/* What do we still have to do?
 *
 * In general:
 *  - Wakeup Sleep
 *
 * For voltage sensing:
 *  - adcv (start cell conversion)
 *  - pollADC (Wait for cell conversion to be done)
 *  - rdcv (read back cell voltages)
 *
 * For temp sensing:
 *  - I2C enable specific channels
 *  - I2C disable mux
 *  - adax (start aux voltage)
 *  - pollADC
 *  -
 */
