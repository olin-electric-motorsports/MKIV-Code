/*
 * OEM LTC6811 library
 */

#ifndef LTC6811_H
#define LTC6811_H

#include <util/delay.h>
#include "spi.h"


#define TOTAL_IC    1
#define NUM_CELLS   12

// Cell voltages read back from the peripheral board
// LSbit = 0.0001V, i.e. 65535 -> 6.5535V
// |  cell_voltages[0][0]| cell_voltages[0][1] |  cell_voltages[0][2]|    .....     |  cell_voltages[0][11]|  cell_voltages[1][0] | cell_voltages[1][1]|  .....   |
// |---------------------|---------------------|---------------------|--------------|----------------------|----------------------|--------------------|----------|
// |IC1 Cell 1           |IC1 Cell 2           |IC1 Cell 3           |    .....     |  IC1 Cell 12         |IC2 Cell 1            |IC2 Cell 2          | .....    |
uint16_t raw_cell_voltages[TOTAL_IC][NUM_CELLS];

void ltc6811_init(volatile uint8_t* cs_port, uint8_t cs_pin) {
    SPI_init(SPI_FOSC_DIV_4, SPI_MODE_1_1, cs_port, cs_pin);
}


//Generic wakeup command to wake the ltc6813 from sleep
void wakeup_sleep(uint8_t total_ic) {
    for (int i =0; i<TOTAL_IC+1; i++) {
        SPI_start(); // chip select
        _delay_us(300); // Guarantees the ltc6813 will be in standby
        SPI_end();
    }
}

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
