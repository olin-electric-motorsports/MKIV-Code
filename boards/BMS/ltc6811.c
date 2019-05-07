/*
 * LTC6811 library
 *
 * Contains functions for 
 *
 * @author Alex Hoppe '19
 *       Vienna Scheyer '21
 */

#include <util/delay.h>
#include "spi.h"
#include "ltc6811.h"

#define TOTAL_IC    1
#define NUM_CELLS   12 //TODO shouldn't this be 10?
#define MD_7KHZ_3KHZ 2 // GPIO selection for ADC conversion

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

void o_ltc6811_adcv(
        uint8_t MD, //ADC Mode
        uint8_t DCP, //Discharge Permit
        uint8_t CH //Cell Channels to be measured
        )
{
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    cmd[0] = md_bits + 0x02;
    md_bits = (MD & 0x01) << 7;
    cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;
    cmd_pec = pec15_calc(2, cmd); //TODO what is pec15_calc
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

}

//This function will block operation until the ADC has finished it's conversion
uint32_t o_ltc6811_pollAdc(void)
{
    uint32_t counter = 0;
    uint8_t finished = 0;
    uint8_t current_time = 0;
    uint8_t cmd[4];
    uint16_t cmd_pec;


    cmd[0] = 0x07;
    cmd[1] = 0x14;
    cmd_pec = pec15_calc(2, cmd); //TODO what is pec15_calc?
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);


    SPI_start(); //TODO is this in the SPI library now? Wrote function instead of PORTB &= _BV(PB4);
    spi_write_array(cmd,4);

    while ((counter<200000)&&(finished == 0)) //TODO why 200000?
    {
        current_time = spi_message(0xFF);
        if (current_time>0)
        {
            finished = 1;
        }
        else
        {
            counter = counter + 10;
        }
    }

    SPI_end();

    return counter;
}

/*
 * Reads and parses the ltc6811 cell voltage registers.
 */
int8_t o_ltc6811_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
        uint8_t total_ic, // the number of ICs in the system
        uint16_t cell_codes[][NUM_CELLS] // Array of the parsed cell codes
        //TODO should this be [TOTAL_IC][NUM_CELLS]
        )
{
    const uint8_t NUM_RX_BYT = 8;
    const uint8_t BYT_IN_REG = 6;
    const uint8_t CELL_IN_REG = 3; //TODO Should be new number?
    const uint8_t NUM_CV_REG = 4;

    uint8_t pec_error = 0;
    uint16_t parsed_cell;
    uint16_t received_pec;
    uint16_t data_pec;
    uint8_t data_counter = 0;
    uint8_t cell_data[NUM_RX_BYT * TOTAL_IC * sizeof(uint8_t)];

    // TODO: Why do we have a different case for the 0th reg?
    if (reg == 0)
    {
        // executes once for each of the ltc6811 cell voltage registers
        for (uint8_t cell_reg = 1; cell_reg < (NUM_CV_REG + 1); cell_reg++)
        {
            data_counter = 0;

            // Reads a single Cell voltage register
            o_ltc6811_rdcv_reg(cell_reg, total_ic, cell_data );

            // executes for every ltc6811 in the daisy chain
            for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)
            {
                // current_ic is used as the IC counter

                // This loop parses the read back data into cell voltages, it
                // loops once for each of the 3 cell voltage codes in the register
                for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)
                {

                    //Each cell code is received as two bytes and is combined to
                    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);
                    // create the parsed cell voltage code

                    cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
                    // Because cell voltage codes are two bytes the data counter
                    // must increment by two for each parsed cell code
                    data_counter = data_counter + 2;

                }

                //The received PEC for the current_ic is transmitted as the 7th and 8th
                received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //TODO << 8?

                //after the 6 cell voltage data bytes
                data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
                if (received_pec != data_pec)
                {
                    //The pec_error variable is simply set negative if any PEC errors
                    //are detected in the serial data
                    pec_error = -1;

                }

                //Because the transmitted PEC code is 2 bytes long the data_counter
                //must be incremented by 2 bytes to point to the next ICs cell
                //voltage data
                data_counter=data_counter+2;
            }
        }
    }
}


void mux_disable(uint8_t total_ic, uint8_t i2c_adress) {
    uint8_t command = 0x00;
    write_i2c(total_ic, i2c_address, command, 0);
}

void mux_set_channel(uint8_t total_ic, uint8_t i2c_address, uint8_t channel) {
    uint8_t command = 0x08 | channel;
    write_i2c(total_ic, i2c_address, command, 0, 0); //(total_ic, address, command, data, data_length)
}

/* What do we still have to do?
 *
 * In general:
 *  - Wakeup Sleep
 *
 * For voltage sensing:
 *x  - adcv (start cell conversion)
 *x  - pollADC (Wait for cell conversion to be done)
 *x  - rdcv (read back cell voltages)
 *
 * For temp sensing:
 *  - I2C enable specific channels
 *  - I2C disable mux
 *          - mux_disable
            - mux_set_channel
 *  - adax (start aux voltage)
 *  - pollADC
 *  -
 */
