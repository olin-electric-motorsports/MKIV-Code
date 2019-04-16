/*
Header:
	Code for the Brakelight/BSPD/Shutdown Sense Board lcoated
	in the LV-Box enclosure
Author:
	@author
*/

/*----- RJ45 LEDs -----*/
//Green: //TODO
//Orange:

/*----- Includes -----*/
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

/*----- Macro Definitions -----*/
/* Shutdown */
#define GLOBAL_SHUTDOWN         0x0

/* pins */

#define Brake_Switch 		//TODO

/* LEDs */
#define LED1                    PD6  
#define LED1_PORT               PORTD
#define LED2                    //TODO
#define LED2_PORT               
#define LED3                    
#define LED3_PORT               

#define EXT_LED_ORANGE          
#define EXT_LED_GREEN           
#define EXT_LED_PORT            

/* CAN Positions */
#define CAN_BRAKE_ANALOG_MSB	0
#define CAN_BRAKE_ANALOG_LSB	1
#define CAN_BRAKE_SWITCH        2
#define CAN_BSPD                3
#define CAN_TSMS            	4
#define CAN_ESTOP	        5
#define CAN_GLVMS		6

/* FLAGS */
#define FLAG_BRAKE		0
//TODO

/* MOBs */
#define MOB_BRAKELIGHT		0
// TODO


/* Functions to Create */
//CAN ISR Interrupt
//CAN transmit
//Timer ISR
//init_Timer function
//Check flags
//init ADC
//main loop




int main(void){
//init functions
//set pins as outputs/inputs
//CAN listen
//TODO

	while(1){
	//timer if statement
	//do stuff
	///TODO
	}
}









