/*

   AIR Control Board


*/

/*----- Includes -----*/
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"

/*----- Outputs -----*/
#define LED1				PC4
#define LED2				PC5
#define LED_PORT	PORTC

#define RJ45_LED1				PB0
#define RJ45_LED2				PB1
#define RJ45_LED_PORT	PORTB

#define PRECHARGE_CTRL			PB4
#define PRECHARGE_PORT		PORTB
#define AIRMINUS_CTRL				PB5
#define AIRMINUS_PORT			PORTB

/*----- Inputs -----*/
#define PIN_AIRMINUS_AUX		PC7 // PCINT15
#define PIN_AIRPLUS_AUX			PC6 // PCINT14
#define INREG_AIRS				 PINC // INREG -> input register

#define PIN_BMS_STATUS			PB2 // PCINT2
#define INREG_BMS_STATUS	 PINB
#define PIN_IMD_STATUS			PB3 // PCINT3
#define INREG_IMD_STATUS	 PINB
#define PIN_SS_HVD					PB6 // PCINT6
#define INREG_SS_HVD			 PINB
#define PIN_SS_MP						PD0 // PCINT16
#define INREG_SS_MP				 PIND
#define PIN_SS_IMD					PC0 // PCINT8
#define INREG_SS_IMD			 PINC
#define PIN_SS_BMS					PC1 // PCINT9
#define INREG_SS_BMS			 PINC

/*----- Fault Codes -----*/
#define FAULT_CODE_GENERAL										0X00;
#define FAULT_CODE_BMS_IMPLAUSIBILITY 				0x01;
#define FAULT_CODE_IMD_IMPLAUSIBILITY 				0x02;
#define FAULT_CODE_AIRPLUS_WELD 							0x03;
#define FAULT_CODE_AIRMINUS_WELD		 					0x04;
#define FAULT_CODE_AIRPLUS_CONTROL_LOSS 			0x05;
#define FAULT_CODE_AIRMINUS_CONTROL_LOSS 			0x06;
#define FAULT_CODE_PRECHARGE_STUCK						0X07;
#define FAULT_CODE_PRECHARGE_CONTROL_LOSS			0X08;
#define FAULT_CODE_DISCHARGE_STUCK						0X09;
#define FAULT_CODE_DISCHARGE_CONTROL_LOSS			0X0A;


/*----- gFlag -----*/
#define UPDATE_STATUS       0
#define FLAG_AIRPLUS_AUX    1
#define FLAG_AIRMINUS_AUX   2
#define FLAG_IMD_STATUS     3
#define FLAG_BMS_STATUS     4
#define FLAG_TSMS_STATUS		5
#define FLAG_TS_VOLTAGE			6
#define FLAG_TS_CURRENT			7

/*----- sFlag -----*/
#define FLAG_SS_HVD   0
#define FLAG_SS_MP    1 // MP -> main pack connector
#define FLAG_SS_IMD   2
#define FLAG_SS_BMS   3

/*----- TS Statuses -----*/
#define TS_STATUS_DEENERGIZED 		0x00
#define TS_STATUS_PRECHARGE_DELAY 0x01
#define TS_STATUS_PRECHARGING 		0x02
#define TS_STATUS_ENERGIZED 			0x03
#define TS_STATUS_DISCHARGING 		0x04
#define TS_STATUS_PANIC						0x05

/*----- MOBs -----*/
#define MOB_BROADCAST_CRITICAL	0 //Broadcasts precharge sequence complete
#define MOB_MOTORCONTROLLER 		1 //Receives messages from motor controller
#define MOB_PANIC 							2 //Panic MOB for BMS to open shutdown circuit
#define MOB_BROADCAST_SS				3
#define	MOB_BRAKELIGHT					4
#define	MOB_IBH									5

volatile uint8_t gFlag = 0x00; // Global Flag
volatile uint8_t sFlag = 0x00; // Shutdown Sense Flag
volatile uint8_t LEDtimer = 0x00;
volatile uint8_t motorControllerVoltage = 0x00;
extern uint8_t tractiveSystemStatus = 0; //

uint8_t msgCritical[4] = {0,0,0,0};
#define MSG_INDEX_PRECHARGE_STATUS	0
#define MSG_INDEX_AIRPLUS_AUX				1
#define MSG_INDEX_AIRMINUS_AUX			2
#define MSG_INDEX_HV_CHECK					3

uint8_t msgShutdownSense[6] = {0,0,0,0,0,0};
#define MSG_INDEX_SS_MP					0
#define MSG_INDEX_SS_HVD				1
#define MSG_INDEX_SS_BMS				2
#define MSG_INDEX_SS_IMD				3
#define MSG_INDEX_BMS_STATUS		4
#define MSG_INDEX_IMD_STATUS		5

ISR(TIMER0_COMPA_vect) {
    /*
    Timer/Counter0 compare match A
    If the clock frequency is 4MHz then this is called 16 times per second
    MATH: (4MHz/1024)/255 = ~16
    */

		gFlag |= _BV(UPDATE_STATUS);
		LEDtimer++;
		if(LEDtimer > 100){
			LEDtimer = 0;
			LED_PORT ^= _BV(LED2);
		}
}

/*
ISR(CAN_INT_vect) {
    CANPAGE = (MOB_AIR_CRIT << MOBNB0); // Switch to MOb 0, the one we're listening on.
    if(bit_is_set(CANSTMOB, RXOK)) {
        volatile uint8_t msg = CANMSG;      //grab the first byte of the CAN message

        if(msg == 0xFF) {
            PORTB |= _BV(LED2_PIN);
        } else {
            PORTB &= ~_BV(LED2_PIN);
        }

        m//Setup to Receive Again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(0, CAN_ID_TUTORIAL6, CAN_LEN_TUTORIAL6, 0xFF);
    }
}
*/

ISR(PCINT0_vect) { // PCINT0-7 -> BMS_STATUS, IMD_STATUS, SS_HVD
    if(bit_is_set(INREG_BMS_STATUS,PIN_BMS_STATUS)){
		 gFlag |= _BV(FLAG_BMS_STATUS);
		} else {
		 gFlag &= ~_BV(FLAG_BMS_STATUS);
		}

		if(bit_is_set(INREG_IMD_STATUS,PIN_IMD_STATUS)){
		 gFlag |= _BV(FLAG_IMD_STATUS);
		} else {
		 gFlag &= ~_BV(FLAG_IMD_STATUS);
		}

		if(bit_is_clear(INREG_SS_HVD,PIN_SS_HVD)){
		 gFlag |= _BV(FLAG_SS_HVD);
		} else {
		 gFlag &= ~_BV(FLAG_SS_HVD);
		}
}

ISR(PCINT1_vect) { // PCINT8-15 -> AIRPLUS_AUX, AIRMINUS_AUX, SS_IMD, SS_BMS
		if(bit_is_set(INREG_AIRS,PIN_AIRPLUS_AUX)){
		 gFlag |= _BV(FLAG_AIRPLUS_AUX);
		} else {
		 gFlag &= ~_BV(FLAG_AIRPLUS_AUX);
		}

		if(bit_is_set(INREG_AIRS,PIN_AIRMINUS_AUX)){
		 gFlag |= _BV(FLAG_AIRMINUS_AUX);
		} else {
		 gFlag &= ~_BV(FLAG_AIRMINUS_AUX);
		}

		if(bit_is_clear(INREG_SS_IMD,PIN_SS_IMD)){
		 gFlag |= _BV(FLAG_SS_IMD);
		} else {
		 gFlag &= ~_BV(FLAG_SS_IMD);
		}

		if(bit_is_clear(INREG_SS_BMS,PIN_SS_BMS)){
		 gFlag |= _BV(FLAG_SS_BMS);
		} else {
		 gFlag &= ~_BV(FLAG_SS_BMS);
		}
}

ISR(PCINT2_vect) { // PCINT16-23 -> SS_MP
		if(bit_is_clear(INREG_SS_MP,PIN_SS_MP)){
		 gFlag |= _BV(FLAG_SS_MP);
		} else {
		 gFlag &= ~_BV(FLAG_SS_MP);
		}
}

void initTimer(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);  // Every 1024 cycles, OCR0A increments
    OCR0A = 0x27; //dec 39  // until 0xff, 255, which then calls for
                            // the TIMER0_COMPA_vect interrupt
			    // currently running at 100Hz
}

void setOutputs(void) {
		//Sets these pins at outputs
		DDRB |= _BV(PRECHARGE_CTRL) | _BV(AIRMINUS_CTRL) | _BV(RJ45_LED1) | _BV(RJ45_LED2);
		DDRC |= _BV(LED1) | _BV(LED2);
}

void checkBMSPowerStagePlausibility (void) {
		if ( (bit_is_set(gFlag, FLAG_BMS_STATUS) && bit_is_clear(sFlag, FLAG_SS_BMS))
		|| (bit_is_clear(gFlag, FLAG_BMS_STATUS) && bit_is_set(sFlag, FLAG_SS_BMS)) {
			panic(FAULT_CODE_BMS_IMPLAUSIBILITY);
		}
}

void checkIMDPowerStagePlausibility (void) {
		if ( (bit_is_set(gFlag, FLAG_IMD_STATUS) && bit_is_clear(sFlag, FLAG_SS_IMD))
		|| (bit_is_clear(gFlag, FLAG_IMD_STATUS) && bit_is_set(sFlag, FLAG_SS_IMD)) {
			panic(FAULT_CODE_IMD_IMPLAUSIBILITY);
		}
}

void checkAIRPLUS (void) {
		if ( bit_is_set(gFlag, FLAG_AIRPLUS_AUX) && bit_is_clear(sFlag, FLAG_TSMS_STATUS) ) {
			panic(FAULT_CODE_AIRPLUS_WELD);
		} else if ( bit_is_clear(gFlag, FLAG_AIRPLUS_AUX) && bit_is_set(sFlag, FLAG_TSMS_STATUS) ) {
			panic(FAULT_CODE_AIRPLUS_CONTROL_LOSS);
		}
}

void checkAIRMINUS (void) {
		if ( bit_is_clear(AIRMINUS_PORT, AIRMINUS_CTRL) && bit_is_set(gFlag, FLAG_AIRMINUS_AUX) ) {
			panic(FAULT_CODE_AIRMINUS_WELD);
		} else if ( bit_is_set(AIRMINUS_PORT, AIRMINUS_CTRL) && bit_is_clear(gFlag, FLAG_AIRMINUS_AUX) ) {
			panic(FAULT_CODE_AIRMINUS_CONTROL_LOSS);
		}
}

void conditionalMessageSet (uint8_t reg, uint8_t bit, uint8_t msg[], uint8_t index, uint8_t condHigh, uint8_t condLow) {
		if(bit_is_set(reg,bit)){
		 msg[index] = condHigh;
		} else {
		 msg[index] = condLow;
		}
}

void sendShutdownSenseCANMessage (void) {
	conditionalMessageSet(sFlag, FLAG_SS_MP, msgShutdownSense, MSG_INDEX_SS_MP, 0xff, 0x00);
	conditionalMessageSet(sFlag, FLAG_SS_HVD, msgShutdownSense, MSG_INDEX_SS_HVD, 0xff, 0x00);
	conditionalMessageSet(sFlag, FLAG_SS_IMD, msgShutdownSense, MSG_INDEX_SS_IMD, 0xff, 0x00);
	conditionalMessageSet(sFlag, FLAG_SS_BMS, msgShutdownSense, MSG_INDEX_SS_BMS, 0xff, 0x00);
	conditionalMessageSet(gFlag, FLAG_BMS_STATUS, msgShutdownSense, MSG_INDEX_BMS_STATUS, 0xff, 0x00);
	conditionalMessageSet(gFlag, FLAG_IMD_STATUS, msgShutdownSense, MSG_INDEX_IMD_STATUS, 0xff, 0x00);
	CAN_transmit(MOB_BROADCAST_SS,
								CAN_ID_AIR_CONTROL_SENSE,
								CAN_LEN_AIR_CONTROL_SENSE,
								msgShutdownSense);
}

void sendCriticalCANMessage (void) {
	// TODO -> hv check?
	conditionalMessageSet(gFlag, FLAG_AIRPLUS_AUX, msgCritical, MSG_INDEX_AIRPLUS_AUX, 0xff, 0x00);
	conditionalMessageSet(gFlag, FLAG_AIRMINUS_AUX, msgCritical, MSG_INDEX_AIRMINUS_AUX, 0xff, 0x00);
	CAN_transmit(MOB_BROADCAST_CRITICAL,
								CAN_ID_AIR_CONTROL_CRITICAL,
								CAN_LEN_AIR_CONTROL_CRITICAL,
								msgCritical);
}

void panic (uint8_t fault_code) {
	tractiveSystemStatus = TS_STATUS_PANIC;
	uint8_t msg[1] = {fault_code};
	CAN_transmit(MOB_PANIC,
								CAN_ID_PANIC,
								CAN_LEN_PANIC,
								msg);
}

int main (void) {
    initTimer();
    sei(); //Inititiates interrupts for the ATMega
    CAN_init(CAN_ENABLED);
    CAN_wait_on_receive(MOB_MOTORCONTROLLER, CAN_ID_MC_VOLTAGE, CAN_LEN_MC_VOLTAGE, 0xFF);
		CAN_wait_on_receive(MOB_BRAKELIGHT, CAN_ID_BRAKE_LIGHT, CAN_LEN_BRAKE_LIGHT, 0xFF); // TODO add interrupt handling

    // Enable interrupt
    PCICR |= _BV(PCIE0) | _BV(PCIE1) | _BV(PCIE2);

		setOutputs();

    while(1) {
			if(bit_is_set(gFlag, UPDATE_STATUS)){

				gFlag &= ~_BV(UPDATE_STATUS);

				checkBMSPowerStagePlausibility();
				checkIMDPowerStagePlausibility();
				checkAIRPLUS();
				checkAIRMINUS();
				sendShutdownSenseCANMessage();
				sendCriticalCANMessage();

				if(tractiveSystemStatus==TS_STATUS_DEENERGIZED){
					if(bit_is_set(gFlag, FLAG_TSMS_STATUS)){ // if tsms closed
						tractiveSystemStatus = TS_STATUS_PRECHARGE_DELAY; // set status to precharge delay
						// reset timer 2
						// set timer 2 ovf threshold for precharge delay
					}
				} else if(tractiveSystemStatus==TS_STATUS_PRECHARGE_DELAY) {
					if(motorControllerVoltage > 0){ // if voltage is increasing, panic(FAULT_CODE_PRECHARGE_STUCK)
						panic(FAULT_CODE_PRECHARGE_STUCK);
					}
					// if timer done
						tractiveSystemStatus = TS_STATUS_PRECHARGING; // set status to precharging
						// reset timer 2
						// set timer 2 ovf threshold for precharging
						msgCritical[MSG_INDEX_PRECHARGE_STATUS] = 0x0f; // update critical can message to precharge started
						PRECHARGE_PORT |= _BV(PRECHARGE_CTRL); // close precharge relay
				} else if(tractiveSystemStatus==TS_STATUS_PRECHARGING) {
					// if time is up
						// if voltage is high enough
							AIRMINUS_PORT |= _BV(AIRMINUS_CTRL); // close air minus
							checkAIRMINUS(); // confirm closure
							PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL); // open precharge relay
							if (tractiveSystemStatus != TS_STATUS_PANIC){ // if we passed AIR minus check
								tractiveSystemStatus = TS_STATUS_ENERGIZED; // set status to energized
								msgCritical[MSG_INDEX_PRECHARGE_STATUS] = 0xff; // update critical can message to precharge complete
							}
						if(motorControllerVoltage == 0){ // if voltage is 0
							panic(FAULT_CODE_PRECHARGE_CONTROL_LOSS); // panic(FAULT_CODE_PRECHARGE_CONTROL_LOSS)
						}
						// if voltage isn't high enough
							// panic(FAULT_CODE_DISCHARGE_STUCK)
				} else if(tractiveSystemStatus==TS_STATUS_ENERGIZED) {
					if(bit_is_clear(gFlag, FLAG_TSMS_STATUS)){ // if tsms node no longer has shutdown voltage
						AIRMINUS_PORT &= ~_BV(AIRMINUS_CTRL); // open air minus
						msgCritical[MSG_INDEX_PRECHARGE_STATUS] = 0x00; // update critical can message to precharge not started
						tractiveSystemStatus = TS_STATUS_DISCHARGING; // set status to discharging
						// reset timer 2
						// set timer 2 ovf threshold for discharging
					}
				} else if(tractiveSystemStatus==TS_STATUS_DISCHARGING) {
					// if time is up
						if(motorControllerVoltage == 0){ // if voltage is 0
							tractiveSystemStatus = TS_STATUS_DEENERGIZED; // set status to deenergized
						} else { // else
							panic(FAULT_CODE_DISCHARGE_STUCK); // panic(FAULT_CODE_DISCHARGE_CONTROL_LOSS)
						}
				} else if(tractiveSystemStatus==TS_STATUS_PANIC) {
					AIRMINUS_PORT &= ~_BV(AIRMINUS_CTRL); // open air minus and precharge
					PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL);
					panic(FAULT_CODE_GENERAL); // see that panic keeps being sent...
				}

			}

    }
}
