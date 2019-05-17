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
#define FAULT_CODE_GENERAL										0X00
#define FAULT_CODE_BMS_IMPLAUSIBILITY 				0x01
#define FAULT_CODE_IMD_IMPLAUSIBILITY 				0x02
#define FAULT_CODE_AIRPLUS_WELD 							0x03
#define FAULT_CODE_AIRMINUS_WELD		 					0x04
#define FAULT_CODE_AIRPLUS_CONTROL_LOSS 			0x05
#define FAULT_CODE_AIRMINUS_CONTROL_LOSS 			0x06
#define FAULT_CODE_PRECHARGE_STUCK						0X07
#define FAULT_CODE_PRECHARGE_CONTROL_LOSS			0X08
#define FAULT_CODE_DISCHARGE_STUCK						0X09 // unused because of difficulty of detection
#define FAULT_CODE_DISCHARGE_CONTROL_LOSS			0X0A
#define FAULT_CODE_PRECHARGE_INCOMPLETE				0X0B // if precharge is too slow or stopping below MINIMUM_VOLTAGE_AFTER_PRECHARGE

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
#define MOB_BROADCAST_CRITICAL	0 // Broadcasts precharge sequence complete
#define MOB_MOTORCONTROLLER 		1 // Receives messages from motor controller
#define MOB_BROADCAST_PANIC 		2 // Panic MOB for BMS to open shutdown circuit
#define MOB_BROADCAST_SS				3
#define	MOB_BRAKELIGHT					4
#define	MOB_IBH									5 // TODO change this to panic message? probably should listen for panics

/*----- Overflow Counts for TS Status Delays -----*/
// Timer1 is running at ~7.63 Hz
#define OVF_COUNT_PRECHARGE_DELAY		0x17 // roughly 3 seconds per FMEA
#define OVF_COUNT_PRECHARGING 			0x10 // roughly 2 seconds -> 3 time constants (1/(2*pi*1k*440uF))*3 = 1.08ish
#define OVF_COUNT_DISCHARGING 			0x1F // roughly 4 seconds -> 3 times as long as precharge (3k res vs 1k res) -> 3.24ish

/*----- Other Macros -----*/
#define MINIMUM_VOLTAGE_AFTER_PRECHARGE		0xB4 // 180V, 90% of minimum pack voltage (200V)

volatile uint8_t gFlag = 0x00; // Global Flag
volatile uint8_t sFlag = 0x00; // Shutdown Sense Flag
volatile uint8_t LEDtimer = 0x00;
volatile uint16_t motorControllerVoltage = 0x0000;
uint8_t tractiveSystemStatus = 0;
volatile uint8_t timer1OverflowCount = 0;

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

ISR(TIMER1_OVF_vect) {
		timer1OverflowCount++;
}

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
		 sFlag |= _BV(FLAG_SS_HVD);
		} else {
		 sFlag &= ~_BV(FLAG_SS_HVD);
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
		 sFlag |= _BV(FLAG_SS_IMD);
		} else {
		 sFlag &= ~_BV(FLAG_SS_IMD);
		}

		if(bit_is_clear(INREG_SS_BMS,PIN_SS_BMS)){
		 sFlag |= _BV(FLAG_SS_BMS);
		} else {
		 sFlag &= ~_BV(FLAG_SS_BMS);
		}
}

ISR(PCINT2_vect) { // PCINT16-23 -> SS_MP
		if(bit_is_clear(INREG_SS_MP,PIN_SS_MP)){
		 sFlag |= _BV(FLAG_SS_MP);
		} else {
		 sFlag &= ~_BV(FLAG_SS_MP);
		}
}

ISR(CAN_INT_vect) {
		CANPAGE = (MOB_MOTORCONTROLLER << MOBNB0);
	  if (bit_is_set(CANSTMOB,RXOK)) {
	      uint8_t msgMC[2];
	      msgMC[0] = CANMSG;
	      msgMC[1] = CANMSG;

	      motorControllerVoltage = msgMC[0] | (msgMC[1]<<8);

	      CANSTMOB = 0x00;
	      CAN_wait_on_receive(MOB_MOTORCONTROLLER,
	                          CAN_ID_MC_VOLTAGE,
	                          CAN_LEN_MC_VOLTAGE,
	                          CAN_MSK_SINGLE);
	  }
		CANPAGE = (MOB_BRAKELIGHT << MOBNB0);
	  if (bit_is_set(CANSTMOB,RXOK)) {
	      uint8_t msgBL[5];
	      msgBL[0] = CANMSG; msgBL[1] = CANMSG; msgBL[2] = CANMSG; msgBL[3] = CANMSG; // increment CANMSG to get to TSMS info
				msgBL[4] = CANMSG; // TSMS sense

	      if(msgBL[4]==0xff){
					gFlag |= _BV(FLAG_TSMS_STATUS);
				} else {
					gFlag &= ~_BV(FLAG_TSMS_STATUS);
				}

	      CANSTMOB = 0x00;
	      CAN_wait_on_receive(MOB_BRAKELIGHT,
	                          CAN_ID_BRAKE_LIGHT,
	                          CAN_LEN_BRAKE_LIGHT,
	                          CAN_MSK_SINGLE);
	  }
}

void initTimer0(void) {
    TCCR0A = _BV(WGM01);    // Set up 8-bit timer in CTC mode
    TCCR0B = 0x05;          // clkio/1024 prescaler
    TIMSK0 |= _BV(OCIE0A);  // Every 1024 cycles, OCR0A increments
    OCR0A = 0x27; //dec 39  // until 0xff, 255, which then calls for
                            // the TIMER0_COMPA_vect interrupt
			    // currently running at 100Hz
}

void initTimer1(void) {
		// Normal operation so no need to set TCCR1A
		TCCR1B |= _BV(CS11); // prescaler set to 8
		// 4MHz CPU, prescaler 8, 16-bit timer overflow -> (4000000/8)/(2^16-1) =  7.63 Hz
		TIMSK1 = 0x01; // enable interrupt on overflow
}

void resetTimer1(void) {
		cli();
		TCNT1H = 0x00; // write timer count to 0, high byte must be written first per datasheet
		TCNT1L = 0x00;
		sei();
		timer1OverflowCount = 0x00;
}

void setOutputs(void) {
		//Sets these pins at outputs
		DDRB |= _BV(PRECHARGE_CTRL) | _BV(AIRMINUS_CTRL) | _BV(RJ45_LED1) | _BV(RJ45_LED2);
		DDRC |= _BV(LED1) | _BV(LED2);
}

void readAllInputs(void){
		PCINT0_vect();
		PCINT1_vect();
		PCINT2_vect();
}

void panic (uint8_t fault_code) {
	//LED_PORT ^= _BV(LED1); // DEBUG
	tractiveSystemStatus = TS_STATUS_PANIC;
	uint8_t msg[1] = {fault_code};
	CAN_transmit(MOB_BROADCAST_PANIC,
								CAN_ID_PANIC,
								CAN_LEN_PANIC,
								msg);
}

void checkBMSPowerStagePlausibility (void) { // tested and working
		if ( (bit_is_set(gFlag, FLAG_BMS_STATUS) && bit_is_set(sFlag, FLAG_SS_MP) && bit_is_clear(sFlag, FLAG_SS_BMS))
		|| (bit_is_clear(gFlag, FLAG_BMS_STATUS) && bit_is_set(sFlag, FLAG_SS_BMS)) ) {
			panic(FAULT_CODE_BMS_IMPLAUSIBILITY);
		}
}

void checkIMDPowerStagePlausibility (void) { // tested and working
		if ( (bit_is_set(gFlag, FLAG_IMD_STATUS) && bit_is_set(sFlag, FLAG_SS_BMS) && bit_is_clear(sFlag, FLAG_SS_IMD))
		|| (bit_is_clear(gFlag, FLAG_IMD_STATUS) && bit_is_set(sFlag, FLAG_SS_IMD)) ) {
			panic(FAULT_CODE_IMD_IMPLAUSIBILITY);
		}
}

void checkAIRPLUS (void) {
		if ( bit_is_set(gFlag, FLAG_AIRPLUS_AUX) && bit_is_clear(gFlag, FLAG_TSMS_STATUS) ) {
			panic(FAULT_CODE_AIRPLUS_WELD);
		} else if ( bit_is_clear(gFlag, FLAG_AIRPLUS_AUX) && bit_is_set(gFlag, FLAG_TSMS_STATUS) ) {
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

void updateCoolingPressure (void) { // TODO where is this going in CAN?
		// TODO
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

int main (void) {
    initTimer0();
		initTimer1();
    sei(); //Inititiates interrupts for the ATMega
    CAN_init(CAN_ENABLED);
    CAN_wait_on_receive(MOB_MOTORCONTROLLER, CAN_ID_MC_VOLTAGE, CAN_LEN_MC_VOLTAGE, CAN_MSK_SINGLE);
		CAN_wait_on_receive(MOB_BRAKELIGHT, CAN_ID_BRAKE_LIGHT, CAN_LEN_BRAKE_LIGHT, CAN_MSK_SINGLE);

    // Enable interrupt
    PCICR |= _BV(PCIE0) | _BV(PCIE1) | _BV(PCIE2);
		PCMSK0 |= _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT6);
		PCMSK1 |= _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT15);
		PCMSK2 |= _BV(PCINT16);

		setOutputs();
		readAllInputs(); // in case they are set high before micro starts up and therefore won't trigger an interrupt

    while(1) {
			if(bit_is_set(gFlag, UPDATE_STATUS)){

				gFlag &= ~_BV(UPDATE_STATUS); // TODO IMD STATUS PIN TURNS OFF SLOW DEBUG

				checkBMSPowerStagePlausibility();
				checkIMDPowerStagePlausibility(); // delay this for IMD startup time
				checkAIRPLUS(); // TODO think about charging, currently compares AIR plus to TSMS can message, won't work in charging
				checkAIRMINUS();
				sendShutdownSenseCANMessage();
				sendCriticalCANMessage();

				if(tractiveSystemStatus==TS_STATUS_DEENERGIZED){
						if(bit_is_set(gFlag, FLAG_TSMS_STATUS)){ // if tsms closed
							tractiveSystemStatus = TS_STATUS_PRECHARGE_DELAY; // set status to precharge delay
							resetTimer1(); // reset timer 1
						}
				} else if(tractiveSystemStatus==TS_STATUS_PRECHARGE_DELAY) {
						if(motorControllerVoltage > 0){ // if voltage is increasing, panic(FAULT_CODE_PRECHARGE_STUCK)
							panic(FAULT_CODE_PRECHARGE_STUCK);
						} else if(timer1OverflowCount>OVF_COUNT_PRECHARGE_DELAY){ // if precharge delay time elapsed
							tractiveSystemStatus = TS_STATUS_PRECHARGING; // set status to precharging
							msgCritical[MSG_INDEX_PRECHARGE_STATUS] = 0x0f; // update critical can message to precharge started
							PRECHARGE_PORT |= _BV(PRECHARGE_CTRL); // close precharge relay
							resetTimer1(); // reset timer 1
						}
				} else if(tractiveSystemStatus==TS_STATUS_PRECHARGING) {
						if(timer1OverflowCount>OVF_COUNT_PRECHARGING){ // if precharging time elapsed
							if(motorControllerVoltage == 0){ // if voltage is 0
								panic(FAULT_CODE_PRECHARGE_CONTROL_LOSS); // panic(FAULT_CODE_PRECHARGE_CONTROL_LOSS)
							} else if(motorControllerVoltage>MINIMUM_VOLTAGE_AFTER_PRECHARGE){ // if voltage is high enough
								AIRMINUS_PORT |= _BV(AIRMINUS_CTRL); // close air minus
								checkAIRMINUS(); // confirm closure
								PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL); // open precharge relay
								if (tractiveSystemStatus != TS_STATUS_PANIC){ // if we passed AIR minus check
									tractiveSystemStatus = TS_STATUS_ENERGIZED; // set status to energized
									msgCritical[MSG_INDEX_PRECHARGE_STATUS] = 0xff; // update critical can message to precharge complete
								}
							} else {
								panic(FAULT_CODE_PRECHARGE_INCOMPLETE);
							}
						}
				} else if(tractiveSystemStatus==TS_STATUS_ENERGIZED) {
						if(bit_is_clear(gFlag, FLAG_TSMS_STATUS)){ // if tsms node no longer has shutdown voltage
							AIRMINUS_PORT &= ~_BV(AIRMINUS_CTRL); // open air minus
							msgCritical[MSG_INDEX_PRECHARGE_STATUS] = 0x00; // update critical can message to precharge not started
							tractiveSystemStatus = TS_STATUS_DISCHARGING; // set status to discharging
							resetTimer1(); // reset timer 1
						}
				} else if(tractiveSystemStatus==TS_STATUS_DISCHARGING) {
						if(timer1OverflowCount>OVF_COUNT_DISCHARGING){ // if discharging time elapsed
							if(motorControllerVoltage == 0){ // if voltage is 0
								tractiveSystemStatus = TS_STATUS_DEENERGIZED; // set status to deenergized
							} else { // else
								panic(FAULT_CODE_DISCHARGE_CONTROL_LOSS); // panic(FAULT_CODE_DISCHARGE_CONTROL_LOSS)
							}
						}
				} else if(tractiveSystemStatus==TS_STATUS_PANIC) {
						AIRMINUS_PORT &= ~_BV(AIRMINUS_CTRL); // open air minus and precharge
						PRECHARGE_PORT &= ~_BV(PRECHARGE_CTRL);
						panic(FAULT_CODE_GENERAL); // see that panic keeps being sent...
				}

			}

    }
}
