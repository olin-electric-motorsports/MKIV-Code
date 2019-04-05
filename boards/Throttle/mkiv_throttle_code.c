/*
  Code for the OEM MKIV Throttle Board
  Author: awenstrup
*/

//Includes
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "can_api.h"
#include "log_uart.h"

#include <util/delay.h>

//Macro Definitinos

//Pin and Port Macros
#define THROTTLE_1 PC4
#define THROTTLE_2 PC5
#define THROTTLE_PORT PORTC

#define THROTTLE1_ADC_NUM 0b01000 //ADC8
#define THROTTLE2_ADC_NUM 0b01001 //ADC9
#define DRIVE_MODE_ADC    0b00010 //ADC2

#define DRIVE_MODE PD5
#define DRIVE_MODE_PORT PORTD

#define SS_ESTOP PB5
#define SS_IS PB6
#define SS_BOTS PB7
#define SS_PORT PORTB

#define PLED1 PC6
#define PLED1_PORT PORTC
#define PLED2 PB3
#define PLED2_PORT PORTB
#define PLED3 PB4
#define PLED3_PORT PORTB

#define LED1 PB0 //orange
#define LED2 PB1 //green

//CAN Macros
#define CAN_THROTTLE 0
#define CAN_DRIVE_MODE 1
#define CAN_ESTOP 2
#define CAN_IS 3
#define CAN_BOTS 4

//Flags
#define FLAG_BRAKE 0
#define FLAG_THROTTLE_BRAKE 1
#define FLAG_ESTOP 2
#define FLAG_IS 3
#define FLAG_BOTS 4
#define FLAG_MOTOR_ON 5
#define FLAG_THROTTLE_10 6
#define FLAG_PANIC 7

//Voltage Reference for Drive Mode Select
#define STANDARD_LOWER_BOUND 0
#define ACCELERATION_LOWER_BOUND 0
#define SKIDPAD_LOWER_BOUND 0
#define AUTOCROSS_LOWER_BOUND 0
#define ENDURANCE_LOWER_BOUND 0

#define ROLLING_AVG_SIZE 8


//********************Global variables***************
uint8_t gFlag = 0;

uint16_t gDriveModeVoltage = 0;
uint8_t gDriveMode = 0;
/* Drive modes:
0 = error (no signal recieved, default to standard)
1 = standard (linear torque request)
2 = acceleration
3 = skid pad
4 = autocross
5 = endurance
*/

uint8_t gError = 0b00000000;
/* Error definitions:
0 = No errors
1 = Drive mode error
*/

uint8_t gThrottle1Voltage = 0;
uint8_t gThrottle2Voltage = 0;
uint8_t gThrottle1In = 0;
uint8_t gThrottle2In = 0;
uint8_t gThrottle1Out = 0;
uint8_t gThrottle2Out = 0;

uint8_t gThrottleArrayIndex = 0;
//uint8_t gAverageSize = 8;
uint8_t gThrottleArray[ROLLING_AVG_SIZE];
uint8_t gThrottleOut = 0;

// Throttle mapping values
// NEEDS TO BE SET ACCORDING TO READ VALUES AFTER CENTERING
//Values set last on March 30th by Alex (BASED ON TESTING POTS)
const uint8_t throttle1_HIGH = 255;
const uint8_t throttle1_LOW = 0;
const uint8_t throttle2_HIGH = 255;
const uint8_t throttle2_LOW = 0;

uint8_t throttle10Counter = 0;
const uint8_t throttle10Ticks = 10; //number of measurements that corespond to an implausibility error

uint8_t gCANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t gCANMotorController[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//********************Functions*************************

//Initializers
void initTimer(void) {
    // Set up 8-bit timer in CTC mode
    TCCR0A = _BV(WGM01);

    // Set clock prescaler to (1/256) - page 89
    TCCR0B = 0b100;

    // Enable Match A interupts - page 90
    TIMSK0 |= _BV(OCIE0A);

    //Makes the timer reset everytime it hits 255
    // - page 90
    OCR0A = 0xFF;
}

void initADC(void) {
    // Enable Analog Digital Converter with
    // frequency (1/32) * system clock - page 212
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);

    //Enable interal reference voltage
    ADCSRB &= _BV(AREFEN);

    //Set internal reference voltage as AVcc
    ADMUX |= _BV(REFS0);

    //Reads by default from ADC0 (pin 11)
    //This line is redundant. The timer
    ADMUX |= _BV(0x00);
}

void setPledOut(void) {
    //Sets the PLEDs as output
    DDRC |= _BV(PLED1);
    DDRB |= _BV(PLED2);
    DDRB |= _BV(PLED3);
}

void initDriveMode(void) {
    ADMUX |= DRIVE_MODE_ADC;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t dm = ADC;

    if(dm < STANDARD_LOWER_BOUND) {
      gDriveMode = 0;
      gError = 1;
    }
    else if(dm < ACCELERATION_LOWER_BOUND) {
      gDriveMode = 1;
    }
    else if(dm < SKIDPAD_LOWER_BOUND) {
      gDriveMode = 2;
    }
    else if(dm < AUTOCROSS_LOWER_BOUND) {
      gDriveMode = 3;
    }
    else if(dm < ENDURANCE_LOWER_BOUND) {
      gDriveMode = 4;
    }
    else {
      gDriveMode = 5;
    }
}

void showError(void) {
    if (gError % 2 == 1) {
      PLED1_PORT |= _BV(PLED1);
    }
    else {
      PLED1_PORT &= ~_BV(PLED1);
    }

    gError /= 2;

    if (gError % 2 == 1) {
      PLED2_PORT |= _BV(PLED2);
    }
    else {
      PLED2_PORT &= ~_BV(PLED2);
    }

    gError /= 2;

    if (gError % 2 == 1) {
      PLED3_PORT |= _BV(PLED3);
    }
    else {
      PLED3_PORT &= ~_BV(PLED3);
    }

    gError = 0;
}

void checkShutdownState(void)   {
    //Sets the value of the CANMessage array
    //at the correct position
    //to 255 if the shutdown sense is triggered
    if(bit_is_set(gFlag,FLAG_ESTOP)){
        gCANMessage[CAN_ESTOP] = 0xFF;
    } else {
        gCANMessage[CAN_ESTOP] = 0x00;
    }

    if(bit_is_set(gFlag,FLAG_IS)) {
        gCANMessage[CAN_IS]= 0xFF;
    } else {
        gCANMessage[CAN_IS] = 0x00;
    }

    if(bit_is_set(gFlag,FLAG_BOTS)){
        gCANMessage[CAN_IS] = 0xFF;
    } else {
        gCANMessage[CAN_BOTS] = 0x00;
    }
}

void checkPanic(void) {
  if(bit_is_set(gFlag,FLAG_PANIC) || bit_is_set(gFlag, FLAG_THROTTLE_10)){
      gThrottle1Out = 0x00;
      gThrottle2Out = 0x00;
      PLED1_PORT ^= _BV(PLED1);
      PLED2_PORT ^= _BV(PLED2);
      PLED3_PORT ^= _BV(PLED3);
  }
}

void readPots(void) {
    //Read values from ADC and store them
    //in their appropriate variables
    //Reads: throttle1 and throttle2

    ADMUX |= THROTTLE1_ADC_NUM;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t t1 = 100;//ADC >> 2;

    ADMUX |= THROTTLE2_ADC_NUM;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t t2 = 100;//ADC >> 2;

    gThrottle1Voltage = t1;
    gThrottle2Voltage = t2;
}

void mapThrottle(void) {
    uint32_t map1;
    uint32_t map2;

    if (gThrottle1Voltage < throttle1_LOW) {
      gThrottle1Voltage = throttle1_LOW;
    }
    else if (gThrottle1Voltage > throttle1_HIGH) {
      gThrottle1Voltage = throttle1_HIGH;
    }

    if (gThrottle2Voltage < throttle2_LOW) {
      gThrottle2Voltage = throttle2_LOW;
    }
    else if (gThrottle2Voltage > throttle2_HIGH) {
      gThrottle2Voltage = throttle2_HIGH;
    }

    map1 = (256 * (gThrottle1Voltage - throttle1_LOW)) / (throttle1_HIGH - throttle1_LOW);
    map2 = (256 * (gThrottle2Voltage - throttle2_LOW)) / (throttle2_HIGH - throttle2_LOW);
    gThrottle1In = map1;
    gThrottle2In = map2;

    //map based on drive mode
    gThrottle1Out = gThrottle1In;
    gThrottle2Out = gThrottle2In;

    PLED1_PORT |= _BV(PLED1);
}

void checkPlausibility(void) {
  if (gThrottle1Voltage * 11 < gThrottle2Voltage * 10 || gThrottle1Voltage * 10 > gThrottle2Voltage * 11) {
    throttle10Counter += 1;
  }
  else {
    throttle10Counter = 0;
  }

  if (throttle10Counter > throttle10Ticks) {
    gFlag |= _BV(FLAG_THROTTLE_10);
  }
  else {
    gFlag &= ~_BV(FLAG_THROTTLE_10);
  }
}

void storeThrottle(void) {
  readPots();
  mapThrottle();
  checkPlausibility();

  uint16_t temp = (gThrottle1Out + gThrottle2Out) > 1;
  gThrottleArray[gThrottleArrayIndex] = temp;

  gThrottleArrayIndex = gThrottleArrayIndex + 1;

  if (gThrottleArrayIndex == ROLLING_AVG_SIZE) {
    gThrottleArrayIndex = 0;
  }
}

void getAverage(void) {
  uint32_t temp = 0;
  for (int i = 0; i < ROLLING_AVG_SIZE; i++) {
    temp += gThrottleArray[i];
  }
  temp /= ROLLING_AVG_SIZE;
  gThrottleOut = temp;
}


void printThrottle1(void) {
  char uart_buf[64];
  sprintf(uart_buf, "t1: %d", gThrottle1In);
  LOG_println(uart_buf, strlen(uart_buf));
}


void printThrottle(void) {
  char uart_buf[32];
  sprintf(uart_buf, "i: %d", gThrottleArrayIndex);
  LOG_println(uart_buf, strlen(uart_buf));
}




//------------Main Loop----------------
int main(void) {
  initTimer();
  initADC();
  sei();
  setPledOut();
  LOG_init();

  while(1) {
    storeThrottle();
    getAverage();
    printThrottle();
    _delay_ms(1000);
  }
}
