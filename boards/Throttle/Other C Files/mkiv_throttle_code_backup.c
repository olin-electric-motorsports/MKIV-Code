/*
  Code for the OEM MKIV Throttle Board
  Author: @awenstrup
*/

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

//********************Global variables***************
uint16_t driveModeVoltage = 0;
uint8_t driveMode = 0;

uint8_t throttle[2] = {0, 0};
uint8_t throttleVoltage[2] = {0, 0};

uint8_t CANMessage[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t CANMotorController = {0, 0, 0, 0 0, 0, 0, 0}
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

void initDriveMode(void) {
    ADMUX |= DRIVE_MODE_ADC;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t throttle1 = ADC;

    if
}

void checkShutdownState(void)   {
    //Sets the value of the CANMessage array
    //at the correct position
    //to 255 if the shutdown sense is triggered
    if(bit_is_set(gFlag,FLAG_ESTOP)){
        CANMessage[CAN_E_STOP] = 0xFF;
    } else {
        CANMessage[CAN_E_STOP] = 0x00;
    }

    if(bit_is_set(gFlag,FLAG_INERTIA)) {
        CANMessage[CAN_IS]= 0xFF;
    } else {
        CANMessage[CAN_IS] = 0x00;
    }

    if(bit_is_set(gFlag,FLAG_BOTS)){
        CANMessage[CAN_IS] = 0xFF;
    } else {
        CANMessage[CAN_BOTS] = 0x00;
    }
}

void checkPanic(void) {
  if(bit_is_set(gFlag,FLAG_PANIC)){
      throttle[0] = 0x00;
      throttle[1] = 0x00;
      LED1_PORT |= _BV(LED1);
      LED2_PORT |= _BV(LED2);
      LED3_PORT |= _BV(LED3);
  }
}

void readPots(void) {
    //Read values from ADC and store them
    //in their appropriate variables
    //Reads: throttle1 and throttle2

    ADMUX |= THROTTLE1_ADC_NUM;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t t1 = ADC;

    ADMUX |= THROTTLE2_ADC_NUM;
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint16_t t2 = ADC;

    throttleVoltage[0] = t1;
    throttleVoltage[1] = t2;
}
