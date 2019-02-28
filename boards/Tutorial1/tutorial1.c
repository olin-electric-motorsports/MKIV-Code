/*
 * Tutorial 1
 *
 * The goal of this program is to show a very basic way to turn on a light. It
 * is also referenced in Tutorial 0, about how to compile code and flash it to
 * the microcontroller, and it explains most of the parts of one of our AVR-C
 * programs.
 *
 * All reference info is pulled from
 * https://www.nongnu.org/avr-libc/user-manual/index.html
 *
 * @author Alex Hoppe '19
 */



/* ----------------------------------------------------------------------------
 * Usually a program starts with some includes:
 */

/* Includes standard AVR definitions (very generic files, each with logic that
 * sets up the right definitions based on which board type we pass in in the
 * avr-gcc command when we compile).
 *
 * #include <avr/sfr_defs.h>    - defines related to Special Function Registers
 * #include <avr/portpins.h>    - defines PORTs and PINs
 * #include <avr/common.h>      - defines shared registers among all AVR devices
 * #include <avr/version.h>     - defines version info of avr libraries */
#include <avr/io.h>

/* Includes pre-defined functions we can use for delays. */
#include <util/delay.h>


/* ----------------------------------------------------------------------------
 * Next up, a program has its own macro definitions and global variables
 */

#define LED_PIN PB0 /* This is a macro. The compiler looks at it at compile
                     * time and replaces all instances of LED_PIN with PB0 in
                     * the code. Funnily enough, PB0 is also a macro, which is
                     * defined in <avr/portpins.h>. It's convention to define
                     * macros in all-caps with underscores. */
                     
#define PINOUT_1 PB6
#define PINOUT_2 PB7

const uint8_t LED_pin = PB0;    /* This is a global variable (constant, so we
                                 * can't modify it). We can use it to define
                                 * something we'll use a lot instead of macros.
                                 * These have the advantage of type safety,
                                 * as opposed to macros which are just text 
                                 * replacement. */

uint8_t gGlobalInt = 0;         /* This is a modifiable global variable. You can
                                 * name it anything, but it's convention to
                                 * start the name with lower-case g. */


/* ----------------------------------------------------------------------------
 * Next, usually you'd find definitions of helper functions before your main
 * loop. This is because the compiler needs to see that your function exists
 * before it allows you to use the function.
 *
 * You can get around this by writing your functions in another file.c and then
 * adding #include "file.c" at the top, but it's conventional to write the
 * function definition in a file.h as well:
 *      void setup_led_pin(void);
 * and #include "file.h" to tell your compiler which functions you intend to
 * use later. If you include a file.h, the compiler will find the corresponding
 * file.c where the code actually is.
 */
void setup_led_pin(void) {
    DDRB |= _BV(LED_PIN);
}

void blink_once(void) {
    PORTB |= _BV(LED_PIN);  // Turn one LED on
    _delay_ms(200);          // wait 200 ms
    PORTB |= _BV(LED_PIN);  // Turn LED off
    _delay_ms(200);          // wait 200 ms
}

/* ----------------------------------------------------------------------------
 * Then at the bottom we have our main function (entry point) where the code
 * actually starts running when we flash it to the microcontroller.
 *
 * By C convention, the main function always has some integer return type.
 * This is a remnant of running C code in the command line, where the return
 * type would be used to return a number of errors that occurred. That way the
 * user would know if the command executed properly. For us, it doesn't do
 * anything because the microcontroller has nowhere to return a value to, and
 * it shouldn't ever exit the main loop anyway.
 */
int main (void) {                       /* This part of the code runs once */

    /* Set the data direction register  
     * so the led pin is output */

    // DDRB = DDRB | _BV(PB0);  // Using | and explicit name         
    DDRB |= _BV(LED_PIN);       // Using |= and pin name macro
    // setup_led_pin();         // A helper function that does the same thing

    /* Set up input pin with pull-up resistor */
    DDRB &= ~_BV(PINOUT_1);     /* Set pin to input (unnecessary, pins
                                 * default to input */
    PORTB |= _BV(PINOUT_1);     /* Set output high (if DDR is set to input, 
                                 * this sets a pull-up resistor */

    /* Blink LED once */
    blink_once();                       

    while (1) {                     /* This part of the code runs forever */
        
//      if (PINB & _BV(PINOUT_1)) {             /* Alternate way of writing */
        if (bit_is_set(PINB, PINOUT_1)) {       // If the Pinout1 pin is high
            PORTB &= ~_BV(LED_PIN);             // Turn LED off
        } else {                                // Otherwise
            PORTB |= _BV(LED_PIN);              // Turn it on
        }
    }                                   
}
