/*
 * rgb_led_v0.c
 *
 *  Created on: Apr 14, 2022
 *      Author: Julian Schneider
 */



//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for sprintf)


// Hardware configuration:
// Red LED:
//   M1PWM3 (PA7) drives an NPN transistor that powers the red LED
// Green LED:
//   M1PWM2 (PA6) drives an NPN transistor that powers the green LED
// Blue LED:
//   M1PWM0 (PD0) drives an NPN transistor that powers the blue LED

//Analog In
// Light Sensor AIN0 (PE3)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "rgb_led_v0.h"
#include "tm4c123gh6pm.h"
#include "clock.h"

// LED Masks
#define RED_LED_MASK 128
#define GREEN_LED_MASK 64
#define BLUE_LED_MASK 1

// Analog Mask
#define AIN0_MASK 8

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initLED()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable Clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1; // turn on PWM
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // port A
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // port D
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // port E
    _delay_cycles(3);

    // Configure three LEDs

    // Data enable GPIO for LEDs
    GPIO_PORTA_DEN_R |= RED_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTD_DEN_R |= BLUE_LED_MASK;

    // AFSEL registers for LEDs
    GPIO_PORTA_AFSEL_R |= RED_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTD_AFSEL_R |= BLUE_LED_MASK;

    // PCTL registers for LEDs
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA7_M | GPIO_PCTL_PA6_M); //r and g
    GPIO_PORTD_PCTL_R &= ~(GPIO_PCTL_PD0_M); // b

    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA7_M1PWM3 | GPIO_PCTL_PA6_M1PWM2; //r and g
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_M1PWM0; // b

    // Configure PWM module 1 to drive RGB LED
        // RED   on M1PWM3 (PA7), M1PWM1b
        // GREEN on M1PWM2 (PA6), M1PWM1a
        // BLUE  on M1PWM0 (PD0), M1PWM0a

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state

    PWM1_1_CTL_R = 0;                                // turn-off PWM1 generator 1 (drives outs 2 and 3)
    PWM1_0_CTL_R = 0;                                // turn-off PWM1 generator 0 (drives outs 0 and 1)


    PWM1_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 3 on PWM1, gen 1b, cmpb
    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 2 on PWM1, gen 1a, cmpa
    PWM1_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
                                                     // output 0 on PWM1, gen 0a, cmpa

    // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_1_LOAD_R = 1024; // generator 1
    PWM1_0_LOAD_R = 1024; // generator 0

    PWM1_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_1_CMPA_R = 0;                               // green off
    PWM1_0_CMPA_R = 0;                               // blue off

    PWM1_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 1
    PWM1_0_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 0

    PWM1_ENABLE_R = PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM0EN;
                                                     // enable outputs
}

void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_1_CMPB_R = red;
    PWM1_1_CMPA_R = green;
    PWM1_0_CMPA_R = blue;

}
