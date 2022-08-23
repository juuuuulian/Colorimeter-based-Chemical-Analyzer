/*
 * demo_embedded1.c
 *
 *  Created on: May 18, 2022
 *      Author: julian
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

// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

// Analog Light Sensor:
//  AIN0 (PE3)

// Motor:
//   (PB4) green wire
//   (PB5) black wire
//   (PE4) yellow
//   (PE5) white

// Tube positions:
// R: 194, 1: 28 , 2: 61
// 3: 94, 4: 127, 5: 161

//  timer input:
//   SIGNAL_IN on PD1 (WT2CCP1)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------


#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "uart0.h"
#include "wait.h"
#include "clock.h"
#include "adc_v0.h"
#include "rgb_led_v0.h"
#include "tm4c123gh6pm.h"
#include "stepperMotor.h"
#include "terminal_interface.h"

// PortD masks
#define TIME_IN_MASK 2

// Analog Mask
#define AIN0_MASK 8

#define NUM_MEASUREMENTS 8
#define SIZE 8


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------



// PWM LEDs
uint16_t pwm_r = 0;
uint16_t pwm_g = 0;
uint16_t pwm_b = 0;

float raw_red = 0;
float raw_green = 0;
float raw_blue = 0;

typedef struct { float r; float g; float b; float cl_val; float dist; } cl_t;

cl_t cl[8] =
{
    {3072.0, 3072.0, 3072.0, 0, 0.0},
    {3036.0, 3010.0, 2802.0, 0.5, 0.0},
    {3412.0, 3025.0, 2404.0, 1.0, 0.0},
    {3466.0, 2991.0, 2031.0, 2.0, 0.0},
    {3141.0, 3165.0, 1050.0, 3.0, 0.0},
    {3159.0, 2937.0, 800.0, 5.0, 0.0},
    {3455.0, 2939.0, 501.0, 6.0, 0.0},
    {3455.0, 2779.0, 210.0, 7.0, 0.0}
};


typedef struct { float r; float g; float b; float ph_val; float dist; } ph_t;

ph_t ph[8] =
{
    {3072.0, 3072.0, 3072.0, 0, 0.0},
    {3036.0, 3010.0, 800.0, 5.5, 0.0},
    {3412.0, 2500.0, 1000.0, 6.8, 0.0},
    {3466.0, 2250.0, 1250.0, 7.2, 0.0},
    {3141.0, 2000.0, 1600.0, 7.5, 0.0},
    {3159.0, 1500.0, 2000.0, 7.8, 0.0},
    {3455.0, 1000.0, 2200.0, 8.2, 0.0},
    {3455.0, 500.0, 2801.0, 9.0, 0.0}
};

bool validremote = false;
uint8_t count = 0;
uint32_t time[50];
uint8_t dataremote;
uint8_t dataNot;
uint8_t address;
uint8_t addressNot;
uint32_t code = 0;

//-----------------------------------------------------------------------------
// Subroutine
//-----------------------------------------------------------------------------


// squares a float
float sqrr(float a)
{
    return a * a;
}

// Just an absolute value func for floats
float absxf( float num)
{
    if (num < 0.0)
    {
        num *= (-1.0);
    }
    return num;
}

// calculates the distance for each rgb triplet
void distanceCalcPh( ph_t *ph, float red, float grn, float blu )
{
    int i;
    for (i = 0; i < SIZE; i++)
    {
        ph[i].dist = sqrr(red - ph[i].r) + sqrr(grn - ph[i].g) + sqrr(blu - ph[i].b);
        //printf("%.2f\n", cl[i].dist);
    }
}



// calculates the distance for each rgb triplet
void distanceCalc( cl_t *cl, float red, float grn, float blu )
{
    int i;
    for (i = 0; i < SIZE; i++)
    {
        cl[i].dist = sqrr(red - cl[i].r) + sqrr(grn - cl[i].g) + sqrr(blu - cl[i].b);
        //printf("%.2f\n", cl[i].dist);
    }
}

float phCalc (ph_t *ph)
{
    //char str[20];
    float low_first = 1111111111111.0;
    float low_second = 1111111111111.0;
    float delta = 0.0;
    int index_first= 0;
    int index_second = 0;

    float calc_ph;

    low_first = low_second;

    int i;

    // Finds the lowest two distances and records the index they're located at

    for (i = 0; i < SIZE; i++)
    {
        if ( absxf(ph[i].dist) < absxf(low_first))
        {
            low_second = low_first;
            index_second = index_first;
            low_first = ph[i].dist;
            index_first = i;
        }
        else if ( (ph[i].dist < low_second) && (ph[i].dist != low_first) )
        {
            low_second = ph[i].dist;
            index_second = i;
        }
    }

    delta = absxf (ph[index_second].ph_val - ph[index_first].ph_val);


    // equation: (x2-x1) * (d1/(d1+d2) + x1)

    calc_ph = delta * (low_first / (low_first + low_second) ) + ph[index_first].ph_val;

    return calc_ph;
}



float chlorineCalc (cl_t *cl)
{
    //char str[20];
    float low_first = 1111111111111.0;
    float low_second = 1111111111111.0;
    float delta = 0.0;
    int index_first= 0;
    int index_second = 0;

    float calc_chlorine;

    low_first = low_second;

    int i;

    // Finds the lowest two distances and records the index they're located at

    for (i = 0; i < SIZE; i++)
    {
        if ( absxf(cl[i].dist) < absxf(low_first))
        {
            low_second = low_first;
            index_second = index_first;
            low_first = cl[i].dist;
            index_first = i;
        }
        else if ( (cl[i].dist < low_second) && (cl[i].dist != low_first) )
        {
            low_second = cl[i].dist;
            index_second = i;
        }
    }

    delta = absxf (cl[index_second].cl_val - cl[index_first].cl_val);


    // equation: (x2-x1) * (d1/(d1+d2) + x1)

    calc_chlorine = delta * (low_first / (low_first + low_second) ) + cl[index_first].cl_val;

    return calc_chlorine;
}


void calibrate ()
{
    // sets tube position to R
    get_to_tube(0);

    pwm_r = 0;
    pwm_g = 0;
    pwm_b = 0;

    while (true)
    {
        waitMicrosecond(1000000);

        uint16_t duty_cycle = 0;    // PWM led
        uint16_t raw = 0;

        while (duty_cycle < 1024 && raw < 3072)
        {
            // Red
            setRgbColor(duty_cycle, 0, 0);
            waitMicrosecond(10000);
            raw = readAdc0Ss3();

            // Checks for an error
            if (duty_cycle == 1023 && raw < 3072)
            {
                // then we start over
                duty_cycle = 0;
            }

            duty_cycle++;
        }

        cl[0].r = (float) raw;

        waitMicrosecond(10000);
        pwm_r = duty_cycle;
        setRgbColor(0, 0, 0);

        raw = 0;
        duty_cycle = 0;

        //green
        while (duty_cycle < 1024 && raw < 3072)
        {
            setRgbColor(0, duty_cycle, 0);
            waitMicrosecond(10000);
            raw = readAdc0Ss3();

            // Checks for an error
            if (duty_cycle == 1023 && raw < 3072)
            {
                // then we start over
                duty_cycle = 0;
            }
            duty_cycle++;
        }

        cl[0].g = (float) raw;

        waitMicrosecond(10000);
        pwm_g = duty_cycle;
        setRgbColor(0, 0, 0);

        raw = 0;
        duty_cycle = 0;

        //blue
        while (duty_cycle < 1024 && raw < 3072)
        {
            setRgbColor(0, 0, duty_cycle);
            waitMicrosecond(10000);
            raw = readAdc0Ss3();

            // Checks for an error
            if (duty_cycle == 1023 && raw < 3072)
            {
                // then we start over
                duty_cycle = 0;
            }
            duty_cycle++;
        }

        cl[0].b = (float) raw;

        waitMicrosecond(10000);
        pwm_b = duty_cycle;
        setRgbColor(0, 0, 0);

        raw = 0;
        duty_cycle = 0;

        break;

    }
}

void measure (uint8_t tube, uint16_t *r, uint16_t *g, uint16_t *b)
{
    // instruct turret to rotate to desire location 0-5
    get_to_tube(tube);
    waitMicrosecond(1000000);

    // set RED led PWM to pwm_r  (green off blue off)
    // record the measured analog result in *r
    setRgbColor(pwm_r, 0, 0);
    waitMicrosecond(10000);

    *r = readAdc0Ss3();
    waitMicrosecond(1000);
    raw_red =(float) *r;

    setRgbColor(0, 0, 0);
    waitMicrosecond(1000);


    // Sets the green LED PWM to pwm_g (red and blue off)
    // records the measured analog result in *g.
    setRgbColor(0, pwm_g, 0);
    waitMicrosecond(100000);

    *g = readAdc0Ss3();
    waitMicrosecond(1000);
    raw_green =(float) *g;

    setRgbColor(0, 0, 0);
    waitMicrosecond(1000);

    // Sets the blue LED PWM to pwm_b (red and green off)
    // records the measured analog result in *b
    setRgbColor(0, 0, pwm_b);
    waitMicrosecond(100000);

    *b = readAdc0Ss3();
    waitMicrosecond(1000);
    raw_blue = (float) *b;

    setRgbColor(0, 0, 0);
    waitMicrosecond(1000);

}


// Initialize Hardware
void initAnalogHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure AIN0 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN0_MASK;           // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~AIN0_MASK;            // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= AIN0_MASK;           // turn on analog operation on pin PE3
}


void enableTimerMode()
{
    WTIMER2_CTL_R &= ~TIMER_CTL_TBEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter

    WTIMER2_TBMR_R = TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR;
    // configure for edge time mode, count up, Compare capture mode

    WTIMER2_CTL_R = TIMER_CTL_TBEVENT_NEG;           // measure time from NEGATIVE edge to NEGATIVE edge
    WTIMER2_IMR_R = TIMER_IMR_CBEIM;                 // turn-on interrupts
    WTIMER2_TBV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TBEN;                 // turn-on counter
    NVIC_EN3_R |= 1 << (INT_WTIMER2B-16-96);         // turn-on interrupt 114 (WTIMER2B)
}

// Initialize Hardware
void initHwTime()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;    // port D
    _delay_cycles(3);

    // Configure SIGNAL_IN for time measurements
    GPIO_PORTD_AFSEL_R |= TIME_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD1_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD1_WT2CCP1;
    GPIO_PORTD_DEN_R |= TIME_IN_MASK;                // enable bit 1 for digital input PD1

    enableTimerMode();
}


void wideTimer2Isr()
{
    // check and see about this bit

    if (WTIMER2_TBV_R > 68*40000)
    {
        count = 0;
    }

    if (count == 0)      // Received the first edge, then zero timer
    {
        WTIMER2_TBV_R = 0;
    }

    time[count] = WTIMER2_TBV_R;    // read counter input

    if (count == 0)
    {
        count++;
    }

    else if (count == 1)   // You are at the end of the 13.5ms Period
    {

        if ( ((time[1] - time[0]) >= 520000 ) && ((time[1] - time[0]) <= 560000))
        {
            count++;
        }

        else
        {
           count = 0;     // Not a valid start of IR command
        }
    }

    else if (count > 1)         // this is one of the 32 addr or data bits with time 2t or 4t
    {
        // 1.5T = 33750, 2.5T = 56250
        // 3.5T = 78750, 4.5T = 101250

        if ( ((time[count] - time[count - 1]) >= 33750 && (time[count] - time[count - 1]) <= 56250)
                || (time[count] - time[count - 1]) >= 78750 && (time[count] - time[count - 1]) <= 101250)
        {
            count++;
        }

        else
        {
            count = 0;
        }
    }

    WTIMER2_ICR_R = TIMER_ICR_CBECINT;           // clear interrupt flag

    if (count == 34)
    {
        count = 0;
        code = 0;
        uint32_t edgeTime = 0;              // time between subsequent edges
        uint8_t i = 1;                     // edge number

        for (i = 1; i < 33; i++)
        {
            edgeTime = time[i + 1] - time[i];
            if ( (edgeTime >= 33750) && (edgeTime <= 56250) )         // bit value is 0
            {
                code |= 0 << (i - 1);
            }
            else if ( (edgeTime >= 78750) && (edgeTime <= 101250) )   // bit value is 1
            {
                code |= 1 << (i - 1);
            }
            else
            {
                code = 0;
                validremote = false;
                break;
            }
        }

        address = (code & 0x000000FF) >> 0;
        addressNot = (code & 0x0000FF00) >> 8;
        dataremote = (code & 0x00FF0000) >> 16;
        dataNot = (code & 0xFF000000) >> 24;

        if (((uint8_t)(address & addressNot) == 0) && ((uint8_t)(dataremote & dataNot) == 0))
        {
            validremote = true;
        }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    // Initialize hardware
    char str0[20];
    uint8_t tubeNumber;
    float measuredChlorine = 0.0;
    float measuredPH = 0.0;

    uint16_t raw_r = 0;     // raw values for the measure command
    uint16_t raw_g = 0;
    uint16_t raw_b = 0;

    USER_DATA data;         // struct for parsing user input
    initHwTime();


    bool valid = false;     // Tests if the command received is a programmed command
    init_motor();

    initLED();

    initAnalogHw();


    initAdc0Ss3();
    setAdc0Ss3Mux(0);
    setAdc0Ss3Log2AverageCount(4);

    initUart0();


    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    char str1[30];
    // Calibrate the R G B (LEDs) against the clear tube.
    // This gets the PWM value where the light sensor
    // reads an analog value of 3072
    calibrate();


    // Endless loop performing multiple tasks
    while (true)
    {
        if (validremote == true)
        {
            validremote = false;

            switch (dataremote)
            {

                case 0x5C:
                    sprintf(str1, "key: 1, code: 0x%x\n", dataremote);
                    home();
                    putsUart0(str1);
                    break;
                case 0x5D:
                    sprintf(str1, "key: 2, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    calibrate();
                    break;
                case 0x41:
                    sprintf(str1, "key: 3, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    get_to_tube(0);
                    break;
                case 0x40:
                    sprintf(str1, "key: 4, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    get_to_tube(1);
                    break;
                case 0x58:
                    sprintf(str1, "key: 5, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    get_to_tube(2);
                    break;
                case 0x59:
                    sprintf(str1, "key: 6, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    get_to_tube(3);
                    break;
                case 0x45:
                    sprintf(str1, "key: 7, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    get_to_tube(4);
                    break;
                case 0x44:
                    sprintf(str1, "key: 8, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    get_to_tube(5);
                    break;

                case 0x54:
                    sprintf(str1, "key: 9, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(0, &raw_r, &raw_g, &raw_b);
                    sprintf(str0, "%4u, %4u, %4u\n", raw_r, raw_g, raw_b);
                    putsUart0(str0);
                    break;
                case 0x55:
                    sprintf(str1, "key: 10, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(1, &raw_r, &raw_g, &raw_b);
                    sprintf(str0, "%4u, %4u, %4u\n", raw_r, raw_g, raw_b);
                    putsUart0(str0);
                    break;
                case 0x49:
                    sprintf(str1, "key: 11, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(2, &raw_r, &raw_g, &raw_b);
                    sprintf(str0, "%4u, %4u, %4u\n", raw_r, raw_g, raw_b);
                    putsUart0(str0);
                    break;
                case 0x48:
                    sprintf(str1, "key: 12, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(3, &raw_r, &raw_g, &raw_b);
                    sprintf(str0, "%4u, %4u, %4u\n", raw_r, raw_g, raw_b);
                    putsUart0(str0);
                    break;
                case 0x50:
                    sprintf(str1, "key: 13, code: 0x%x\n", dataremote);
                    putsUart0(str1);

                    measure(4, &raw_r, &raw_g, &raw_b);
                    sprintf(str0, "%4u, %4u, %4u\n", raw_r, raw_g, raw_b);
                    putsUart0(str0);
                    break;
                case 0x51://key 14
                    sprintf(str1, "key: 14, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(5, &raw_r, &raw_g, &raw_b);
                    sprintf(str0, "%4u, %4u, %4u\n", raw_r, raw_g, raw_b);
                    putsUart0(str0);
                    break;
                case 0x4D://key 15
                    sprintf(str1, "key: 15, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(0, &raw_r, &raw_g, &raw_b);

                    distanceCalc( cl, raw_red, raw_green, raw_blue);
                    measuredChlorine = chlorineCalc(cl);

                    sprintf(str0, "Chlorine: %.2f\n", measuredChlorine);
                    putsUart0(str0);
                    break;
                case 0x4C://key 16
                    sprintf(str1, "key: 16, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(1, &raw_r, &raw_g, &raw_b);

                    distanceCalc( cl, raw_red, raw_green, raw_blue);
                    measuredChlorine = chlorineCalc(cl);

                    sprintf(str0, "Chlorine: %.2f\n", measuredChlorine);
                    putsUart0(str0);
                    break;
                case 0x1C://key 17
                    sprintf(str1, "key: 17, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(2, &raw_r, &raw_g, &raw_b);

                    distanceCalc( cl, raw_red, raw_green, raw_blue);
                    measuredChlorine = chlorineCalc(cl);

                    sprintf(str0, "Chlorine: %.2f\n", measuredChlorine);
                    putsUart0(str0);
                    break;
                case 0x1D://key 18
                    sprintf(str1, "key: 18, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(3, &raw_r, &raw_g, &raw_b);

                    distanceCalc( cl, raw_red, raw_green, raw_blue);
                    measuredChlorine = chlorineCalc(cl);

                    sprintf(str0, "Chlorine: %.2f\n", measuredChlorine);
                    putsUart0(str0);
                    break;
                case 0x1E://key 19
                    sprintf(str1, "key: 19, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(4, &raw_r, &raw_g, &raw_b);

                    distanceCalc( cl, raw_red, raw_green, raw_blue);
                    measuredChlorine = chlorineCalc(cl);

                    sprintf(str0, "Chlorine: %.2f\n", measuredChlorine);
                    putsUart0(str0);
                    break;
                case 0x1F://key 20
                    sprintf(str1, "key: 20, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(5, &raw_r, &raw_g, &raw_b);

                    distanceCalc( cl, raw_red, raw_green, raw_blue);
                    measuredChlorine = chlorineCalc(cl);

                    sprintf(str0, "Chlorine: %.2f\n", measuredChlorine);
                    putsUart0(str0);
                    break;
                case 0x18://key 21
                    sprintf(str1, "key: 21, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(0, &raw_r, &raw_g, &raw_b);

                    distanceCalcPh( ph, raw_red, raw_green, raw_blue);
                    measuredPH = phCalc(ph);

                    sprintf(str0, "Ph: %.2f\n", measuredPH);
                    putsUart0(str0);
                    break;
                case 0x19://key 22
                    sprintf(str1, "key: 22, code: 0x%x\n", dataremote);
                    putsUart0(str1);

                    measure(1, &raw_r, &raw_g, &raw_b);

                    distanceCalcPh( ph, raw_red, raw_green, raw_blue);
                    measuredPH = phCalc(ph);

                    sprintf(str0, "Ph: %.2f\n", measuredPH);
                    putsUart0(str0);
                    break;
                case 0x1A://key 23
                    sprintf(str1, "key: 23, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(2, &raw_r, &raw_g, &raw_b);

                    distanceCalcPh( ph, raw_red, raw_green, raw_blue);
                    measuredPH = phCalc(ph);

                    sprintf(str0, "Ph: %.2f\n", measuredPH);
                    putsUart0(str0);
                    break;
                case 0x1B://key 24
                    sprintf(str1, "key: 24, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(3, &raw_r, &raw_g, &raw_b);

                    distanceCalcPh( ph, raw_red, raw_green, raw_blue);
                    measuredPH = phCalc(ph);

                    sprintf(str0, "Ph: %.2f\n", measuredPH);
                    putsUart0(str0);
                    break;
                case 0x14://key 25
                    sprintf(str1, "key: 25, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(4, &raw_r, &raw_g, &raw_b);

                    distanceCalcPh( ph, raw_red, raw_green, raw_blue);
                    measuredPH = phCalc(ph);

                    sprintf(str0, "Ph: %.2f\n", measuredPH);
                    putsUart0(str0);
                    break;
                case 0x15://key 26
                    sprintf(str1, "key: 26, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    measure(5, &raw_r, &raw_g, &raw_b);

                    distanceCalcPh( ph, raw_red, raw_green, raw_blue);
                    measuredPH = phCalc(ph);

                    sprintf(str0, "Ph: %.2f\n", measuredPH);
                    putsUart0(str0);
                    break;
                case 0x16://key 27
                    sprintf(str1, "key: 27, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x17://key 28
                    sprintf(str1, "key: 28, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x10://key 29
                    sprintf(str1, "key: 29, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x11://key 30
                    sprintf(str1, "key: 30, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x12://key 31
                    sprintf(str1, "key: 31, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x13://key 32
                    sprintf(str1, "key: 32, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x0C://key 33
                    sprintf(str1, "key: 33, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x0D://key 34
                    sprintf(str1, "key: 34, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x0E://key 35
                    sprintf(str1, "key: 35, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x0F://key 36
                    sprintf(str1, "key: 36, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x08://key 37
                    sprintf(str1, "key: 37, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x09://key 38
                    sprintf(str1, "key: 38, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x0A://key 39
                    sprintf(str1, "key: 39, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x0B://key 40
                    sprintf(str1, "key: 40, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x04://key 41
                    sprintf(str1, "key: 41, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x05://key 42
                    sprintf(str1, "key: 42, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x06://key 43
                    sprintf(str1, "key: 43, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
                case 0x07://key 44
                    sprintf(str1, "key: 44, code: 0x%x\n", dataremote);
                    putsUart0(str1);
                    break;
            }
        }
        if (kbhitUart0())
        {
            getsUart0(&data);

            parseFields(&data);

            // command evaluation

            // set address, data -> address and data are integers
            if (isCommand(&data, "set", 2))
            {
                int32_t address = getFieldInteger( &data, 1 );
                int32_t test_data = getFieldInteger( &data, 2 );
                valid = true;
            }

            // Command to set motor to home position
            if ( isCommand( &data, "home", 0 ) )
            {
                home();
                valid = true;
            }

            if ( isCommand(&data, "tube", 1) )
            {
                tubeNumber = getFieldInteger( &data, 1 );
                get_to_tube( tubeNumber );
                valid = true;
            }

            if (isCommand(&data, "measure", 3))
            {
                char *str1 = getFieldString( &data, 1 );  // string for "tube"
                tubeNumber = getFieldInteger( &data, 2 ); // holds the tube selected
                char *str2 = getFieldString( &data, 3 );  // string for "raw" or "Cl" or "pH"

                // Goes to the selected tube number
                //get_to_tube(tubeNumber);

                if ( strCmp( "raw", str2 ) == 0 && strCmp( "tube", str1 ) == 0 )
                {
                    measure(tubeNumber, &raw_r, &raw_g, &raw_b);

                    sprintf(str0, "%4u, %4u, %4u\n", raw_r, raw_g, raw_b);
                    putsUart0(str0);
                    valid = true;
                }

                else if ( ( strCmp( "Cl", str2 ) == 0 ) && ( strCmp( "tube", str1 ) == 0 ) )
                {
                    measure(tubeNumber, &raw_r, &raw_g, &raw_b);

                    distanceCalc( cl, raw_red, raw_green, raw_blue);
                    measuredChlorine = chlorineCalc(cl);

                    sprintf(str0, "Chlorine: %.2f\n", measuredChlorine);
                    putsUart0(str0);
                    valid = true;
                }

                else if ( ( strCmp( "ph", str2 ) == 0 ) && ( strCmp( "tube", str1 ) == 0 ) )
                {
                    measure(tubeNumber, &raw_r, &raw_g, &raw_b);

                    distanceCalcPh( ph, raw_red, raw_green, raw_blue);
                    measuredPH = phCalc(ph);

                    sprintf(str0, "ph: %.2f\n", measuredPH);
                    putsUart0(str0);
                    valid = true;
                }

            }

            else if (isCommand (&data, "calibrate", 0))
            {
                get_to_tube(0); // go to reference tube
                calibrate();
                sprintf(str0, "PWM: %4u, %4u, %4u\n", pwm_r, pwm_g, pwm_b);
                putsUart0(str0);
                valid = true;
            }

            // process other commands here
            // look for error

            if (!valid)
            {
                putsUart0("Invalid command\n");
            }
        }

    }
}



