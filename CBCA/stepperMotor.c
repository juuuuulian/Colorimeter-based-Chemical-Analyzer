// Julian Schneider
// stepperMotor.c

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for sprintf)


// Motor:
//   (PB4) green wire
//   (PB5) black wire
//   (PE4) yellow
//   (PE5) white

// Tube positions:
// R: 197
// 1: 31
// 2: 64
// 3: 96
// 4: 129
// 5: 163

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include "wait.h"
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "stepperMotor.h"

// Bit-band aliases
#define M_WHITE       (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))
#define M_YELLOW      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))
#define M_BLACK       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define M_GREEN       (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))

// Wire Masks
#define M_YELLOW_MASK 16
#define M_BLACK_MASK 32
#define M_WHITE_MASK 32
#define M_GREEN_MASK 16

// Position Masks
#define TUBE_0_POSITION 194
#define TUBE_1_POSITION 28
#define TUBE_2_POSITION 61
#define TUBE_3_POSITION 94
#define TUBE_4_POSITION 127
#define TUBE_5_POSITION 161

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

uint8_t position = 0;       // Current position of the motor
uint8_t phase = 0;          // Current electrical phase the motor is in
uint8_t inputPhase = 0;     // Phase to be applied on the motor
uint8_t homePosition = 0;   // The position of the R tube

//-----------------------------------------------------------------------------
// Subroutines
//--------------------------------------------------------- --------------------

// Should be called at power up. It calls stepCw() enough times to ensure
// the motor moves to the stop position closest to the position of the
// where the reference vial (R) is aligned with the optical path.

void init_motor()
{
    initSystemClockTo40Mhz(); //REMOVE
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    // setting gpios as outputs
    GPIO_PORTE_DIR_R |= M_YELLOW_MASK | M_WHITE_MASK;
    GPIO_PORTB_DIR_R |= M_BLACK_MASK | M_GREEN_MASK;

    GPIO_PORTE_DR2R_R |= M_YELLOW_MASK | M_WHITE_MASK;
    GPIO_PORTB_DR2R_R |= M_BLACK_MASK | M_GREEN_MASK;

    // enable motor outputs and pushbuttons
    GPIO_PORTE_DEN_R |= M_YELLOW_MASK | M_WHITE_MASK;
    GPIO_PORTB_DEN_R |= M_BLACK_MASK | M_GREEN_MASK;

    home();
}

// depending on the tube selected this will turn the stepper
// to the location determined by the the Defined Tube Positions
void get_to_tube(uint8_t tube)
{
    switch (tube)
    {
        case 0:
            set_position(TUBE_0_POSITION);
            break;

        case 1:
            set_position(TUBE_1_POSITION);
            break;

        case 2:
            set_position(TUBE_2_POSITION);
            break;

        case 3:
            set_position(TUBE_3_POSITION);
            break;

        case 4:
            set_position(TUBE_4_POSITION);
            break;

        case 5:
            set_position(TUBE_5_POSITION);
            break;

        default:
            set_position(TUBE_0_POSITION);
    }
}

// function called at start up which set the motor position to tube R
void home()
{
    int i;
    position = 0;
    for (i = 0; i < 200; i++)
    {
        step_CW();

    }
    for (i = 0; i < 3; i++)
    {
        step_CCW();

    }
    homePosition = position;
}

// steps in either direction until the current position
// is equal to the desired position.
void set_position(uint8_t desiredPosition)
{
    while (position < desiredPosition)
    {
        step_CW();
    }

    while (position > desiredPosition)
    {
        step_CCW();
    }
}

// Steps motor in clockwise direction by one step.
// Also updates the position and sets the desired phase
void step_CW()
{
    waitMicrosecond(12000);
    inputPhase = 0;
    position++;
    inputPhase = (phase + 1) % 4;
    apply_phase(inputPhase);
}

// Steps motor in counter-clockwise direction by one step.
// Also updates the position and sets the desired phase
void step_CCW()
{
    inputPhase = 0;
    waitMicrosecond(12000);

    if (phase == 0)
    {
        inputPhase = 3;
    }
    else
    {
        inputPhase = (phase - 1) % 4;
    }
    position--;
    apply_phase(inputPhase);
}

// Depending on the desired phase it energizes certain coils
void apply_phase(uint8_t input)
{
    switch (input)
    {
        case 0: // 1+ 2- 3+ 4+
            M_BLACK = 1;
            M_WHITE = 0;
            M_YELLOW = 0;
            M_GREEN = 0;
            break;

        case 1: // 1- 2- 3+ 4-
            M_BLACK = 0;
            M_WHITE = 0;
            M_YELLOW = 1;
            M_GREEN = 0;
            break;

        case 2: // 1- 2+ 3- 4-
            M_BLACK = 0;
            M_WHITE = 1;
            M_YELLOW = 0;
            M_GREEN = 0;
            break;

        case 3: // 1- 2- 3- 4+
            M_BLACK = 0;
            M_WHITE = 0;
            M_YELLOW = 0;
            M_GREEN = 1;
            break;
    }
    phase = input;
}

