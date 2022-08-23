/*
 * terminal_interface.c
 *
 *  Created on: Apr 11, 2022
 *      Author: Julian Schneider
 */

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "stepperMotor.h"
#include "terminal_interface.h"
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"


// UI information
#define MAX_CHARS 80
#define MAX_FIELDS 5

// Struct for holding parsed data from user
//typedef struct _USER_DATA
//{
//    char buffer[MAX_CHARS + 1];
//    uint8_t fieldCount;
//    uint8_t fieldPosition[MAX_FIELDS];
//    char fieldType[MAX_FIELDS];
//} USER_DATA;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

/*
   Function to receive chars from the UI, processing special chars such as backspace
   and writing the resultant string into the buffer
*/

void getsUart0 (USER_DATA *data)
{
    char c;
    uint8_t count = 0;

    while (true)
    {
        c = getcUart0();                                 // get a char and put in buffer
        if ((c == 8 || c == 127) && (count > 0))         // remove backspace char and check if backspace is the first char
        {
            count--;
        }
        else if (c == 13 || c == 10)                     // check if <enter key> was pressed
        {
            data->buffer[count] = '\0';
            break;
        }
        else if (c >= 32)                                // check if <space> or any printable char is pressed
        {
            data->buffer[count++] = c;

            if (count == MAX_CHARS)                      // program will exit if max char are input
            {
                data->buffer[count] = '\0';
                break;
            }
        }
    }
}

void parseFields (USER_DATA *data)
{
    // alpha A : 65 , Z : 90 , a : 97 , z : 122
    // numeric 0 : 48 , 9 : 57
    // everything else is a delimiter

    char previous = 'd';

    data->fieldCount = 0;

    uint8_t count = 0;
    uint8_t index = 0;

    while ( data->buffer[count] != NULL )
    {
        char c = data->buffer[count];

        // exit the loop if we already have our max fields
        if ( data->fieldCount == MAX_FIELDS )
        {
            break;
        }

        // check if it's an alpha
        //&& ( (previous == 'd') || (previous == 'n') )
        else if ( ( (c >= 65 && c <= 90) || (c >= 97 && c <= 122) ) )
        {
            if (previous == 'a')
            {
                count++;
                continue;
            }
            data->fieldCount++;
            data->fieldType[index] = 'a';
            data->fieldPosition[index++] = count;
            previous = 'a';
        }

        //check if its numeric
        // || (previous == 'a')&& ( (previous == 'd') )
        else if ( (c >= 48 && c <= 57)|| (c == 45) )
        {
            if (previous == 'n')
            {
                count++;
                continue;
            }
            data->fieldCount++;
            data->fieldType[index] = 'n';
            data->fieldPosition[index++] = count;
            previous = 'n';
        }

        // otherwise, it's a delimiter
        else
        {
            previous = 'd';
            data->buffer[count] = '\0';
        }

        count++;
    }
}

// Returns the value of a field requested if the field
// is in range or NULL otherwise.
// returns the address of
char* getFieldString (USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber <= data->fieldCount)
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return NULL;
    }
}

// function to return the integer value of the field if the
// field number is in range and the field type is numeric or 0 otherwise.
int32_t getFieldInteger (USER_DATA *data, uint8_t fieldNumber)
{
    if ( (fieldNumber <= data->fieldCount) && (data->fieldType[fieldNumber] == 'n') )
    {
        return atoi( &data->buffer[ data->fieldPosition[ fieldNumber ] ] );
    }
    else
    {
        return 0;
    }
}

// compares two strings and return <0, 0, or >0
int strCmp (const char *s1, const char *s2)
{
    while ( *s1 && ( *s1 == *s2 ) )
    {
        s1++;
        s2++;
    }
    return *(const unsigned char*)s1 - *(const unsigned char*)s2;
}

// function which returns true if the command matches the first field
// and the number of arguments (excluding the command field) is greater
// than or equal to the requested number of minimum arguments.
bool isCommand (USER_DATA *data, const char strCommand[], uint8_t minArguements)
{
    uint8_t fieldNums = data->fieldCount;

    if ( fieldNums-1 >= minArguements && ( strCmp(data->buffer, strCommand) == 0 ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

