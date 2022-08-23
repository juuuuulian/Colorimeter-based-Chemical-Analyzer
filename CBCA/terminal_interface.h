/*
 * terminal_interface.h
 *
 *  Created on: Apr 11, 2022
 *      Author: julian
 */

#ifndef TERMINAL_INTERFACE_H_
#define TERMINAL_INTERFACE_H_


// UI information
#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

/* initialize onboard-LEDs for visual feedback */
//void initTerminal();

/* Parsing Functions */
void getsUart0 (USER_DATA *data);
void parseFields (USER_DATA *data);
char* getFieldString (USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger (USER_DATA *data, uint8_t fieldNumber);

/* Verification functions */
int strCmp (const char *s1, const char *s2);
bool isCommand (USER_DATA *data, const char strCommand[], uint8_t minArguements);

/* Terminal Loop */
//int terminal_loop(void);

#endif /* TERMINAL_INTERFACE_H_ */
