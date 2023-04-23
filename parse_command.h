/*
 * parse_command.h
 *
 *  Created on: Apr 22, 2022
 *      Author: dmainz copied Uday's code in here to make it modular
 */

#ifndef PARSE_COMMAND_H_
#define PARSE_COMMAND_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_CHARS 128
#define MAX_FIELDS 12

// structure for parse field
typedef struct _USER_DATA
{
  char buffer[MAX_CHARS+1];
  uint8_t fieldCount;
  uint8_t fieldPosition[MAX_FIELDS];
  char fieldType[MAX_FIELDS];
} USER_DATA;

void parseFields(struct _USER_DATA *data);
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);
char* getFieldString(USER_DATA* data, uint8_t fieldNumber);

#endif /* PARSE_COMMAND_H_ */
