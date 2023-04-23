/*
 * cli.h
 *
 *  Created on: Apr 14, 2022
 *      Author: dmainz
 */

#ifndef CLI_H_
#define CLI_H_
#include <termios.h>

typedef struct termios termios_t;

typedef struct cmds {
	char *cmd;
	int  num_args;
	int  cmd_code;
	char *usage;
} commands;

#define ARGS 4
#define ARGSIZE 128
#define MATCHLEN 128
#define MAX_CLI_CHARS 512

int cli( char line[MAX_CLI_CHARS] );
void disable_term_canonical(termios_t *old, termios_t *new);
void restore_term( termios_t *old );
void usage( char *cmd );

#endif /* CLI_H_ */
