/*
 * cli.c
 *
 *  Created on: Apr 14, 2022
 *      Author: dmainz
 */

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "cli.h"

#define NUM_CMDS 19

void disable_term_canonical(termios_t *old, termios_t *new) {
	tcgetattr( STDIN_FILENO, old ); // save current stdin settings
	*new = *old;                    // copy to new
    // turn off canonical mode to disable line buffering. Turn off echo to control what's printed.
	new->c_lflag &= ~(ICANON | ECHO);
	tcsetattr( STDIN_FILENO, TCSANOW, new);  // make change immediate
}

void restore_term( termios_t *old ) {
	// Return terminal settings to the original values
	tcsetattr( STDIN_FILENO, TCSANOW, old );
}

commands cli_cmds[NUM_CMDS] = {
		//command    args code  usage text
		{ "help",      0, 0,    "prints accepted commands" },
		{ "ls",        0, 0,    "send ls to shell" },
		{ "cd",        0, 0,    "send cd to shell" },
		{ "pwd",       0, 0,    "send pwd to shell" },
		{ "quit",      0, 0,    "quit" },
		{ "start",     0, 1,    "Tell CNC machine to start milling" },
		{ "stop",      0, 0xb,  "Tell CNC machine to stop milling" },
		{ "pause",     0, 2,    "Tell CNC machine to pause" },
		{ "calibrate", 0, 5,    "Tell CNC machine to do calibration" },
		{ "jog",       3, 6,    "Move CNC head to x y z position" },
		{ "status",    0, 7,    "Get status from CNC machine" },
		{ "upload",    1, 9,    "Upload a file to CNC machine" },
		{ "setactive", 0, 3,    "Tell CNC machine which file to work on" },
		{ "dir",       0, 0xa,  "Get a directory listing from CNC machine" },
		{ "resume",    0, 0xc,  "Resume after a pause" },
		{ "arcto",     3, 0xd,  "Move the CNC head in an arc: Xxval Yyval a|c" },
		{ "sbt",       1, 0xe,  "Set Board Thickness: number in mm" },
        { "home",      0, 0xf,  "Home the CNC head" },
		{ 0, 0, 0, 0 }
};

int complete_cmd( char *command ) {
	int found = 0, i = 0, onematch;
	int len = strlen(command);

	while( cli_cmds[i].cmd ) {
		if( ! strncmp( cli_cmds[i].cmd, command, len )) {
			found++;
			if( found == 1 ) onematch = i;
		}
		i++;
	}

	if( found == 1 ) {
		strcpy( command, cli_cmds[onematch].cmd );
	}

	return found;
}

void get_matches( char *command, char *matches ) {
	int i = 0;
	int len = strlen(command);

	while( cli_cmds[i].cmd ) {
		if( ! strncmp( cli_cmds[i].cmd, command, len )) {
			if( i == 0 ) sprintf( matches, "%s", cli_cmds[i].cmd );
			else  sprintf( matches, "%s %s", matches, cli_cmds[i].cmd );
		}
		i++;
	}
	sprintf( matches, "%s", matches );
}

void usage( char *cmd ) {
	int i = 0;

	if( cmd == NULL ) {
		printf("Press tab to complete.  Two tabs show all candidates.\n");
		while( cli_cmds[i].cmd ) {
			printf("%s ", cli_cmds[i++].cmd);
		}
		printf("\n");
	} else {
		while( cli_cmds[i].cmd && strcmp(cmd, cli_cmds[i].cmd ) ) { i++; };
		if( ! cli_cmds[i].cmd ) printf("Unrecognized command: %s.\n", cmd );
		else printf("%s: %s\n", cmd, cli_cmds[i].usage );
	}
}

int cli( char line[MAX_CLI_CHARS] ) {
	const char prompt[6] = "cnc> ";
	bool quit = false;
	char command[ARGS][ARGSIZE];

	memset((char *)(command[0]), 0, ARGSIZE);

	int i = 0, j = 0, tabcount = 0, f = 0;
	int charcount = 0, spacecount = 0;

	printf(prompt);
	while( !quit ) {
		int c = getchar();
		switch(c) {
		case '\t':
			if( i == 0 ) {
				if( tabcount == 0 ) {
					f = complete_cmd(command[i]);  // puts found command in command[i] if f = 1
					if( f > 1 ) {
						tabcount++;
					} else if ( f == 1 ) {
						for( int k = 0; k < j; k++ ) {
							putchar('\b');
							charcount --;
						}
						printf("%s", command[i]);
						j = strlen(command[i]);
						for( int k = 0; k < j; k++ ) {
							line[charcount ++] = command[i][k];
						}
					}

				} else {
					char matches[MATCHLEN] = "";
					get_matches(command[i], matches);
					printf("\n%s\n", matches);
					printf(prompt);
					for( int k = 0; k < i; k++ )
						printf("%s ", command[k]);
					printf("%s", command[i]);
					tabcount = 0;
				}
			}
			break;
		case ' ':
			if( spacecount == 0 && j > 0) {
				i++;
				j = 0;
				memset((char *)(command[i]), 0, ARGSIZE);
			}
			putchar(c);
			line[charcount] = ' ';
			tabcount = 0;
			spacecount ++;
			charcount ++;
			break;
		case 127:
		case '\b':
			if( charcount > 0 ) {
				if( j > 0 ) {
					j --;
				} else if( line[charcount-1] != ' ' && i > 0 ) {
					command[i][j] = 0;
					i --;
					j = strlen(command[i]) - 1;
					spacecount = 0;
				}
				command[i][j] = 0;
				line[charcount] = 0;
				charcount --;
                                putchar('\b');
                                putchar(' ');
                                putchar('\b');
			} else if( charcount == 0 ) {
				command[0][0] = 0;
			}
			break;
		case '\n':
			quit = 1;
			if( j > 0 ) i++;
			break;
		default:
			if( j < ARGSIZE && i < ARGS) {
				command[i][j++] = c;
				putchar(c);
				line[charcount] = c;
				charcount ++;
				tabcount = 0;
				spacecount = 0;
			}
		}
	}
	return i;
}
