/*
 * options.h
 *
 *  Created on: Apr 18, 2022
 *      Author: dmainz
 */

#ifndef OPTIONS_H_
#define OPTIONS_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct _opts {
	bool ipValid;
	char ipAddr[16];
	bool listenPortValid;
	uint16_t listenPort;
	bool mcCmdPortValid;
	uint16_t mcCmdPort;
	bool fsCmdPortValid;
	uint16_t fsCmdPort;
	bool dataPortValid;
	uint16_t dataPort;
} opts;

int get_args( int argc, char *argv[] );

extern opts userOpts;

#endif /* OPTIONS_H_ */
