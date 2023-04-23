/*
 * options.c
 *
 *  Created on: Apr 9, 2022
 *      Author: dmainz
 */

#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "options.h"

typedef struct option option_t;
#define NUM_OPTS 6
#define YES  1
#define NO   0
#define NONE 0

static const option_t options[NUM_OPTS+1] = {
//        option   args?    flag   val
		{ "ip",     YES,    NONE,  'i' },
		{ "lport",  YES,    NONE,  'l' },
		{ "mcport", YES,    NONE,  'm' },
		{ "fsport", YES,    NONE,  'f' },
		{ "dport",  YES,    NONE,  'd' },
		{ 0,       0,      0,      0 }
};

opts userOpts = { false, "", false, 0 };

int get_args( int argc, char *argv[] ) {
	// optind - index into argv of the next argument
	// optarg - argument value, 0 for no value
	// longopts - array of long options captured, indexed by idx.
	// optopt  - latest opt parsed
	// opterr  - print error when set to 1
	int idx;
	int result = 0, ret = 0;
	while( ret >= 0) {

		ret = getopt_long( argc, argv, "i:l:m:f:d:", options, &idx );

		switch( ret ) {
		case 'i':
			if( optarg != 0 ) {
				memcpy( userOpts.ipAddr, optarg, strlen(optarg) );
				userOpts.ipValid = true;
				result ++;
				//printf(" IP is %s\n", optarg );
			} else {
				printf(" -i, --ip requires one argument.\n");
				result = -1;
			}
			break;
		case 'l':
			if( optarg != 0 ) {
				userOpts.listenPort = (uint16_t) atoi(optarg);
				userOpts.listenPortValid = true;
				result ++;
			} else {
				printf(" -l, --lport requires one argument.\n");
				result = -1;
			}
			break;
		case 'm':
			if( optarg != 0 ) {
				userOpts.mcCmdPort = (uint16_t) atoi(optarg);
				userOpts.mcCmdPortValid = true;
				result ++;
			} else {
				printf(" -m, --mcport requires one argument.\n");
				result = -1;
			}
			break;
		case 'f':
			if( optarg != 0 ) {
				userOpts.fsCmdPort = (uint16_t) atoi(optarg);
				userOpts.fsCmdPortValid = true;
				result ++;
			} else {
				printf(" -f, --fsport requires one argument.\n");
				result = -1;
			}
			break;
		case 'd':
			if( optarg != 0 ) {
				userOpts.dataPort = (uint16_t) atoi(optarg);
				userOpts.dataPortValid = true;
				result ++;
			} else {
				printf(" -d, --dport requires one argument.\n");
				result = -1;
			}
			break;
		default:
			if( ret != -1 ) {
				printf("Did not recognize option %c\n",ret);
				result = -1;
			}
		}
	}
	return result;
}

int test_get_args( int argc, char *argv[] ) {
//	printf("Getting options. argv[1] = %s\n", argv[1]);
	int ret = get_args( argc, argv );
//	printf("Done with options. ret = %d\n", ret);
	return ret;
}
