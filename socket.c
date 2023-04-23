/*
 * socket.c
 *
 *  Created on: Apr 4, 2022
 *      Author: dmainz
 */

/*** To assign a source port to the client use bind ***
 *** Example:
 struct sockaddr_in sin;

if (-1==(sock=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP))) return 0;
memset(&sin,0,sizeof(sin));
sin.sin_family=AF_INET;
sin.sin_port=htons(fromport);
sin.sin_addr.s_addr=INADDR_ANY;
if (-1==bind(sock,(struct sockaddr *)&sin,sizeof(struct sockaddr_in))) return 0;
***/

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/errno.h>

#define UDP 17

int main() {
	int sock = socket(PF_INET, SOCK_DGRAM, UDP);
	if( sock < 0 ) {
		printf("Socket returned an error: %d\n",sock);
		return sock;
	}
	struct in_addr inaddr;
	inet_pton( AF_INET, "192.168.1.199", &inaddr.s_addr );

#if defined(__APPLE__)
    struct sockaddr_in  dest_addr = { 0, AF_INET, htons(1024), inaddr };
    dest_addr.sin_len = sizeof(dest_addr);
#else
    struct sockaddr_in  dest_addr = { AF_INET, htons(1024), inaddr };
#endif

	char message[20] = "";
	char response[20];

	printf("sock = %d, addr = %x\n", sock, (uint32_t)(dest_addr.sin_addr.s_addr));

	while( message[0] != 'X' ) {
		printf("> ");
		scanf("%s", message);
		ssize_t sendresp = sendto(sock, (const char *)message, strlen(message), 0, (const struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if( sendresp < 0 ) {
			printf("sendto returned an error: %d\n",errno);
			return sendresp;
		}

		socklen_t len;
		int rcv = recvfrom(sock, (char *)response, 19, MSG_WAITALL, (struct sockaddr *) &dest_addr, &len);
		if( rcv > 0 ) {
			printf("%s\n",response);
		}
	}
	close(sock);
	printf("Done\n");
	return 0;
}
