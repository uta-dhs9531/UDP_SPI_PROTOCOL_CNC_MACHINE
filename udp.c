// UDP Library
// Computer Engineering IoT/Networks Course
// Jason Losh

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdlib.h>          // EXIT_ codes
#include <stdbool.h>         // bool
#include <stdio.h>           // printf, scanf
#include <string.h>          // memset, strlen, strcmp

#include <unistd.h>          // close
#include <sys/socket.h>      // socket, bind, recvfrom, sendto, setsockopt
#include <netinet/in.h>      // in_addr
#include <arpa/inet.h>       // inet_aton, inet_ntoa
#include <errno.h>

#include "udp.h"

//-----------------------------------------------------------------------------
// Local statics
//-----------------------------------------------------------------------------

int listen_sockfd;
struct in_addr listen_remote_ip;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// Sends UDP package containing a string to ipv4Address::port
// returns true if successful
bool sendData(const char ipv4Address[], int port, const char str[])
{
    struct sockaddr_in remote_addr;
    struct in_addr remote_ip;

    int sockfd;
    int count;
    bool ok;

    ok = (sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != -1;
    if (ok)
    {
        inet_aton(ipv4Address, &remote_ip);

        remote_addr.sin_family = AF_INET;
        remote_addr.sin_port = htons(port);
        remote_addr.sin_addr = remote_ip;
        memset(&(remote_addr.sin_zero), 0, 8);

        count = sendto(sockfd, str, strlen(str)+1, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
        ok = count > 0;
        close(sockfd);
    }
    return ok;
}

// Opens listener port that accepts only data from given address
int openListenerPort(const char ipv4Address[], int listenPort)
{
    struct sockaddr_in local_addr, get_addr;
    int opt = 1;
    bool ok = true;
    socklen_t len = sizeof(struct sockaddr_in);

    // Store expected source of messages
    inet_aton(ipv4Address, &listen_remote_ip);

    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(listenPort);
    local_addr.sin_addr.s_addr = INADDR_ANY;
    memset(&(local_addr.sin_zero), 0, 8);
    ok = (listen_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) != -1;
    if (ok)
        ok = setsockopt(listen_sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) != -1;
    if (ok)
        ok = setsockopt(listen_sockfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)) != -1;
    if (ok)
    	ok = bind(listen_sockfd, (struct sockaddr *)&local_addr, sizeof(struct sockaddr)) != -1;
    if (ok)
    	ok = getsockname(listen_sockfd, (struct sockaddr *)&get_addr, &len) != -1;
    if (ok)
    	printf(" bound port = %d\n", htons(get_addr.sin_port) );
    else {
    	printf(" Error during bind.  errno = %d\n", errno);
    }
    return ok;
}

int receiveData(char str[], int str_length)
{
    // Listener
    struct sockaddr_in sender_addr;
    unsigned int sender_add_length = sizeof(struct sockaddr);
    int count;
    bool received = false;

    while (!received)
    {
        count = recvfrom(listen_sockfd, str, str_length, 0, (struct sockaddr *)&sender_addr, &sender_add_length);
        if (count > 0)
            received = strcmp(inet_ntoa(sender_addr.sin_addr), inet_ntoa(listen_remote_ip)) == 0;
    }
    return count;
}

// Closes listener port
void closeListenerPort()
{
    close(listen_sockfd);
}
