// UDP Library
// Computer Engineering IoT/Networks Course
// Jason Losh

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdbool.h>         // bool

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

bool sendData(const char ipv4Address[], int port, const char str[]);
int openListenerPort(const char ipv4Address[], int port);
int receiveData(char str[], int str_length);
void closeListenerPort();
