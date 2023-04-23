// Ethernet Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL w/ ENC28J60
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// ENC28J60 Ethernet controller on SPI0
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS (SW controlled) on PA3
//   WOL on PB3
//   INT on PC6

// Pinning for IoT projects with wireless modules:
// N24L01+ RF transceiver
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS on PE0
//   INT on PB2
// Xbee module
//   DIN (UART1TX) on PC5
//   DOUT (UART1RX) on PC4


//-----------------------------------------------------------------------------
// Hints for using Wireshark to examine packets
//-----------------------------------------------------------------------------

// sudo ethtool --offload eno2 tx off rx off
// in wireshark, preferences->protocol->ipv4->validate the checksum if possible
// in wireshark, preferences->protocol->udp->validate the checksum if possible

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "eth0.h"
#include "gpio.h"
#include "spi0.h"
#include "uart0.h"
#include "wait.h"
#include "spi1.h"


// --------------------MOTION CONTROL TEAM--------------------------

// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4

#define MAX_SPLIT_LEN 5
#define MAX_SPLIT_BRE 10
#define MAX_LEN_LINE  80
#define CHAR_SPACE    0x20
#define CHAR_NEWLINE 0x0A
#define CHAR_NEWLINE1 0x0D

#define STATE_RUN   1
#define STATE_PAUSE 2
#define STATE_STOP  3

float MIN_XDELTA_MOV = 0.1;
float MIN_YDELTA_MOV = 0.1;
#define PROCESSOR_CLOCK_FREQ 40000000
#define MIN_TO_SEC_MULTIPLIER 60
#define ARC_SEGMENTS 10
#define COUNTERCLOCKWISE 1

#define PC_COMMAND_START 1
#define PC_COMMAND_PAUSE 2
#define PC_COMMAND_JOG 6
#define PC_COMMAND_STOP 11
#define PC_COMMAND_RESUME 12
#define PC_COMMAND_ARCTO 13
#define PC_COMMAND_HOME 15


float large_distance;
float small_distance;
float large_delta;
float small_delta;
float delta_x;
float delta_y;
float actual_large;
float actual_small;
float small_delta_ideal;
float x_init;
float y_init;
float z_init;
bool Lx;
uint16_t segments;
uint16_t actual_segment;
float x0;
float x_motor;
float y_motor;


float x_1_arc;
float y_1_arc;
float x_2_arc;
float y_2_arc;
bool direction_counterclockwise;
bool linear_mov;
float c_x;
float c_y;
float d_c1;
float d_c2;
float d_12;
float angle;
float angle_segments;
uint16_t actual_angle_segment;
float cos_angle;
float sin_angle;
float actual_x_arc;
float actual_y_arc;

bool gcode_finished = true;
bool enable_motion;
bool step_done=true;
bool move_xy;
bool move_z;

char split_array[MAX_SPLIT_BRE][MAX_SPLIT_LEN];
uint8_t state;
bool start_read_gline;

typedef struct _COMMAND
{
    char *command;
    float x;
    float y;
    float z;
    float f;
    float s;
    float i;
    float j;
    uint8_t arc_dir;
} COMMAND;





/* GCODE TEST ARRAY */
uint8_t counter;
//char gcode_test_array[9][20] = {"G00 X0 Y0 F70\0",
//                                "G00 X0 Y5\0",
//                                "G02 X5 Y0 I0 J-5\0",
//                                //"G01 Z-1 F50\0",
//                                "G01 X0 Y5 F50\0",
//                                "G01 X5 Y5\0",
//                                "G01 X5 Y0\0",
//                                "G01 X0 Y0\0",
//                                //"G00 Z0 F70\0",
//                                "M30\0"};

//char gcode_test_array[5][25] = {//"G00 X0 Y0 F70\0",
//                                "G03 X-50 Y50 I-50 J0\0",
//                                "G00 X-50 Y0\0",
//                                "G00 X-50 Y50\0",
//                                "G02 X0 Y0 I50 J-50\0",
//                                "M30\0"};

//char gcode_test_array[5][25] = {//"G00 X0 Y0 F70\0",
//                                "G00 X0 Y90\0",
//                                "G02 X90 Y0 I0 J-90\0",
//                                "G03 X0 Y90 I-90 J0\0",
//                                "G00 X0 Y0\0",
//                                "M30\0"};

//char gcode_test_array[2][25] = {//"G00 X0 Y0 F70\0",
//                                "G01 X10 Y0\0",
//                                "M30\0"};


//char gcode_test_array[6][20] = {"G00 X0 Y0 F70\0",
//                                "G01 X0 Y20 F50\0",
//                                "G01 X25 Y20\0",
//                                "G01 X25 Y0\0",
//                                "G01 X0 Y0\0",
//                                "M30\0"};

/* GCODE TEST ARRAY */



// --------------------MOTION CONTROL TEAM--------------------------

// --------------------ETHERNET CONTROL TEAM-------------------------


typedef void (*fn)();

#define maxChar 25
#define fsComPort     1024
#define mcComPort     1060
#define fsDataPort    1080
#define statusCommand 7
#define dirCom        10
#define fsStatusCom   8
#define fssetactivecom 3
#define fsuploadcom    9


#define MAX_BUFFERS 2
uint8_t Command=2;
uint16_t lengthofafile=16;
uint8_t acknowledgement=0;

char *str;
char a[12];
bool filesCommandempty;
bool filesDataEmpty;
bool fileCommandResponse=true;

uint16_t destPort;

uint8_t q;

#define max_route 2
uint8_t request=0;
uint8_t mc;
uint8_t fs;


#define MAX_PACKET_SIZE 1522
#define BUFFER_SIZE 513
uint8_t data[MAX_PACKET_SIZE];
uint8_t Buffer[2][BUFFER_SIZE];

// --------------------ETHERNET CONTROL TEAM-------------------------

// --------------------MOTOR CONTROL TEAM-------------------------

uint32_t tick_count_x = 0;
uint32_t tick_count_y = 0;
uint32_t tick_count_z = 0;
bool axisLimitError = 0; // flag when cnc hits one of the min or max on any axes; 0 = no error, 1 = error
bool Home_Flag = 0;

#define PWMX  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))//PB6 PWMX
#define XDir  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 7*4)))//PD7 XDir
#define Xmin  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))//PC4 Xmin
#define Xmax  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))//PC5 Xmax
// Y axis bitband
#define PWMY  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))//PB7 PWMY
#define YDir  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))//PD6 YDir
#define Ymin  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))//PC6 Ymin
#define Ymax  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))//PC7 Ymax
// Z axis bitband
#define PWMZ  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))//PB5 PWMZ
#define ZDir  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))//PB4 ZDir
#define Zmin  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4)))//PB0 Zmin
#define Zmax  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))//PB1 Zmax
//Touch Probe
#define TP_EN  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))//PE1 P
//#define ZDir  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))//PB4 ZDir


// X axis masks
#define PWMX_MASK 64
#define XDIR_MASK 128
#define Xmin_MASK 16
#define Xmax_MASK 32
// z axis masks
#define PWMZ_MASK 32
#define ZDIR_MASK 16
#define Zmin_MASK 1
#define Zmax_MASK 2

#define RED_RGB_MASK 2
// Y axis masks
#define PWMY_MASK 128
#define YDIR_MASK 64
#define Ymin_MASK 64
#define Ymax_MASK 128
// Touch Probe #define PWMY_MASK 128
#define TP_ENABLE_MASK 2
#define TP_IN_MASK 4

//define
#define POS_DIR 0
#define NEG_DIR 1
#define Posy_Dir 1
#define Negy_Dir 0

// -------------------------FILE  TEAM-----------------------------
uint32_t sector_num;
uint32_t ind_file_data;


uint8_t  writeDatabuff[] ="mmmmmmmmmm";

bool fileset =false;


 // GLOABLE WRITE BUFF
uint32_t writeFileLen;
char *writefilename;

int i = 0, j = 0, k = 0;
uint8_t setActiveFileName[50];
uint8_t lengthActiveFileName[50];
uint32_t lengthActiveFile;
uint32_t blockWrite;
bool createfile = false;

//------------------------------------------------------
uint32_t lbaBeginSec=2048,sectorCount=4194304;   // thats value of my SD card but my function readfirstsector() give a dynamic value for any sd card
uint32_t fatSector, fileClusterNum,fileLen;  // GET FROM  ROOT DIR
uint32_t sectorsPerfat,clusterSector,sectorsPercluster, rootDirectory;
uint32_t nFats, nRootDir, bytesPersector,reservedSector,clusterStartsector,activeFileStartsector,maxBlocktoRead;  // FOR FILE LEN /2+1
uint8_t rootDirdata[512];   // store rootDir data
char intstr[];
uint8_t Data[512];// read global buff


char restofdata[30];  // while we use cmd12
char buff3[30];
uint32_t SD_READ_LEN=512;
/* Definitions for MMC/SDC command */
#define CMD0    (0x40+0)    /* GO_IDLE_STATE */
#define CMD1    (0x40+1)    /* SEND_OP_COND */
#define CMD8    (0x40+8)    /* SEND_IF_COND */
#define CMD9    (0x40+9)    /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */


//
#define SSI1TX PORTD,3
#define SSI1RX PORTD,2
#define SSI1CLK PORTD,0
#define CS PORTD,1

#define GLINE_MAXLIM 30
char gline[GLINE_MAXLIM];
// -------------------------FILE  TEAM-----------------------------


// --------------------MOTION CONTROL TEAM-------------------------

void parse_command_PC(char *);
void clear_command_struct(COMMAND*);
void clear_split_array(void);
void populate_command(COMMAND *);
uint8_t str_length(const char *);
bool str_compare(const char *, const char *);
void check_command(COMMAND *);
void line_init(float, float);
void move_linear(void);
void enable_timer(void);
void disable_timer(void);
void check_command_gcode(COMMAND *);
uint32_t calculate_timer_load(COMMAND *);
void set_timer_load(uint32_t);
void set_read_gline_status(bool);
bool get_read_gline_status(void);
void arc_init(COMMAND *);
void arc_next_point(void);
void update_initPoint(COMMAND *);
void populate_ij_arc(COMMAND *);
// --------------------MOTION CONTROL TEAM-------------------------

// --------------------MOTOR CONTROL TEAM-------------------------

void move(char axis, uint8_t dir, float dist1);
void home(void);
// --------------------MOTOR CONTROL TEAM-------------------------


//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // MOTION CONTROL TEAM
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Enable clocks
    enablePort(PORTF);
    enablePort(PORTB);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    //SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinDigitalInput(PUSH_BUTTON);



    //TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    //TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    //TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    //TIMER1_TAILR_R = 400000;                         // set load value to 40e6 for 1 Hz interrupt rate
    //TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    //NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)


    // PWM CONFIGURATION  - MOTOR CONTROL TEAM
    // Port B config
    GPIO_PORTB_DIR_R |= PWMX_MASK | PWMY_MASK | PWMZ_MASK|ZDIR_MASK;
    GPIO_PORTB_DEN_R |= PWMX_MASK | PWMY_MASK | PWMZ_MASK|ZDIR_MASK;
    GPIO_PORTB_AFSEL_R |= PWMX_MASK | PWMY_MASK | PWMZ_MASK;

    GPIO_PORTB_DIR_R &= ~(Zmin_MASK | Zmax_MASK);
    GPIO_PORTB_PDR_R |= Zmin_MASK | Zmax_MASK;
    GPIO_PORTB_DEN_R |= Zmin_MASK | Zmax_MASK;
    GPIO_PORTB_AFSEL_R |= Zmin_MASK | Zmax_MASK;

    //Port C config
    GPIO_PORTC_DIR_R &= ~(Xmin_MASK | Xmax_MASK | Ymin_MASK | Ymax_MASK);
    GPIO_PORTC_PDR_R |= Xmin_MASK | Xmax_MASK | Ymin_MASK | Ymax_MASK;
    GPIO_PORTC_DEN_R |= Xmin_MASK | Xmax_MASK | Ymin_MASK | Ymax_MASK;
    GPIO_PORTC_AFSEL_R |= Xmin_MASK | Xmax_MASK | Ymin_MASK | Ymax_MASK;

    // Port D config

    GPIO_PORTD_LOCK_R |= GPIO_LOCK_KEY | GPIO_LOCK_UNLOCKED;
    GPIO_PORTD_CR_R |= 0xFF;
    GPIO_PORTD_DIR_R |= XDIR_MASK | YDIR_MASK;
    GPIO_PORTD_DEN_R |= XDIR_MASK | YDIR_MASK;



    // Port E config
    GPIO_PORTE_DIR_R |= TP_ENABLE_MASK;
    GPIO_PORTE_DEN_R |= TP_ENABLE_MASK;
    //  GPIO_PORTE_PDR_R |= TP_ENABLE_MASK;

    //        GPIO_PORTF_DIR_R |= RED_RGB_MASK;
    //        GPIO_PORTF_DEN_R |= RED_RGB_MASK;

    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB6_M | GPIO_PCTL_PB7_M | GPIO_PCTL_PB5_M);
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_M0PWM0 | GPIO_PCTL_PB7_M0PWM1
            | GPIO_PCTL_PB5_M0PWM3;
    // Configure PWM module
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_0_CTL_R = 0; // turn-off PWM0
    PWM0_1_CTL_R = 0; // turn-off PWM0

    PWM0_0_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO; //x
    PWM0_1_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO; //z
    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO; //y

    PWM0_0_LOAD_R = 20000;    //1Khz
    PWM0_1_LOAD_R = 20000;    //1Khz

    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = 0;  // red off (0=always low, 1023=always high)
    PWM0_1_CMPB_R = 0;

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;
    PWM0_1_CTL_R = PWM_0_CTL_ENABLE; // turn-on PWM1 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM3EN;

    //Systick config
    NVIC_ST_CURRENT_R = 0;
    //  NVIC_ST_RELOAD_R = 40000;
    NVIC_ST_RELOAD_R = 4000;
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;

 }


// --------------------ETHERNET CONTROL TEAM-------------------------

void displayConnectionInfo()
{
    uint8_t i;
    char str[10];
    uint8_t mac[6];
    uint8_t ip[4];
    etherGetMacAddress(mac);
    putsUart0("HW: ");
    for (i = 0; i < 6; i++)
    {
        sprintf(str, "%02x", mac[i]);
        putsUart0(str);
        if (i < 6-1)
            putcUart0(':');
    }
    putsUart0("\n\r");
    etherGetIpAddress(ip);
    putsUart0("IP: ");
    for (i = 0; i < 4; i++)
    {
        sprintf(str, "%u", ip[i]);
        putsUart0(str);
        if (i < 4-1)
            putcUart0('.');
    }
    if (etherIsDhcpEnabled())
        putsUart0(" (dhcp)");
    else
        putsUart0(" (static)");
    putsUart0("\n\r");
    etherGetIpSubnetMask(ip);
    putsUart0("SN: ");
    for (i = 0; i < 4; i++)
    {
        sprintf(str, "%u", ip[i]);
        putsUart0(str);
        if (i < 4-1)
            putcUart0('.');
    }
    putsUart0("\n\r");
    etherGetIpGatewayAddress(ip);
    putsUart0("GW: ");
    for (i = 0; i < 4; i++)
    {
        sprintf(str, "%u", ip[i]);
        putsUart0(str);
        if (i < 4-1)
            putcUart0('.');
    }

    putsUart0("\n\r");

    putsUart0("ENC28J60 rev ID is ");
    sprintf(str, "%u", etherGetRevId());
    putsUart0(str);
    putsUart0("\n\r");

    if (etherIsLinkUp())
        putsUart0("Link is up\n\r");
    else
        putsUart0("Link is down\n\r");
}

//void funct(uint8_t* data, uint8_t *word, status* stat_ptr)
//{
//    char string[50];
//    sprintf(string, "x: %d, y: %d, z:%d, Feedrate=%d", stat_ptr -> x, stat_ptr -> y, stat_ptr -> z, stat_ptr -> Feedrate);//, stat_ptr-> y ,stat_ptr-> z);
//    etherSendUdpResponse(data, (uint8_t*)string, 50);
//}

void DataBuffer(char *udpData)
{
    uint16_t i = 0;
    while (udpData[i] != '\0')
    {
        Buffer[request][i]=udpData[i];
        i++;
    }
}

uint8_t strLength(const char* word)
{
    uint8_t strLen = 0;

    while(word[strLen] != '\0')
    {
        strLen += 1;
    }
    return strLen;
}

void ResponsetoPC(uint8_t data[], uint8_t *Bufferdata, char *str)
{
    if (*Bufferdata == dirCom || *Bufferdata == fsStatusCom)
    {
        filesCommandempty = true;
        etherSendUdpResponse(data, (uint8_t*) str, strLength(str));
        request--;
    }
    else
    {
        filesCommandempty = true;
        etherSendUdpResponse(data, (uint8_t*) "received", 9);
        request--;
    }
}


// --------------------ETHERNET CONTROL TEAM-------------------------


// --------------------MOTION CONTROL TEAM-------------------------

void populate_command(COMMAND *command_struct_ptr)
{
    char word[MAX_LEN_LINE];
    char c;
    uint8_t ind_bre = 1;
    uint8_t ind_len = 0;
    float num_val;

    clear_command_struct(command_struct_ptr);

    command_struct_ptr -> command = split_array[0];

    for(ind_bre = 1; ind_bre < MAX_SPLIT_BRE; ind_bre++)
    {
        for(ind_len = 0; ind_len < MAX_SPLIT_LEN; ind_len++)
        {
            c = split_array[ind_bre][ind_len];

            num_val = (float) atof(&split_array[ind_bre][ind_len + 1]);

            if( c == 'X' || c == 'x')
            {
                command_struct_ptr -> x = num_val;
            }
            else if(c == 'Y' || c == 'y')
            {
                command_struct_ptr -> y = num_val;
            }
            else if(c == 'Z' || c == 'z')
            {
                command_struct_ptr -> z = num_val;
            }
            else if(c == 'F' || c == 'f')
            {
                command_struct_ptr -> f = num_val;
            }
            else if(c == 'I' || c == 'i')
            {
                command_struct_ptr -> i = num_val;
            }
            else if(c == 'J' || c == 'j')
            {
                command_struct_ptr -> j = num_val;
            }
            else if(c == 'D' || c == 'd')
            {
                command_struct_ptr -> arc_dir = (uint8_t)num_val;
            }
        }
    }

    sprintf(word, "\r%s X = %f Y = %f Z = %f F = %f\n", command_struct_ptr -> command, command_struct_ptr -> x, command_struct_ptr -> y, command_struct_ptr -> z, command_struct_ptr -> f);
    putsUart0(word);
}


void parse_command_PC(char *line)
{
    uint8_t ind_line = 0;
    uint8_t ind_split_bre = 0;
    uint8_t ind_split_len = 0;

    clear_split_array();

    while(line[ind_line] != NULL)
    {
        if(line[ind_line] != CHAR_SPACE)
        {
            split_array[ind_split_bre][ind_split_len] = line[ind_line];

            ind_split_len += 1;
        }
        else
        {
            split_array[ind_split_bre][ind_split_len] = NULL;

            ind_split_bre += 1;
            ind_split_len = 0;
        }

        ind_line += 1;
    }
}


void clear_split_array(void)
{
    uint8_t ind_bre = 0;
    uint8_t ind_len = 0;

    for(ind_bre = 0; ind_bre < MAX_SPLIT_BRE; ind_bre++)
    {
        for(ind_len = 0; ind_len < MAX_SPLIT_LEN; ind_len++)
        {
            split_array[ind_bre][ind_len] = NULL;
        }
    }
}


void clear_command_struct(COMMAND* command_struct_ptr)
{
    command_struct_ptr -> command = NULL;
}


void check_command(COMMAND *command_struct_ptr)
{
    if(*(command_struct_ptr -> command) == PC_COMMAND_JOG)
    {
        state = STATE_RUN;

        if(command_struct_ptr ->z != z_init)
        {
            move_xy=0;
            move_z=1;
            if(command_struct_ptr ->z > z_init)
            {
                move('z',POS_DIR,command_struct_ptr ->z - z_init);
            }
            else
            {
                move('z',NEG_DIR,z_init - command_struct_ptr ->z);
            }
        }
        else
        {
            move_xy=1;
            move_z=0;
            line_init(command_struct_ptr->x, command_struct_ptr->y);
            linear_mov = true;
        }
        enable_timer();

    }
    else if(*(command_struct_ptr -> command) == PC_COMMAND_PAUSE)
    {
        state = STATE_PAUSE;

        disable_timer();
    }
    else if(*(command_struct_ptr -> command) == PC_COMMAND_RESUME)
    {
        if(state != STATE_STOP)
        {
            state = STATE_RUN;
            enable_timer();
        }
    }
    else if(*(command_struct_ptr -> command) == PC_COMMAND_STOP)
    {
        state = STATE_STOP;

        disable_timer();
    }
    else if (*(command_struct_ptr -> command) == PC_COMMAND_START)
    {
        state = STATE_RUN;

        gcode_finished = false;

        set_read_gline_status(true);
        //first 512 bytes
        sector_num = activeFileStartsector; //2728
        singleBlockRead(sector_num);

    }
    else if (*(command_struct_ptr -> command) == PC_COMMAND_ARCTO)
    {
        state = STATE_RUN;
        move_xy=1;
        move_z=0;
        direction_counterclockwise = (bool)(command_struct_ptr -> arc_dir);
        populate_ij_arc(command_struct_ptr);
        arc_init(command_struct_ptr);
        arc_next_point();
        line_init(actual_x_arc,actual_y_arc);
        linear_mov = false;
        enable_timer();
    }
    else if (*(command_struct_ptr -> command) == PC_COMMAND_HOME)
    {
        state = STATE_STOP;
        disable_timer();
        home();
    }

}


void check_command_gcode(COMMAND *command_struct_ptr)
{
    if(str_compare(command_struct_ptr -> command, "G00") || str_compare(command_struct_ptr -> command, "G01"))
    {
        state = STATE_RUN;

        if(command_struct_ptr ->z != z_init)
        {
            move_xy=0;
            move_z=1;
            if(command_struct_ptr ->z > z_init)
            {
                move('z',POS_DIR,command_struct_ptr ->z - z_init);
            }
            else
            {
                move('z',NEG_DIR,z_init - command_struct_ptr ->z);
            }
        }
        else
        {
            move_xy=1;
            move_z=0;
            line_init(command_struct_ptr->x, command_struct_ptr->y);
            linear_mov = true;
        }
        enable_timer();
    }
    else if(str_compare(command_struct_ptr -> command, "G02") )
    {
        state = STATE_RUN;
        move_xy=1;
        move_z=0;
        direction_counterclockwise = false;
        arc_init(command_struct_ptr);
        arc_next_point();
        line_init(actual_x_arc,actual_y_arc);
        linear_mov = false;
        enable_timer();
    }
    else if(str_compare(command_struct_ptr -> command, "G03"))
    {
        state = STATE_RUN;
        move_xy=1;
        move_z=0;
        direction_counterclockwise = true;
        arc_init(command_struct_ptr);
        arc_next_point();
        line_init(actual_x_arc,actual_y_arc);
        linear_mov = false;
        enable_timer();
    }
    else if(str_compare(command_struct_ptr -> command, "M3"))
    {
        //spindle on clockwise; call functions defined by motor control team

        set_read_gline_status(true);
    }
    else if(str_compare(command_struct_ptr -> command, "M4"))
    {
        //spindle on counter clockwise; call functions defined by motor control team

        set_read_gline_status(true);
    }
    else if(str_compare(command_struct_ptr -> command, "M5"))
    {
        //spindle stop; call functions defined by motor control team

        set_read_gline_status(true);
    }
    else if(command_struct_ptr -> command[0] == 'M' && command_struct_ptr -> command[1] == '3' && command_struct_ptr -> command[2] == '0')
    {
        state = STATE_STOP;

        disable_timer();

        gcode_finished = true;
    }
}


void set_read_gline_status(bool stat)
{
    start_read_gline = stat;
}


bool get_read_gline_status(void)
{
    return start_read_gline;
}


uint32_t calculate_timer_load(COMMAND *command_struct_ptr)
{
    return ((MIN_XDELTA_MOV * PROCESSOR_CLOCK_FREQ * MIN_TO_SEC_MULTIPLIER) / command_struct_ptr -> f);
}


void set_timer_load(uint32_t load)
{
    TIMER1_TAILR_R = load;
}


void enable_timer(void)
{
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;
    enable_motion = true;
}


void disable_timer(void)
{
    //TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    enable_motion = false;
}

void update_initPoint(COMMAND *command_struct_ptr)
{
    x_init = command_struct_ptr->x;
    y_init = command_struct_ptr->y;
    z_init = command_struct_ptr ->z;
}
void arc_next_point(void)
{
    cos_angle=cos(angle_segments*actual_angle_segment);
    sin_angle=sin(angle_segments*actual_angle_segment);
    if(direction_counterclockwise == true)
    {
        actual_x_arc=c_x+((x_1_arc-c_x)*cos_angle)-((y_1_arc-c_y)*sin_angle);
        actual_y_arc=c_y+((x_1_arc-c_x)*sin_angle)+((y_1_arc-c_y)*cos_angle);
    }
    else
    {
        actual_x_arc=c_x+((x_1_arc-c_x)*cos_angle)+((y_1_arc-c_y)*sin_angle);
        actual_y_arc=c_y-((x_1_arc-c_x)*sin_angle)+((y_1_arc-c_y)*cos_angle);
    }
    actual_angle_segment=actual_angle_segment+1;
}

void populate_ij_arc(COMMAND *command_struct_ptr)
{
    if(direction_counterclockwise == true)
    {
        command_struct_ptr->i = ((command_struct_ptr-> x - x_init)/2)-((command_struct_ptr->y - y_init)/2) ;
        command_struct_ptr->j = ((command_struct_ptr-> y - y_init)/2)+((command_struct_ptr->x - x_init)/2) ;
    }
    else
    {
        command_struct_ptr->i = ((command_struct_ptr-> x - x_init)/2)+((command_struct_ptr->y - y_init)/2) ;
        command_struct_ptr->j = ((command_struct_ptr-> y - y_init)/2)-((command_struct_ptr->x - x_init)/2) ;
    }
}
void arc_init(COMMAND *command_struct_ptr)
{
    actual_angle_segment=1;
    x_1_arc=x_init;
    y_1_arc=y_init;
    //Final point
    x_2_arc=command_struct_ptr->x;
    y_2_arc=command_struct_ptr->y;
    // Center (x,y)
    c_x=x_1_arc+command_struct_ptr->i;
    c_y=y_1_arc+command_struct_ptr->j;
    // Distances of vectors
    d_c1=sqrt(pow((c_x-x_1_arc),2)+pow((c_y-y_1_arc),2));
    d_c2=sqrt(pow((c_x-x_2_arc),2)+pow((c_y-y_2_arc),2));
    d_12=sqrt(pow((x_1_arc-x_2_arc),2)+pow((y_1_arc-y_2_arc),2));
    // angle of radius
    angle=acos((pow(d_c1,2)+pow(d_c2,2)-pow(d_12,2))/(2*d_c1*d_c2));
    // # of segments
    angle_segments=angle/ARC_SEGMENTS;
}

void line_init(float x_destination, float y_destination)
{
    /* find biggest difference */
    float diff_x;
    float diff_y;
    float diff_x_abs;
    float diff_y_abs;

    actual_segment=0;
    // CONSIDER CONDITION WHEN DIFF_X AND DIFF_Y IS 0. NO NEED TO MOVE

    diff_x = x_destination - x_motor;
    diff_y = y_destination - y_motor;

    diff_x_abs=diff_x;
    if(diff_x_abs < 0)
        diff_x_abs = diff_x_abs * (-1);
    diff_y_abs=diff_y;
    if(diff_y_abs < 0)
        diff_y_abs = diff_y_abs * (-1);


    if(diff_x_abs > diff_y_abs)
    {
        large_delta = delta_x;
        small_delta = delta_y;

        large_distance = diff_x;
        small_distance = diff_y;

        actual_large = x_motor;
        actual_small = y_motor;

        Lx = true;
        x0 = y_motor;
    }
    else
    {
        large_delta = delta_y;
        small_delta = delta_x;

        large_distance = diff_y;
        small_distance = diff_x;

        actual_large = y_motor;
        actual_small = x_motor;

        Lx = false;
        x0 = x_motor;
    }

    small_delta_ideal = (small_distance / large_distance) * large_delta;

    if(small_delta_ideal < 0)
        small_delta_ideal = small_delta_ideal * (-1);

    segments = abs(round(large_distance / large_delta));
}


uint8_t str_length(const char* word)
{
    uint8_t strLen = 0;

    while(word[strLen] != '\0')
    {
        strLen += 1;
    }
    return strLen;
}


bool str_compare(const char* str1, const char* str2)
{
    uint8_t i        = 0;
    uint8_t str1_len = str_length(str1);
    uint8_t str2_len = str_length(str2);

    if(str1_len != str2_len)
    {
        return 0;
    }

    while(i < str2_len)
    {
        if(str2[i] != str1[i])
        {
            return 0;
        }
        i += 1;
    }
    return 1;
}


void move_linear(void)
{
    char word[100];
    //step_done=false;
    actual_segment += 1;
    if(large_distance >= 0 && small_distance >= 0)
    {
        actual_large = actual_large + large_delta;
        if(actual_small < (x0 + (actual_segment * small_delta_ideal)))
        {
            actual_small = actual_small + small_delta;
        }
    }
    else if(large_distance < 0 && small_distance < 0)
    {
        actual_large = actual_large - large_delta;
        if(actual_small > (x0 - (actual_segment * small_delta_ideal)))
        {
            actual_small = actual_small - small_delta;
        }
    }
    else if(large_distance <= 0 && small_distance >= 0)
    {
        actual_large = actual_large - large_delta;
        if(actual_small < (x0 + (actual_segment * small_delta_ideal)))
        {
            actual_small = actual_small + small_delta;
        }

    }
    else if(large_distance > 0 && small_distance < 0)
    {
        actual_large = actual_large + large_delta;
        if(actual_small > (x0 - (actual_segment * small_delta_ideal)))
        {
            actual_small = actual_small - small_delta;
        }
    }

    if(Lx == true)
    {
        if(x_motor < actual_large)
        {
            move('x',0,MIN_XDELTA_MOV);
        }
        else if(x_motor > actual_large)
        {
            move('x',1,MIN_XDELTA_MOV);
        }
        if(y_motor < actual_small)
        {
            move('y',1,MIN_XDELTA_MOV);
        }
        else if(y_motor > actual_small)
        {
            move('y',0,MIN_XDELTA_MOV);
        }
        x_motor = actual_large;
        y_motor = actual_small;
    }
    else
    {
        if(x_motor < actual_small)
        {
            move('x',0,MIN_XDELTA_MOV);
        }
        else if(x_motor > actual_small)
        {
            move('x',1,MIN_XDELTA_MOV);
        }
        if(y_motor < actual_large)
        {
            move('y',1,MIN_XDELTA_MOV);
        }
        else if(y_motor > actual_large)
        {
            move('y',0,MIN_XDELTA_MOV);
        }
        x_motor = actual_small;
        y_motor = actual_large;
    }
    sprintf(word, "\r\nx motor = %f, y motor = %f large delta = %f\n", x_motor, y_motor, large_delta);
    putsUart0(word);
}

// --------------------MOTION CONTROL TEAM-------------------------


// --------------------MOTOR CONTROL TEAM-------------------------

void home(void)
{
    Home_Flag = 1;
    NVIC_ST_CTRL_R = 0;
    while (Xmin != 1)
    {
        XDir = NEG_DIR;
        PWM0_0_CMPA_R = 10000;
    }
    XDir = POS_DIR;
    PWM0_0_CMPA_R = 0;

    while (Ymin != 1)
    {
        YDir = Negy_Dir;
        PWM0_0_CMPB_R = 10000;
    }
    YDir = Posy_Dir;
    PWM0_0_CMPB_R = 0;
    while (Zmax != 1)
    {
        ZDir = NEG_DIR;
        PWM0_1_CMPB_R = 10000;
    }
        ZDir = POS_DIR;
        PWM0_1_CMPB_R = 0;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;

    waitMicrosecond(500000);
    move('x', POS_DIR, 10);
    move('y', Posy_Dir, 10);
    move('z', POS_DIR, 50);
    waitMicrosecond(2000000);
    PWM0_1_CMPB_R = 0;
    tick_count_z = 0;
    PWM0_0_CMPB_R = 0;
    tick_count_y = 0;
    PWM0_0_CMPA_R = 0;
        tick_count_x = 0;
//    move('x', POS_DIR, 145);
// //   move('y', Posy_Dir, 10);
//    move('z', POS_DIR, 180);
//    waitMicrosecond(2000000);
    Home_Flag = 0;
 //  while(1);

}
void systickIsr()
{
    if ((tick_count_x > 0) || (tick_count_y > 0) || (tick_count_z > 0))
    {
        if (tick_count_x > 0)
           {
               tick_count_x--;
               if (tick_count_x == 0)
               {
                   PWM0_0_CMPA_R = 0;
               }
           }
        if (tick_count_y > 0)
           {
               tick_count_y--;
               if (tick_count_y == 0)
               {
                   PWM0_0_CMPB_R = 0;
               }
           }
        if (tick_count_z > 0)
           {
               tick_count_z--;
               if (tick_count_z == 0)
               {
                   PWM0_1_CMPB_R = 0;
               }
           }
    }

    if ((tick_count_x == 0) && (tick_count_y == 0) && (tick_count_z == 0))
    {
        step_done = 1;
    }
    if (Home_Flag == 0)
    {
//    if (Xmin || Xmax || Ymin || Ymax || Zmin || Zmax)
//    {
//        PWM0_0_CMPA_R = 0;
//        PWM0_0_CMPB_R = 0;
//        PWM0_1_CMPB_R = 0;
//        //PWM0_0_CTL_R = 0;
//        PWM0_1_CTL_R = 0;
//        tick_count_x = 0;
//        tick_count_y = 0;
//        tick_count_z = 0;
//        if (step_done == 0)
//        {
//            axisLimitError = 1;
//        }
//    }
    }
}

void move(char axis, uint8_t dir, float dist1)
{
    uint32_t dist = (uint32_t) (dist1*10);

//void move(char axis, uint8_t dir, uint32_t dist)
//{
    step_done = 0;
    //dist=(uint8_t)dist;
    if (axis =='x')
    {
        while (tick_count_x !=0);
        XDir = dir;
        tick_count_x = (dist/7.3) * (1000)/1.6;
        PWM0_0_CMPA_R = 10000;
        if (tick_count_y == 0)
        {
            PWM0_0_CMPB_R = 0;
        }
    }
    else if (axis =='y')
    {
        while (tick_count_y !=0);
        YDir = dir;
        tick_count_y = (dist/7.3) * (1000)/1.6;
        if (tick_count_x == 0)
        {
            PWM0_0_CMPA_R = 0;
        }
        PWM0_0_CMPB_R = 10000;
    }
    else if (axis =='z')
    {
        while (tick_count_z !=0);
        //PWM0_1_CTL_R = PWM_0_CTL_ENABLE;
       // PWM0_1_CMPA_R = 0;
        PWM0_1_CMPB_R = 0;
        ZDir = dir;
        tick_count_z = (dist/7.3) * (1000)/1.6;
        //PWM0_1_CMPA_R = 0;
        PWM0_1_CMPB_R = 10000;
    }

}
// --------------------MOTOR CONTROL TEAM-------------------------

// -----------------------------FILE TEAM--------------------------
void intToString(uint32_t number)
{
    int j;
    int i =0;
    char temp;
    j=0;

    while(number)
    {
        intstr[j] = number%10 + '0';
        j++;
        number/=10;
    }
        intstr[j] ='\0';
        j--;

    while(i<j)
    {
        temp = intstr[j];
        intstr[j] = intstr[i];
        intstr[i] = temp;
        i++;
        j--;
    }
    putsUart0(intstr);
    putsUart0("\r\n");
}
//void setTxHigh()
//{
//    GPIO_PORTD_AFSEL_R &= ~SSI1_TX;
//    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD3_M;
//    setPinValue(SSI1TX, 1);
//}
void  dummySend()
{

    uint8_t i;
   for(i=0;i<10;i++)
   {
       writeSpi1Data(0xFF);
     //  setPinValue(SSI1TX,1);
   }

    //setPinAuxFunction(SSI1TX, GPIO_PCTL_PD3_SSI1TX);
}

void readSdcarData()
{
    int i;
    uint8_t receive;
    for(i=0;i<512;i++)
    {
         writeSpi1Data(0xFF);
         receive = readSpi1Data();

         Data[i] = receive;

    }


//
//    for(i = 0; i < 45; i++)
//    {
//        putcUart0(Data[i]);
//    }

}


void increase_sector_num(void)
{
    sector_num += 1;
}


char* get_line_from_file_buffer(void)
{
    static uint8_t g_count = 0;
    static uint8_t left_count = 0;

    while(true)
    {
        if(Data[ind_file_data] != CHAR_NEWLINE)
        {
            gline[g_count] = Data[ind_file_data];

            g_count += 1;

            left_count += 1;

            ind_file_data += 1;
        }
        else
        {
            gline[g_count] = '\0';

            g_count = 0;

            left_count = 0;

            ind_file_data += 1;

            break;
        }

        if(ind_file_data >= 512 && left_count != 0)
        {
            sector_num += 1;

            singleBlockRead(sector_num);

            ind_file_data = 0;
        }
    }

    return gline;
}


bool readSdcard(uint8_t cmd)
{
    uint32_t receive;
    bool ok= false;
    uint8_t i,p=0;
    const char version_2[] ={0x00,0x00,0x01,0xAA};
    const char cmd58_check[] ={0xC0,0xFF,0x80,0x00};
    if(cmd==CMD0){
        for(i=0;i<255;i++)
        {
            writeSpi1Data(0xFF);
            receive = readSpi1Data();
            if(receive==0x00000001)
            {   ok =true;
            return ok;
            }
        }
        return ok;
    }
    else if (cmd==CMD8)
    {
        for(i=0;i<255;i++)
        {
            writeSpi1Data(0xFF);
            receive = readSpi1Data();
           // buff[i]=receive;
            if(receive==0x01)
            {
                for(p=0;p<4;p++)
                {
                    writeSpi1Data(0xFF);
                    receive = readSpi1Data();
                    //buff[i+p+1]=receive;
                    if( version_2[p]==receive)
                    {
                        ok=true;
                    }
                    else
                    {   putsUart0("\r>SD card is not version 2\r\n ");
                    ok= false;
                    return ok;
                    }


                }
                if(ok)
                {  putsUart0("\r>SD card has Version 2.. \r\n");
                return ok;
                }


            }
        }
        return ok;
    }
    else if(cmd==CMD55)
    {
        for(i=0;i<255;i++)
        {
            writeSpi1Data(0xFF);
            receive = readSpi1Data();
           // buff1[i]=receive;
            if(receive==0x00000001 ||receive==0x00000000)
            {   ok =true;

            return ok;
            }
        }
        return ok;
    }
    else if(cmd==CMD41)
    {
        for(i=0;i<255;i++)
        {
            writeSpi1Data(0xFF);
            receive = readSpi1Data();
           // buff2[i]=receive;
            if(receive==0x00000000)
            {  ok =true;
            return ok;
            }
        }
        return ok;
    }
    else if(cmd ==CMD58)
    {
        for(i=0;i<255;i++)
        {
            writeSpi1Data(0xFF);
            receive = readSpi1Data();
           // buff3[i]=receive;
            if(receive==0x00000000)
            {
                for(p=0;p<4;p++)
                {
                    writeSpi1Data(0xFF);
                    receive = readSpi1Data();
                   // buff3[i+p+1]=receive;
                    if( cmd58_check[p]==receive)
                    {
                        ok= true;
                    }
                    else
                    {
                        ok= false;
                        putsUart0("\r>Error in Cmd 58 .. 2\r\n ");
                        return ok;
                    }
                }

            break;
            }
        }
        return ok;

    }
    else if(cmd==CMD16)       //  give that successfully read len set or not
    {
        for(i=0;i<255;i++)
        {
            writeSpi1Data(0xFF);
            receive = readSpi1Data();
            //buff3[i]=receive;
            if(receive==0x00)
            {   ok =true;
               return ok;
            }
        }
        return ok;

    }
    else if(cmd==CMD17)
    {
        for(i=0;i<255;i++)
        {
            writeSpi1Data(0xFF);
            receive = readSpi1Data();
           // buff3[i]=receive;
            if(receive==0x00000000)
            {
                for(p=0;p<50;p++)
                {
                    writeSpi1Data(0xFF);
                    receive = readSpi1Data();
                   // buff[p]=receive;
                    if(receive==0xFE)        // after that we can read a data
                    {  //readSdcarData();
                        ok= true;

                       return ok;

                    }
                }

                break;
            }
        }
        return ok;
    }
    return ok;
}

void parseFileCommand(uint8_t str[fs])     // file system function
{

    for (i = 2; str[i] != '\0'; i++)
    {
        if (str[i] != 32)
        {
            setActiveFileName[j] = str[i];
            j++;
        }
        else
        {
            j = 0;
            i++;
            break;

        }
    }
    for (; str[i] != '\0'; i++)
    {
        lengthActiveFileName[j] = str[i];
        j++;
    }
    lengthActiveFile = atoi(lengthActiveFileName);
//    lengthActiveFile = (uint32_t) lengthActiveFileName[k]
//            + (((uint32_t) lengthActiveFileName[k + 1]) << 8)
//            + (((uint32_t) lengthActiveFileName[k + 2]) << 16)
//            + (((uint32_t) lengthActiveFileName[k + 3]) << 24);
    lengthActiveFile = ((lengthActiveFile / 512)+1);
}


bool sendCmd(uint8_t cmd)
{    bool status= false;


    setPinValue(CS,1);
    setPinValue(CS,0);
    waitMicrosecond(300000);

    switch(cmd)
    {
    case CMD0:
        writeSpi1Data(0x40);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0x95);
        status=readSdcard(CMD0);
        break;
    case CMD8:
        writeSpi1Data(0x48);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0x01);
        writeSpi1Data(0xAA);
        writeSpi1Data(0x87);
        status=readSdcard(CMD8);
        break;
    case CMD55:
        writeSpi1Data(CMD55);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0xFF);
        status=readSdcard(CMD55);
        break;
    case CMD41:
        writeSpi1Data(CMD41);
        writeSpi1Data(0x40);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0x00);
        writeSpi1Data(0xFF);
        status=readSdcard(CMD41);
        break;
    case CMD58:
            writeSpi1Data(CMD58);
            writeSpi1Data(0x40);
            writeSpi1Data(0x00);
            writeSpi1Data(0x00);
            writeSpi1Data(0x00);
            writeSpi1Data(0xFF);
            status=readSdcard(CMD58);
            break;




    }
    return status;
}






//bool readSdcard1()
//{   const char version_2[] ={0x00,0x00,0x01,0xAA};
//    bool ok= false;
//
//    uint32_t receive;
//
//    for(i=0;i<255;i++)
//    {
//        writeSpi1Data(0xFF);
//        receive = readSpi1Data();
//        buff[i]=receive;
//        //receive = receive;
//        if(receive==0x01)
//        {
//            for(p=0;p<4;p++)
//            {
//                writeSpi1Data(0xFF);
//                receive = readSpi1Data();
//                buff[i+p+1]=receive;
//                if( version_2[p]==receive)
//                {
//                    ok=true;
//                }
//                else
//                {   putsUart0(" SD card is not version 2\r\n ");
//                    ok= false;
//                    break;
//                }
//
//
//            }
//            if(ok)
//            {  putsUart0("version 2 of SD card\r\n");
//                break;
//            }
//
//
//        }
//    }
// return ok;
//}
void setBlocklen(uint32_t len)
{
    uint8_t i;
    uint8_t Arg;
    setPinValue(CS,1);
   setPinValue(CS,0);
    waitMicrosecond(300000);
    writeSpi1Data(CMD16);
    for(i=0;i<4;i++)
    {
     Arg= len>>(24-(8*i));
     writeSpi1Data(Arg);
    }
    writeSpi1Data(0xFF);
    if(readSdcard(CMD16))
    {
        SD_READ_LEN=len;
        putsUart0("\r>Successfully Block Bytes Length  set to -");
        intToString(len);
    }
    else
    {
        putsUart0("\r>!error Block Bytes Length   not set ");
    }
}

void singleBlockRead(uint32_t sector)    // send CMD17
{
    uint8_t i;
    uint8_t Arg;
    setPinValue(CS,1);
     setPinValue(CS,0);
      waitMicrosecond(300000);
    writeSpi1Data(CMD17);
    for(i=0;i<4;i++)
    {
        Arg= sector>>(24-(8*i));
        writeSpi1Data(Arg);
    }
    writeSpi1Data(0xFF);
    if(readSdcard(CMD17))
    {

        putsUart0("\r>Successfully start reading from -");
        //putsUart0(startAdd);
        readSdcarData();


    }
    else
    {
        putsUart0("\r>!error in start reading ");
    }

}
void readFirstsector()    // send CMD17
{
    uint8_t readSectorData[512];
    uint32_t startAdd= 0x00000000;
    uint8_t  receive;
    uint8_t i;
    uint8_t Arg;
    setPinValue(CS,1);
    setPinValue(CS,0);
    waitMicrosecond(300000);
    writeSpi1Data(CMD17);
    for(i=0;i<4;i++)
    {
        Arg= startAdd>>(24-(8*i));
        writeSpi1Data(Arg);
    }
    writeSpi1Data(0xFF);
    if(readSdcard(CMD17))
    {


        putsUart0("\r>Successfully starting reading sector one/MSB...\r\n");
        //putsUart0(startAdd);
        int i;
        for(i=0;i<512;i++)
        {
            writeSpi1Data(0xFF);
            receive  = readSpi1Data();
            readSectorData[i]=receive;

        }
        writeSpi1Data(0xFF);
        writeSpi1Data(0xFF);
        writeSpi1Data(0xFF);
        lbaBeginSec=(uint32_t)readSectorData[454]+(((uint32_t)readSectorData[455])<<8)+(((uint32_t)readSectorData[456])<<16)+(((uint32_t)readSectorData[457])<<24);
        sectorCount=(uint32_t)readSectorData[458]+(((uint32_t)readSectorData[459])<<8)+(((uint32_t)readSectorData[460])<<16)+(((uint32_t)readSectorData[461])<<24);


    }
    else
    {
        putsUart0("\r>!error in start reading\r\n ");
    }

}

void readLbsector(uint32_t sector)
{
    uint8_t readSectorData[512];
    uint8_t  receive;
        uint8_t i;
        uint8_t Arg;
        setPinValue(CS,1);
        setPinValue(CS,0);
        waitMicrosecond(300000);
        writeSpi1Data(CMD17);
        for(i=0;i<4;i++)
        {
            Arg= sector>>(24-(8*i));
            writeSpi1Data(Arg);
        }
        writeSpi1Data(0xFF);
        if(readSdcard(CMD17))
        {


            putsUart0("\r>Successfully starting reading sector of LBS...\r\n");
            //putsUart0(startAdd);
            int i;
            for(i=0;i<512;i++) // we required first fifty bytes  of that sector
            {
                writeSpi1Data(0xFF);
                receive  = readSpi1Data();
                readSectorData[i]=receive;

            }
            writeSpi1Data(0xFF);
            writeSpi1Data(0xFF);
            writeSpi1Data(0xFF);
            fatSector = sector + (uint32_t)readSectorData[14] + (((uint32_t)readSectorData[15])<<8); // gives physical sector number
            sectorsPerfat=(uint32_t)readSectorData[22]+(((uint32_t)readSectorData[23])<<8);
            nFats = (uint32_t)readSectorData[16];
            sectorsPercluster =  readSectorData[13];//BPB_SecPerClus;
            nRootDir = (uint32_t)readSectorData[17]+(((uint32_t)readSectorData[18])<<8);//BPB_RootClus;
            rootDirectory =fatSector +(sectorsPerfat*(nFats));  // give physical sector number
            bytesPersector=(uint32_t)readSectorData[11]+(((uint32_t)readSectorData[12])<<8);   //512
            reservedSector=(uint32_t)readSectorData[14];    //8
            clusterStartsector= sector+( ((reservedSector* bytesPersector) +((sectorsPerfat*bytesPersector)*nFats)+( nRootDir*32))/bytesPersector) ;// 32 bytes for each  entry

        }
        else
        {
            putsUart0("\r>!error in reading sector of LBS...\r\n");
        }

}


void readRootDirectory(uint32_t sector)
{
    uint8_t  receive;
           uint8_t i;
           uint8_t Arg;
           setPinValue(CS,1);
           setPinValue(CS,0);
           waitMicrosecond(300000);
           writeSpi1Data(CMD17);
           for(i=0;i<4;i++)
           {
               Arg= sector>>(24-(8*i));
               writeSpi1Data(Arg);
           }
           writeSpi1Data(0xFF);
           if(readSdcard(CMD17))
           {


               //putsUart0("\r>Successfully starting reading sector Root...\r\n");
               //putsUart0(startAdd);
               int i;
               for(i=0;i<512;i++) // we required first fifty bytes  of that sector
               {
                   writeSpi1Data(0xFF);
                   receive  = readSpi1Data();
                   rootDirdata[i]=receive;

               }
               writeSpi1Data(0xFF);
               writeSpi1Data(0xFF);
               writeSpi1Data(0xFF);
           }
}



bool activatefile(char *str) /// str will come from file team
{

    int i=0,j=0,p,a=0,index=0;
    uint32_t read32SecofRoot= rootDirectory; /// this  is global variable that always gives starting sector of root Dir
    for(i=0;str[i]!='\0';i++)
    {
       while(!(((str[i]>='a') && (str[i]<='z')) || ((str[i]>='A') && (str[i]<='Z')) ||((str[i]>=48) &&(str[i]<=57))))
       {
           for(j=i;str[j]!= '\0';j++)
               {
                   str[j] = str[j+1];
               }
           //str[j] = '\0';
       }
    }
    putsUart0("\r");
    putsUart0(str);
    putsUart0("\r\n");
    for(i=0;str[i]!='\0';i++)
    {
        if((str[i]>='a') && (str[i]<='z'))
        {
            str[i]=str[i]-32;
        }
    }
    putsUart0("\r");
    putsUart0(str);
    putsUart0("\r\n");

    // now we call the root Dir to compare the string son we cane find out file is there or not
  while(a<32)
  {
    readRootDirectory(read32SecofRoot);  // after running that global rootDirdata[512] array will gives data
    read32SecofRoot=read32SecofRoot+1;
    for(i=0; i<512;i=i+32)
    {   index =0;
        for(p=i;p<i+8;p++)
        {
            if(str[index]==rootDirdata[p] || rootDirdata[p]==32)
            {
                if(rootDirdata[p]!=32)
                {
                    index++;
                }

            }
            else
            {
                break;
            }
        }
        if(p==i+8)
        {
           for(p=i+8;p<i+11;p++)
           {
               if(str[index]==rootDirdata[p])
               {
                   index++;
               }
               else
               {
                   break;
               }
           }
        }
        if(p==i+11)
        {
            fileClusterNum=(uint32_t)rootDirdata[p+15]+(((uint32_t)rootDirdata[p+16])<<8);
            fileLen=(uint32_t)rootDirdata[p+17] + (((uint32_t)rootDirdata[p+18])<<8)+(((uint32_t)rootDirdata[p+19])<<16)+(((uint32_t)rootDirdata[p+19])<<24);
            activeFileStartsector= clusterStartsector +((fileClusterNum-2)* sectorsPercluster );
            maxBlocktoRead=(fileLen/512);
            if(fileLen%512 !=0)
            {
                maxBlocktoRead= maxBlocktoRead+1;
            }
            return 1;

        }
    }a++;
  }
  putsUart0("\r>!Error file not found...\r\n");

    return 0;


}



void writeBlock(uint32_t sector,uint8_t str[])
{
    int i;
    uint8_t Arg;
    uint8_t receive;
    setPinValue(CS,1);
    setPinValue(CS,0);
    waitMicrosecond(300000);
    writeSpi1Data(CMD24);
    for(i=0;i<4;i++)
    {
        Arg= sector>>(24-(8*i));
        writeSpi1Data(Arg);
    }
    writeSpi1Data(0xFF);  //crc
    if(readSdcard(CMD24))
        {

            putsUart0("\r>Successfully start writing file data-..\r\n");
            //putsUart0(startAdd);
            for(i=0;i<512;i++)
            {
               writeSpi1Data(str[i]);   // global write buff
            }
            writeSpi1Data(0xFF);  // CHECK CRC
            writeSpi1Data(0xFF);

            // CHECK SUCCESSFULLY WRITE ACCEPTED OR NOT
            for(i=0;i<20;i++)
            {
                writeSpi1Data(0xFF);
                receive = readSpi1Data();
                buff3[i]=receive;
                if(receive==0x03)
                {

                }

            }


        }
        else
        {
            putsUart0("\r>!error in start writing ");
        }



}
uint16_t  allocateFatSpace(uint32_t sector)   // block cluster for  new file.
{   int i,p;

    uint32_t clusterforNewFile;
    uint32_t temp =sector;
   for(i=0;i<128;i++)   //128
   {
       singleBlockRead(temp);

          for(p=0;p<bytesPersector;p++)   //512
          {
             if(Data[p]==0 &&Data[p+1]==0)     // find first free space
             {
                 Data[p]=0xFF;
                 clusterforNewFile=p/2;
                 Data[p+1]=0xFF;

                    writeBlock(temp,Data);
                    temp=temp+128;
                    writeBlock(temp,Data);
                    return clusterforNewFile;


             }
          }


          temp=temp+1;
   }
   // write this updated  Fat block  in two position


   putsUart0("\rfat not allocated\r\n");

  return false;

}


void createFile(char *str,uint32_t len)
{
    int i,a=0,p ;
    uint16_t clusterforNewFile;


    uint32_t read32SecofRoot= rootDirectory;
    clusterforNewFile= allocateFatSpace(fatSector);  // fat table updated
    // now update root table

        readRootDirectory(read32SecofRoot);  // after running that global rootDirdata[512] array will gives data
        //read32SecofRoot=read32SecofRoot+1;
        for(i=0;i<512;i=i+32)
        {
           if((rootDirdata[i]==0 && rootDirdata[i+1]==0))
           {
               break;
           }

        }
         for(p=i;p<i+8;p++)
         {
             if(str[a]!='.')
             {
              rootDirdata[p]=str[a];
              a++;
             }
             else
             {
                 a++;
                 p=i+8;
                 break;

             }
         }
         if(str[8]=='.')
         {
             a++;
         }

         for(p=i+8;p<i+11;p++)
         {
             rootDirdata[p]=str[a];
             a++;
         }
         p=i+11;
         rootDirdata[p]=32;    // type of file;
         rootDirdata[p+15]= ((uint8_t)clusterforNewFile);
         rootDirdata[p+16]= ((uint8_t)(clusterforNewFile>>8));
         rootDirdata[p+17]=((uint8_t)len);
         rootDirdata[p+18]=((uint8_t)(len>>8));
         rootDirdata[p+19]=((uint8_t)(len>>16));
         rootDirdata[p+20]=((uint8_t)(len)>>24);



       writeBlock(read32SecofRoot,rootDirdata);




    // send data block that receive
}


void initSd()
{
    // Initialize SPI1 Interface
    initSpi1(USE_SSI_RX);
    setSpi1BaudRate(25e4, 40e6); // 250 khz
    setSpi1Mode(0, 0);
    selectPinPushPullOutput(CS);
    setPinValue(CS, 0);
    setPinValue(CS, 1);
    bool ok =false;
    uint8_t p =0;
    //setTxHigh();
   // setPinValue(SSI1TX, 1);
    setPinValue(CS, 1);                                                       //card i. Set DI and CS high and apply 74 or more clock pulses to SCLK.
    dummySend();
    ok=sendCmd(CMD0);
    if(ok)
    {
        putsUart0("\r>SD Card is in Idle State ..\r\n");

        if(sendCmd(CMD8))              // sd card version 2
        {
            for(p=0;p<15;p++)
            {   ok =false;
                ok=(sendCmd(CMD55) && sendCmd(CMD41));
                if(ok==true)
                {
                    putsUart0("\r>CMD55 & CMD41 Run Successfully..\r\n");
                    //intToString(p);
                  break;
                }
            }
            if((ok && sendCmd(CMD58)))
            {
                putsUart0("\r>SD Card is -Secure Digital High Capacity Card.. \r\n");
                putsUart0("\r>SD Card Initialization is Done  in SPI Mode.. \r\n");
                putsUart0("\r>SD Card is Ready for read & write.. \r\n");



            }
            else
            {
                putsUart0("\r>Error in finding Card Type\r\n");
            }

        }
        else
        {
            // write code for version one
        }
    }
    else
    {
        putsUart0("\r>!Error- Card is not in Idle state\r\n");
    }
    setBlocklen(512);
    readFirstsector() ;
    readLbsector(lbaBeginSec);

}

// ----------------------------FILE TEAM--------------------------


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

// Max packet is calculated as:
// Ether frame header (18) + Max MTU (1500) + CRC (4)

int main(void)
{
    uint8_t* udpData;
    COMMAND command;
    command.x=0;
    command.y=0;
    command.z=0;
    command.f=0;
    command.s=0;
    command.i=0;
    command.j=0;
    delta_x = MIN_XDELTA_MOV;
    delta_y = MIN_YDELTA_MOV;
    // Init controller
    initHw();

    // Setup UART0
    initUart0();
    setUart0BaudRate(115200, 40e6);
    putsUart0("Going Home... \r\n");
    //home();
    initSd();

    // --------------------ETHERNET CONTROL TEAM-------------------------


    // Init ethernet interface (eth0)
    putsUart0("\n\rStarting eth0\n\r");
    etherSetMacAddress(2, 3, 4, 5, 6, 7);
    etherDisableDhcpMode();
    etherSetIpAddress(192, 168, 1, 199);
    etherSetIpSubnetMask(255, 255, 255, 0);
    etherSetIpGatewayAddress(192, 168, 1, 1);
    etherInit(ETHER_UNICAST | ETHER_BROADCAST | ETHER_HALFDUPLEX);
    waitMicrosecond(100000);
    displayConnectionInfo();

    filesCommandempty=true;
    filesDataEmpty=true;


    // --------------------ETHERNET CONTROL TEAM-------------------------

//    uint8_t i;

//    for(i = 0; i < 5; i++)
//    {
//        putsUart0(gcode_test_array[i]);
//        putsUart0("\r\n");
//    }



    // Flash LED
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(100000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(100000);


//    parse_command_PC((char *) "start");
////
//    populate_command(&command);
////
//    check_command(&command);

    // Main Loop
    // RTOS and interrupts would greatly improve this code,
    // but the goal here is simplicity
    while (true)
    {
        // --------------------MOTION CONTROL TEAM-------------------------

        if((step_done == true) && (enable_motion == true))
        {
           if(move_xy == true)
           {
               if(state == STATE_RUN)
                   {
                       move_linear();
                   }


               //after destination reached
               if(actual_segment >= segments)
               {
                   if(linear_mov == true)
                   {
                       state = STATE_STOP;
                       //set_read_gline_status(true);
                   }
                   else
                   {
                       if(actual_angle_segment > ARC_SEGMENTS)
                       {
                           state = STATE_STOP;
                           //set_read_gline_status(true);
                       }
                       else
                       {
                           arc_next_point();
                           line_init(actual_x_arc,actual_y_arc);
                       }
                   }
               }
           }
           else if(move_z == true)
           {
               state = STATE_STOP;
           }
       }

               //disable timer
               if(state == STATE_STOP)
               {
                   update_initPoint(&command);
                   disable_timer();
                   set_read_gline_status(true);
               }

               if((get_read_gline_status()) && (gcode_finished == false))
               {

                   set_read_gline_status(false);

                   char *gcode_line_ptr;

                   //get line from file team
                   gcode_line_ptr = get_line_from_file_buffer();
                   putsUart0(gcode_line_ptr);
                   putsUart0("\r\n");
                   if(ind_file_data >= 512)
                   {
                       ind_file_data = 0;

                       sector_num += 1;

                       singleBlockRead(sector_num);
                   }
                   //parse receive line
                   parse_command_PC(gcode_line_ptr);

                   //populate command structure
                   populate_command(&command);

                   //call gcode command check
                   check_command_gcode(&command);
               }
        // --------------------MOTION CONTROL TEAM-------------------------




        // --------------------ETHERNET CONTROL TEAM-------------------------


               if (kbhitUart0())
                       {
                       }

                       // Packet processing
                       if (etherIsDataAvailable())
                       {
                           if (etherIsOverflow())
                           {
                               setPinValue(RED_LED, 1);
                               waitMicrosecond(100000);
                               setPinValue(RED_LED, 0);
                           }

                           // Get packet
                           if (request < 2)
                           {
                               etherGetPacket(data, MAX_PACKET_SIZE);
                           }
                           else
                           {
                           }

                           // Handle ARP request
                           if (etherIsArpRequest(data))
                           {
                               etherSendArpResponse(data);
                           }

                           // Handle IP datagram
                           if (etherIsIp(data))
                           {
                           if (etherIsIpUnicast(data))
                           {
                               // handle icmp ping request
                               if (etherIsPingRequest(data))
                               {
                                   etherSendPingResponse(data);
                               }

                               // Process UDP datagram
                               // test this with a udp send utility like sendip
                               //   if sender IP (-is) is 192.168.1.198, this will attempt to
                               //   send the udp datagram (-d) to 192.168.1.199, port 1024 (-ud)
                               // sudo sendip -p ipv4 -is 192.168.1.198 -p udp -ud 1024 -d "on" 192.168.1.199
                               // sudo sendip -p ipv4 -is 192.168.1.198 -p udp -ud 1024 -d "off" 192.168.1.199
                               if (etherIsUdp(data))
                               {

                                   initUart0();
                                   destPort = FindDestinationPort(data);
                                   memset(Buffer[request],0 , 513);
                                   udpData = etherGetUdpData(data);
                                   DataBuffer((char *)udpData);
                                   uint8_t *Bufferdata=Buffer[request];

                                   if (destPort == fsComPort)
                                   {
                                       if (filesCommandempty == true)
                                       {
                                           filesCommandempty = false;


           //                                           char *strstatus= "15";
                                           fs = request;
                                           request++;
                                           if (*Bufferdata == dirCom|| *Bufferdata == fsStatusCom)
                                           {
                                               // *strDir=call files dir command function after that make fileCommandResponsebit==true

                                           }

                                           else
                                           {
                                               // call command function after that make fileCommandResponsebit==true
           //                                        parse_command_PC((char *)Bufferdata);
           //                                        putsUart0(split_array[0]);
                                               if (*Bufferdata == fssetactivecom)
                                               {


                                                       parseFileCommand(Buffer[fs]);
                                                       if(activatefile(setActiveFileName))
                                                       {

                                                           fileCommandResponse = true; // read cmd will call by motion team
                                                       }


                                               }
                                               else if (*Bufferdata == fsuploadcom)   // upload
                                               {
                                                   parseFileCommand(Bufferdata);
                                                   fileCommandResponse = true;
                                                   createFile(setActiveFileName,lengthActiveFile);

                                               }
                                               else
                                               {

                                               }
                                               filesCommandempty = true;
                                           }


                                       }

                                       if (fileCommandResponse == true)
                                       {
                                           ResponsetoPC(data, Bufferdata, str);

                                       }
                                   }


                                   if (destPort == mcComPort)
                                   {

                                       if(Bufferdata[0]!=statusCommand)
                                       {
                                       etherSendUdpResponse(data,(uint8_t*) "received", 9);
                                       }

                                           if (*Bufferdata==statusCommand)
                                           {
                                               char word[50];
                                               sprintf(word, "x%f y%f z%f", x_motor, y_motor, command.z);
                                               etherSendUdpResponse(data,(uint8_t*)word, 50);
//                                               sprintf(word, "x%f y%f z%f", x_motor, y_motor, command.z);
//                                               etherSendUdpResponse(data,(uint8_t*) word,50);
                                               request--;
                                           }
                                           else
                                           {
                                              mc = request;
                                              request++;
                                              putsUart0((char *)Buffer[0]);
                                              parse_command_PC((char *)Bufferdata );

                                              populate_command(&command);

                                              //call gcode command check
                                              check_command(&command);

                                                   request--;

                                           }
                                   }

                                   if (destPort == fsDataPort)
                                   {

                                       if (filesDataEmpty == true)
                                       {
                                           request = fs;
                                           request++;
                                           if(activatefile(setActiveFileName))
                                           {  static int f;
                                               for(f=0;f<lengthActiveFile;f++)
                                               {
                                                writeBlock(activeFileStartsector,Buffer[fs]) ;  // write in existing file

                                                fileCommandResponse =true;
                                                 break;

                                               }
                                           }
                                           else
                                           {
                                               putsUart0("file not found");
                                           }

                                           etherSendUdpResponse(data,(uint8_t*) "received", 9);
                                           request--;
                                           //                          filesDataEmpty=false;
                                           // Call Buffer in which files team has created
                                           //                        }

                                       }
                                   }
                               }  // isUdp
                           } // isIpUnicast
                       }
                       // isIp
                   } // isEtherAvail


        // --------------------ETHERNET CONTROL TEAM-------------------------

    }


}
