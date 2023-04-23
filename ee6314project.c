//Lab3b
//Liem Nguyen

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "uart0.h"

// Bitband aliases
#define Red  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))//PF1 red LED
// X axis bitband
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
//define
#define Pos_Dir 0
#define Neg_Dir 1
uint32_t tick_count = 0;

#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{ char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

#define NULL 0
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void move(char* axis, uint8_t dir, uint32_t dist);
void getsUart0(USER_DATA* data);
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments);
void parseFields(USER_DATA *data);
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
bool mystrcmp(char *str, const char strCommand[]);
void systickIsr()
{
    if (tick_count > 0)
    {
        tick_count--;
        if (tick_count == 0)
        {
            PWM0_0_CTL_R = 0;
            PWM0_1_CTL_R = 0;
        }
    }
    if (Xmin)
    {
        Red = 1;
        XDir = Pos_Dir;
        while (Xmin);
    }
    else if (Xmax)
    {
        Red = 0;
        XDir = Neg_Dir;
        while (Xmax);
    }
    if (Ymin)
        {
            Red = 1;
            YDir = Neg_Dir;
            while (Xmin);
        }
        else if (Ymax)
        {
            Red = 0;
            YDir = Pos_Dir;
            while (Xmax);
        }
}
// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R4| SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R3| SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Port B config
    //pwm
    GPIO_PORTB_DIR_R |= PWMX_MASK|PWMY_MASK | PWMZ_MASK;
    GPIO_PORTB_DEN_R |= PWMX_MASK|PWMY_MASK| PWMZ_MASK;
    GPIO_PORTB_AFSEL_R |= PWMX_MASK|PWMY_MASK| PWMZ_MASK;

    GPIO_PORTB_DIR_R &= ~(Zmin_MASK | Zmax_MASK);
    GPIO_PORTB_PDR_R |= Zmin_MASK | Zmax_MASK;
    GPIO_PORTB_DEN_R |= Zmin_MASK | Zmax_MASK;
    GPIO_PORTB_AFSEL_R |= Zmin_MASK | Zmax_MASK;

    //Port C config
    GPIO_PORTC_DIR_R &= ~(Xmin_MASK | Xmax_MASK|Ymin_MASK|Ymax_MASK);
    GPIO_PORTC_PDR_R |= Xmin_MASK | Xmax_MASK|Ymin_MASK|Ymax_MASK;
    GPIO_PORTC_DEN_R |= Xmin_MASK | Xmax_MASK|Ymin_MASK|Ymax_MASK;
    GPIO_PORTC_AFSEL_R |= Xmin_MASK | Xmax_MASK|Ymin_MASK|Ymax_MASK;

    // Port F config
    GPIO_PORTD_LOCK_R |= GPIO_LOCK_KEY | GPIO_LOCK_UNLOCKED;
    GPIO_PORTD_CR_R |= 0xFF;
    GPIO_PORTD_DIR_R |= XDIR_MASK|YDIR_MASK;
    GPIO_PORTD_DEN_R |= XDIR_MASK|YDIR_MASK;

//    GPIO_PORTF_DIR_R |= RED_RGB_MASK;
//    GPIO_PORTF_DEN_R |= RED_RGB_MASK;

    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB6_M|GPIO_PCTL_PB7_M|GPIO_PCTL_PB5_M);
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB6_M0PWM0|GPIO_PCTL_PB7_M0PWM1|GPIO_PCTL_PB5_M0PWM3;
    // Configure PWM module
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_0_CTL_R = 0; // turn-off PWM0
    PWM0_1_CTL_R = 0;// turn-off PWM0
    //  PWM1_3_CTL_R = 0;         // turn-off PWM1 generator 3 (drives outs 6 and 7)
    //  PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    // output 5 on PWM1, gen 2b, cmpb
    PWM0_0_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;//x
    PWM0_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;//x
    PWM0_1_GENB_R = PWM_1_GENB_ACTCMPAD_ONE | PWM_1_GENB_ACTLOAD_ZERO;//z
    PWM0_0_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;//y
    // output 6 on PWM1, gen 3a, cmpa
    //  PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
    // output 7 on PWM1, gen 3b, cmpb

    // PWM0_0_LOAD_R = 50000; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_0_LOAD_R = 20000;    //1.6Khz
    PWM0_1_LOAD_R = 20000;    //1.6Khz
    //   PWM1_3_LOAD_R = 1024;

    PWM0_0_CMPA_R = 0;
    PWM0_0_CMPB_R = 0;  // red off (0=always low, 1023=always high)
    PWM0_1_CMPB_R= 0 ;
    PWM0_1_CMPA_R = 0;
    //  PWM1_3_CMPB_R = 0;                               // green off
    //  PWM1_3_CMPA_R = 0;                               // blue off

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;
    PWM0_1_CTL_R = PWM_0_CTL_ENABLE; // turn-on PWM1 generator 2
    // PWM1_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN|PWM_ENABLE_PWM1EN|PWM_ENABLE_PWM3EN ;

    //Systick config
//    NVIC_ST_CURRENT_R = 0;
//    NVIC_ST_RELOAD_R = 40000;
//    NVIC_ST_CTRL_R = 0;
//    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN| NVIC_ST_CTRL_CLK_SRC;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();
    USER_DATA data;
    bool valid;
    setUart0BaudRate(115200, 40e6);
   // YDir = Pos_Dir;
  //  XDir = Pos_Dir;
  //  PWM0_0_CMPA_R = 6250;
  //  PWM0_0_CMPB_R = 6250;
    move( "y", 1, 500);
    while(tick_count > 0);
    move( "x", 1, 500);
    while(tick_count > 0);
    move( "y", 0, 500);
    while(tick_count > 0);
    move( "x", 0, 500);
    while (true)
    {
        Red = 1;
        valid = 0;
        getsUart0(&data);
        putsUart0(data.buffer);
        putsUart0("\r\n");
        parseFields(&data);
        // Setup UART0 baud rate
        if (isCommand(&data, "move", 3))
        {
            char *dir = getFieldString(&data, 2);
            char *axis = getFieldString(&data, 1);
            uint32_t dist = getFieldInteger(&data, 3);
            if (mystrcmp(dir, "p"))
            {
                move(axis,Pos_Dir,dist);
                valid = true;
            }
            else if (mystrcmp(dir, "n"))
            {
                move(axis,Neg_Dir,dist);
                valid = true;
            }
        }
        else if (isCommand(&data, "stop", 0))
        {
            PWM0_0_CTL_R = 0;
            PWM0_0_CMPA_R = 0;
            PWM0_0_CMPB_R = 0;
            tick_count = 0;
            valid=true;
        }
        if (!valid)
        {
            putsUart0("Invalid Command\r\n");
        }



    }
}

void move(char* axis, uint8_t dir, uint32_t dist)
{
    while (tick_count !=0);
    if (mystrcmp(axis, "x"))
    {
        PWM0_0_CTL_R = PWM_0_CTL_ENABLE;
        PWM0_0_CMPA_R = 0;
        PWM0_0_CMPB_R = 0;
        XDir = dir;
        tick_count = dist * (1000 / 94.12);
        PWM0_0_CMPA_R = 10000;
        PWM0_0_CMPB_R = 0;
    }
    else if (mystrcmp(axis, "y"))
    {
        PWM0_0_CTL_R = PWM_0_CTL_ENABLE;
        PWM0_0_CMPA_R = 0;
        PWM0_0_CMPB_R = 0;
        YDir = dir;
        tick_count = dist * (1000 / 94.12);
        PWM0_0_CMPA_R = 0;
        PWM0_0_CMPB_R = 10000;
    }
    else if (mystrcmp(axis, "z"))
    {
        PWM0_1_CTL_R = PWM_0_CTL_ENABLE;
        PWM0_1_CMPA_R = 0;
        PWM0_1_CMPB_R = 0;
        ZDir = dir;
        tick_count = dist * (1000 / 94.12);
        PWM0_1_CMPA_R = 0;
        PWM0_1_CMPB_R = 10000;
    }

}
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if (minArguments <= (data->fieldCount - 1))
    {
        int i = data->fieldPosition[0];             // first field
        int j = 0;
        while (strCommand[j] != NULL ||data->buffer[i]!=NULL )
        {
            if ((strCommand[j] - data->buffer[i]) != 0)
            {
                return 0;
            }
            i++;
            j++;
        }
        return 1;
    }
    else
        return 0;
}
void parseFields(USER_DATA *data)
{
    data->fieldCount = 0;
    int i;
    int offset = 0;
    int state = 0; //0: delimiter, 1: numeric, 2:alpha
    for (i = 0; i <= MAX_CHARS; i++)
    {
        if (data->buffer[i] == 0)
        {
            i = MAX_CHARS;
            return;
        }
        if (data->buffer[i] >= 48 && data->buffer[i] <= 57
                || data->buffer[i] == 45 || data->buffer[i] == 46)
        {
            if (state != 1 && state != 2)
            {
                data->fieldType[offset] = 'n';
                data->fieldPosition[offset] = i;
                data->fieldCount++;
                offset++;
                state = 1;
            }
            else
            {
                goto cont;
            }
        }
        else if (data->buffer[i] >= 65 && data->buffer[i] <= 90)
        {
            if (state != 2 & state != 1)
            {
                data->fieldType[offset] = 'a';
                data->fieldPosition[offset] = i;
                data->fieldCount++;
                offset++;
                state = 2;
            }
            else
            {
                goto cont;
            }
        }
        else if (data->buffer[i] >= 97 && data->buffer[i] <= 122)
        {
            if (state != 2 && state != 1)
            {
                data->fieldType[offset] = 'a';
                data->fieldPosition[offset] = i;
                data->fieldCount++;
                offset++;
                state = 2;
            }
            else
            {
                goto cont;
            }
        }
        else
        {
            state = 0;
            data->buffer[i] = 0;
        }
        cont: if (data->fieldCount == MAX_CHARS)
        {
            i = MAX_CHARS;
            return;
        }

    }

}
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    int number = 0;
    int offset = 0;
    if (fieldNumber <= data->fieldCount && data->fieldType[fieldNumber] == 'n')
    {
        offset = data->fieldPosition[fieldNumber];
        while (data->buffer[offset] != 0)
        {
            number = number * 10 + data->buffer[offset] - '0';
            offset++;
        }
        return number;
    }
    else
        return NULL;
}
char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{

    if (fieldNumber <= data->fieldCount & data->fieldType[fieldNumber] == 'a')
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
        return NULL;
}
bool mystrcmp(char *str, const char strCommand[])
{
    int i = 0;
    while (strCommand[i] != NULL||str[i]!=NULL)
    {
        if (strCommand[i] - str[i] != 0)
        {
            return 0;
        }
        i++;
    }
    return 1;
}
void getsUart0(USER_DATA* data)
{
    uint8_t count=0;
    char n;
    int i=1;
    while (i)
    {
    n= getcUart0();
    if(count== MAX_CHARS )
    {
        data->buffer[count]=0;
        i=1;
        return;
    }
    else if (n == 8 | n == 127 & count > 0)
    {
        count--;
    }
    else if (n == 13 )
      {
          data->buffer[count]=0;
          i=0;
          return;
      }
    else if (n>=32)
    {
        data->buffer[count]=n;
        count++;
    }

    }
}
