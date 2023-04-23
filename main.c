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
#include "uart0.h"
#include "wait.h"

// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4

#define MAX_SPLIT_LEN 5
#define MAX_SPLIT_BRE 10
#define MAX_LEN_LINE  80
#define CHAR_SPACE    0x20

#define STATE_RUN   1
#define STATE_PAUSE 2
#define STATE_STOP  3

float MIN_XDELTA_MOV = 0.1;
float MIN_YDELTA_MOV = 0.1;
#define PROCESSOR_CLOCK_FREQ 40000000
#define MIN_TO_SEC_MULTIPLIER 60
#define ARC_SEGMENTS 10
#define COUNTERCLOCKWISE 1

#define MAX_PACKET_SIZE 1522
uint8_t data[MAX_PACKET_SIZE];

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
} COMMAND;


char split_array[MAX_SPLIT_BRE][MAX_SPLIT_LEN];
uint8_t state;
bool start_read_gline;


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

char gcode_test_array[5][25] = {//"G00 X0 Y0 F70\0",
                                "G00 X0 Y50\0",
                                "G02 X50 Y0 I0 J-50\0",
                                "G03 X0 Y50 I-50 J0\0",
                                "G00 X0 Y0\0",
                                "M30\0"};

//char gcode_test_array[5][25] = {//"G00 X0 Y0 F70\0",
//                                "G01 X20 Y0\0",
//                                "G01 Z30\0",
//                                "G01 Z0\0",
//                                "M30\0"};

//char gcode_test_array[2][25] = {//"G00 X0 Y0 F70\0",
//                                "G01 X50 Y50\0",
////                                "G01 Z30\0",
////                                "G01 Z0\0",
//                                "M30\0"};


//char gcode_test_array[6][20] = {"G00 X0 Y0 F70\0",
//                                "G01 X0 Y20 F50\0",
//                                "G01 X25 Y20\0",
//                                "G01 X25 Y0\0",
//                                "G01 X0 Y0\0",
//                                "M30\0"};

/* GCODE TEST ARRAY */


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

bool gcode_finished;
bool enable_motion;
bool step_done=true;
bool move_xy;
bool move_z;

// --------------------MOTOR CONTROL TEAM-------------------------

uint32_t tick_count_x = 0;
uint32_t tick_count_y = 0;
uint32_t tick_count_z = 0;
bool axisLimitError = 0; // flag when cnc hits one of the min or max on any axes; 0 = no error, 1 = error
bool Home_Flag = 0;
//bool step_done = 0; // flag for motion control team; 1 = done, 0 = in progress
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
#define Pos_Dir 0
#define Neg_Dir 1
#define Posy_Dir 1
#define Negy_Dir 0
// -----------------------------------------------



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
// --------------------MOTOR CONTROL TEAM-------------------------
void move(char axis, uint8_t dir, float dist1);
//void move(char axis, uint8_t dir, uint32_t dist);

//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------


void home(void)
{
    Home_Flag = 1;
    NVIC_ST_CTRL_R = 0;
    while (Xmin != 1)
    {
        XDir = Neg_Dir;
        PWM0_0_CMPA_R = 10000;
    }
    XDir = Pos_Dir;
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
        ZDir = Neg_Dir;
        PWM0_1_CMPB_R = 10000;
    }
        ZDir = Pos_Dir;
        PWM0_1_CMPB_R = 0;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_CLK_SRC;

    waitMicrosecond(500000);
    move('x', Pos_Dir, 10);
    move('y', Posy_Dir, 10);
    move('z', Pos_Dir, 50);
    waitMicrosecond(2000000);
//    move('x', Pos_Dir, 145);
// //   move('y', Posy_Dir, 10);
//    move('z', Pos_Dir, 180);
//    waitMicrosecond(2000000);
    Home_Flag = 0;
 //  while(1);

}

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

        // Port F config
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
    if(str_compare(command_struct_ptr -> command, "goto"))
    {
        state = STATE_RUN;
        move_xy=1;
        move_z=0;
        line_init(command_struct_ptr->x, command_struct_ptr->y);
        linear_mov = true;
        enable_timer();
    }
    else if(str_compare(command_struct_ptr -> command, "pause"))
    {
        state = STATE_PAUSE;

        disable_timer();
    }
    else if(str_compare(command_struct_ptr -> command, "resume"))
    {
        if(state != STATE_STOP)
        {
            state = STATE_RUN;
            enable_timer();
        }
    }
    else if(str_compare(command_struct_ptr -> command, "stop"))
    {
        state = STATE_STOP;

        disable_timer();
    }
    else if (str_compare(command_struct_ptr -> command, "start"))
    {
        state = STATE_RUN;

        gcode_finished = false;

        set_read_gline_status(true);
    }

    /* implement arc2 */

}


void check_command_gcode(COMMAND *command_struct_ptr)
{
    if(str_compare(command_struct_ptr -> command, "G00") || str_compare(command_struct_ptr -> command, "G01"))
    {
        //uint32_t load;

        state = STATE_RUN;

        if(command_struct_ptr ->z != z_init)
        {
            move_xy=0;
            move_z=1;
            if(command_struct_ptr ->z > z_init)
            {
                move('z',Pos_Dir,command_struct_ptr ->z - z_init);
            }
            else
            {
                move('z',Neg_Dir,z_init - command_struct_ptr ->z);
            }
        }
        else
        {
            //timer load value calculations
            //load = calculate_timer_load(command_struct_ptr);

            //set_timer_load(load);
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
    else if(str_compare(command_struct_ptr -> command, "M30"))
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
                   PWM0_1_CTL_R = 0;
               }
           }
       }

       if ((tick_count_x == 0) && (tick_count_y == 0) && (tick_count_z == 0))
       {
           step_done = 1;
       }
       if (Home_Flag == 0)
       {
//           if (Xmin || Xmax || Ymin || Ymax || Zmin || Zmax)
//           {
//               PWM0_0_CMPA_R = 0;
//               PWM0_0_CMPB_R = 0;
//               PWM0_1_CMPB_R = 0;
//               //PWM0_0_CTL_R = 0;
//               PWM0_1_CTL_R = 0;
//               tick_count_x = 0;
//               tick_count_y = 0;
//               tick_count_z = 0;
//               if (step_done == 0)
//               {
//                   axisLimitError = 1;
//               }
//           }
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
        PWM0_1_CTL_R = PWM_0_CTL_ENABLE;
        PWM0_1_CMPA_R = 0;
        PWM0_1_CMPB_R = 0;
        ZDir = dir;
        tick_count_z = (dist/7.3) * (1000)/1.6;
        PWM0_1_CMPA_R = 0;
        PWM0_1_CMPB_R = 10000;
    }

}

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
    //sprintf(str, "%u", etherGetRevId());
    putsUart0(str);
    putsUart0("\n\r");

    if (etherIsLinkUp())
        putsUart0("Link is up\n\r");
    else
        putsUart0("Link is down\n\r");
}



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

    // Init controller
    initHw();

    // Setup UART0
    initUart0();
    setUart0BaudRate(115200, 40e6);
   // TP_EN = 1;
    putsUart0("Going Home... \r\n");
    home();

//    move('x',0,1);
//    move('y',0,1);
//    step_done=0;
//    move('x',0,1);
//    move('y',0,1);
//    step_done=0;
//    move('x',0,1);
//    move('y',0,1);
//    step_done=0;
//    move('x',0,1);
//    move('y',0,1);
//    step_done=0;


    uint8_t i;

    for(i = 0; i < 5; i++)
    {
        putsUart0(gcode_test_array[i]);
        putsUart0("\r\n");
    }



    // Flash LED
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(100000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(100000);


    delta_x = MIN_XDELTA_MOV;
    delta_y = MIN_YDELTA_MOV;

    parse_command_PC((char *) "start");

    populate_command(&command);

    check_command(&command);

    // Main Loop
    // RTOS and interrupts would greatly improve this code,
    // but the goal here is simplicity
    while (true)
    {
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
            gcode_line_ptr = gcode_test_array[counter];

            //parse receive line
            parse_command_PC(gcode_line_ptr);

            //populate command structure
            populate_command(&command);

            //call gcode command check
            check_command_gcode(&command);
            counter += 1;
        }
    }
}
