/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0002 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIO_Stack[TASK_STACKSIZE];
OS_STK SwitchIO_Stack[TASK_STACKSIZE];

// Task Priorities

#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define BUTTONIO_PRIO      8
#define SWITCHIO_PRIO      9

// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define BUTTONIO_PERIOD 300
#define SWITCHIO_PERIOD 300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Engine;

// Semaphores
OS_EVENT *VehicleSem;
OS_EVENT *ControlSem;
OS_EVENT *ButtonIOSem;
OS_EVENT *SwitchIOSem;

// Callbackfunctions
void VehicleCallback(void *ptmr, void *callback_arg) {
  OSSemPost(VehicleSem);
}
void ControlCallback(void *ptmr, void *callback_arg) {
  OSSemPost(ControlSem);
}
void ButtonIOCallback(void *ptmr, void *callback_arg) {
  OSSemPost(ButtonIOSem);
}
void SwitchIOCallback(void *ptmr, void *callback_arg) {
  OSSemPost(SwitchIOSem);
}

// SW-Timer
OS_TMR *VehicleTmr;
OS_TMR *ControlTmr;
OS_TMR *ButtonIOTmr;
OS_TMR *SwitchIOTmr;

/*
 * Types
 */
enum active {on = 2, off = 1};


/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs


/*
 * Helper functions
 */

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

static int b2sLUT[] = {0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */
void VehicleTask(void* pdata)
{ 
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT16U position = 0; 
  INT16S velocity = 0; 
  enum active brake_pedal = off;
  enum active engine = off;

  printf("Vehicle task created!\n");

  while(1)
  {
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

    //OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); 
    OSSemPend(VehicleSem, 0, &err);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_NO_ERR) 
      throttle = (INT8U*) msg;
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 1, &err); 
    if (err == OS_NO_ERR) 
      brake_pedal = (enum active) msg;
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine, 1, &err); 
    if (err == OS_NO_ERR) 
      engine = (enum active) msg;


    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;

    show_velocity_on_sevenseg((INT8S) velocity);
  }
} 

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 40; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* current_velocity;

  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off; 

  printf("Control Task created!\n");

  while(1)
  {
    msg = OSMboxPend(Mbox_Velocity, 0, &err);
    current_velocity = (INT16S*) msg;

    // Here you can use whatever technique or algorithm that you prefer to control
    // the velocity via the throttle. There are no right and wrong answer to this controller, so
    // be free to use anything that is able to maintain the cruise working properly. You are also
    // allowed to store more than one sample of the velocity. For instance, you could define
    //
    // INT16S previous_vel;
    // INT16S pre_previous_vel;
    // ...
    //
    // If your control algorithm/technique needs them in order to function. 

    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);

    //OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);

    OSSemPend(ControlSem, 0, &err);
  }
}

void ButtonIO(void* pdata) {
  INT8U err;
  INT16S* current_velocity;
  void* msg;
  int state;

  enum active cruise_control = off;
  enum active top_gear = off; //////////////////////////////////////////////
  enum active gas_pedal = off;
  enum active brake_pedal = off;

  printf("ButtonIO created!\n");

  while(1) {
    state = buttons_pressed();
    OSSemPend(ButtonIOSem, 0, &err);
    if (state & CRUISE_CONTROL_FLAG) { // button(key) 1 curise_control
      printf("Cruise control is pressed!\n");
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_2);
      if (top_gear == on && gas_pedal == off && brake_pedal == off) {
          msg = OSMboxPend(Mbox_Velocity, 0, &err);
          current_velocity = (INT16S*) msg;
          if (*current_velocity >= 20) {
              printf("Cruise control is activated!\n");
              cruise_control = on;
              //IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_0);
              IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_2 | LED_GREEN_0);
          }
      }
    }
    else if (state & BRAKE_PEDAL_FLAG) { // button(key) 2 brake_pedal
      printf("Brake is on!\n");
      brake_pedal = on;
      cruise_control = off;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_4);
    }
    else if (state & GAS_PEDAL_FLAG) { // button(key) 3 gas_pedal
      printf("Gas pedal is on!\n");
      gas_pedal = on;
      cruise_control = off;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_6);
    }
    else {
      gas_pedal = off;
      brake_pedal = off;
      if (cruise_control == on) {
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_0);
      }
      else {
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, 0);
      }
    }

  }
}

void SwitchIO(void* pdata) {
  INT8U err;
  INT16S* current_velocity;
  void* msg;
  int state;

  enum active engine = off;
  enum active top_gear = off;
  enum active cruise_control = off; //////////////////////////////////////////

  printf("SwitchIO Created!\n");

  while (1) {
    state = switches_pressed();
    OSSemPend(SwitchIOSem, 0, &err);
    if (state == (ENGINE_FLAG + TOP_GEAR_FLAG)) {
      printf("Engine and Gear are on!\n");
      engine = on;
      top_gear = on;
      //IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0);
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0 | LED_RED_1);
    }
    else if (state == ENGINE_FLAG) {
      printf("Only Engine is on!\n");
      engine = on;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0);
      top_gear = off;
      cruise_control = off;
    }
    else if (state == TOP_GEAR_FLAG) {
      top_gear = on;
      msg = OSMboxPend(Mbox_Velocity, 0, &err);
      current_velocity = (INT16S*) msg;
      if (*current_velocity == 0) {
        engine = off;
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_1);
        printf("Only Gear is on!\n");
      }
      else {
        engine = on;
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0 | LED_RED_1);
        printf("Gear is on! Engine is also on! Velocity > 0!\n");
      }
    }
    else {
      top_gear = off;
      cruise_control = off;
      msg = OSMboxPend(Mbox_Velocity, 0, &err);
      current_velocity = (INT16S*) msg;
      if (*current_velocity == 0) {
        engine = off;
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, 0);
        printf("Engine and Gear are off!\n");
      }
      else {
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0);
        printf("Engine is still on! Velocity > 0!\n");
      }
    }
  }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!n");
  }

  /* 
   * Create and start Software Timer 
   */

  VehicleTmr = OSTmrCreate(0, // delay
                           VEHICLE_PERIOD/100, //period
                           OS_TMR_OPT_PERIODIC, //timer to automatically reload itself
                           VehicleCallback, //callback function
                           (void*) 0,
                           "VehicleTmr",
                           &err);

  ControlTmr = OSTmrCreate(0, // delay
                           CONTROL_PERIOD/100, //period-------------------------------------------------------------------------------
                           OS_TMR_OPT_PERIODIC, //timer to automatically reload itself
                           ControlCallback, //callback function
                           (void*) 0,
                           "ControlTmr",
                           &err);
                           
  ButtonIOTmr = OSTmrCreate(0,
                            BUTTONIO_PERIOD/100,
                            OS_TMR_OPT_PERIODIC,
                            ButtonIOCallback,
                            (void*) 0,
                            "ButtonIOTmr",
                            &err);

  SwitchIOTmr = OSTmrCreate(0,
                            SWITCHIO_PERIOD/100,
                            OS_TMR_OPT_PERIODIC,
                            SwitchIOCallback,
                            (void*) 0,
                            "SwitchIOTmr",
                            &err);
  
  // Start timer
  OSTmrStart(VehicleTmr, &err);
  OSTmrStart(ControlTmr, &err); 
  OSTmrStart(ButtonIOTmr, &err);
  OSTmrStart(SwitchIOTmr, &err);
  
  // Create Semaphores
  VehicleSem = OSSemCreate(1);
  ControlSem = OSSemCreate(1);
  ButtonIOSem = OSSemCreate(1);
  SwitchIOSem = OSSemCreate(1);

  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 1); /* Empty Mailbox - Velocity */
  Mbox_Engine = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine */

  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      ButtonIO, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ButtonIO_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      BUTTONIO_PRIO,
      BUTTONIO_PRIO,
      (void *)&ButtonIO_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      SwitchIO, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &SwitchIO_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      SWITCHIO_PRIO,
      SWITCHIO_PRIO,
      (void *)&SwitchIO_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);      

  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSStart();

  return 0;
}
