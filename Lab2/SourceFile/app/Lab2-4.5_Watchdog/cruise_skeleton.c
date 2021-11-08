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
#include "altera_avalon_performance_counter.h"

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

#define LED_RED_12 0x1000     // [2000m, 2400m]
#define LED_RED_13 0x2000     // [1600m, 2000m)
#define LED_RED_14 0x4000     // [1200m, 1600m)
#define LED_RED_15 0x8000     // [800m, 1200m)
#define LED_RED_16 0x10000    // [400m, 800m)
#define LED_RED_17 0x20000    // [0m, 400m)


//Definition of Tasks

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];
OS_STK SwitchIOTask_Stack[TASK_STACKSIZE];
OS_STK OverloadDetectionTask_Stack[TASK_STACKSIZE];
OS_STK WatchDogTask_Stack[TASK_STACKSIZE];
OS_STK ExtraLoadTask_Stack[TASK_STACKSIZE];

// Task Priorities
#define WATCHDOG_PRIO 6
#define STARTTASK_PRIO     5
#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  11
#define BUTTONIO_PRIO 12
#define SWITCHIO_PRIO 13
#define OVERLOAD_PRIO 15
#define EXTRALOADTASK_PRIO 14
// Task Periods

#define WATCHDOG_PERIOD 300
#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define SWITCHIO_PERIOD 300
#define BUTTONIO_PERIOD 300
#define OVERLOAD_PERIOD 300
#define EXTRALOAD_PERIOD 300

//Definition of Kernel Objects 

// Mailboxes
OS_EVENT * Mbox_Throttle;
OS_EVENT * Mbox_Velocity;
OS_EVENT * Mbox_Brake;
OS_EVENT * Mbox_Engine;
OS_EVENT * Mbox_Cruise_Control;
OS_EVENT * Mbox_Gas_Pedal;
OS_EVENT * Mbox_Gear;
OS_EVENT * Mbox_Cruise_Velocity;

// Semaphores
OS_EVENT * VehicleTmrSem;
OS_EVENT * ControlTmrSem;
OS_EVENT * ButtonIOSem;
OS_EVENT * SwitchIOSem;
OS_EVENT * OverloadDetectionSem;
OS_EVENT * WatchDogSem;
OS_EVENT * ExtraLoadSem;

// SW-Timer
OS_TMR * VehicleTmr;
OS_TMR * ControlTmr;
OS_TMR * ButtonIOTmr;
OS_TMR * SwitchIOTmr;
OS_TMR * OverloadDetectionTmr;
OS_TMR * WatchDogTmr;
OS_TMR * ExtraLoadTmr;

//Types

enum active {on = 2, off = 1};

//Global variables
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs


//Helper functions
void VehicleTmrCallback(){
  OSSemPost(VehicleTmrSem);
  //printf("Vehicel call back!");
}

void ControlTmrCallback(){
  OSSemPost(ControlTmrSem);
  //printf("Control call back!");
}

void ButtonIOCallback(){
  OSSemPost(ButtonIOSem);
  //printf("Button call back!");
}

void SwitchIOCallback(){
  OSSemPost(SwitchIOSem);
  //printf("Switch call back!");
}

void OverloadDetectionCallback(){
  OSSemPost(OverloadDetectionSem);
  //printf("Overload detectin call back!");
}

void WatchDogCallback(){
  OSSemPost(WatchDogSem);
}

void ExtraLoadCallback(){
  //printf("extra load call back\n");
  OSSemPost(ExtraLoadSem);
}

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

//ISR for HW Timer
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
 //convert int to seven segment display format

int int2seven(int inval){
  return b2sLUT[inval];
}

// output current velocity on the seven segement display
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

 // shows the target velocity on the seven segment display (HEX5, HEX4)
 // when the cruise control is activated (0 otherwise)
void show_target_velocity(INT8U target_vel)
{
  int tmp = target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = 
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

int led_ON = 0; //position led

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
  int led = 0;
  //led_ON = 0;
  if (position >= 0 && position < 400)
    led = LED_RED_17;
  else if (position >= 400 && position < 800)
    led = LED_RED_16;
  else if (position >= 800 && position < 1200)
    led = LED_RED_15;
  else if (position >= 1200 && position < 1600)
    led = LED_RED_14;
  else if (position >= 1600 && position < 2000)
    led =  LED_RED_13;
  else if (position >= 2000 && position <= 2400)
    led =  LED_RED_12;
  
  led_ON = led_ON | led;
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
  enum active * brake_pedal_pointer;
  enum active * engine_pointer;

  printf("Vehicle task created!\n");

  while(1)
  {
    
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);
    OSSemPend(VehicleTmrSem, 0, &err);
    //OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); 

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_NO_ERR) 
      throttle = (INT8U*) msg;
      int t = (int) throttle;
      //printf("throttle: %d\n", *throttle);
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 1, &err); 
    err = OSMboxPost(Mbox_Brake, msg);
    if (err == OS_NO_ERR) {
      brake_pedal_pointer = (enum active *) msg;
      brake_pedal = *brake_pedal_pointer;
    }
      
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine, 1, &err); 
    if (err == OS_NO_ERR) {
      engine_pointer = (enum active *) msg;
      engine = *engine_pointer;
    }
      
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
    show_position(position);
  }
} 

int cruise_active = 0;
/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */


void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 40; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S * current_velocity;
  INT16S * maintained_velocity;

  enum active gas_pedal;
  enum active top_gear;
  enum active cruise_control; 
  enum active brake_pedal;

  enum active * helper_pointer;
  INT16S initial_cruise_velocity = 0; 
  err = OSMboxPost(Mbox_Cruise_Velocity, (void *) &initial_cruise_velocity);
  printf("Control Task created!\n");

  while(1)
  {
    printf("control task loop\n");
    OSSemPend(ControlTmrSem, 0, &err);
    msg = OSMboxPend(Mbox_Velocity, 1, &err); //current velocity
    //err = OSMboxPost(Mbox_Velocity, msg);
    current_velocity = (INT16S*) msg;
    err = OSMboxPost(Mbox_Velocity, msg); 
    //err = OSMboxPost(Mbox_Velocity, msg);
    
    msg = OSMboxPend(Mbox_Gear, 1, &err); //top gear
    helper_pointer = (enum active *) msg;
    top_gear = *helper_pointer;
    err = OSMboxPost(Mbox_Gear, msg);
    printf("top gear: %d\n", top_gear);
    msg = OSMboxPend(Mbox_Cruise_Control, 1, &err); //cruise control
    helper_pointer = (enum active *) msg;
    cruise_control = *helper_pointer; 
    printf("cruise control: %d\n", cruise_control);
    msg = OSMboxPend(Mbox_Gas_Pedal, 1, &err); //gas pedal
    helper_pointer = (enum active *) msg;
    gas_pedal = *helper_pointer;
    msg = OSMboxPend(Mbox_Brake, 1, &err); //brake
    helper_pointer = (enum active *) msg;
    brake_pedal = *helper_pointer;
    //err = OSMboxPost(Mbox_Brake, msg);
    msg = OSMboxPend(Mbox_Cruise_Velocity, 1, &err); //maintained velocity
    err = OSMboxPost(Mbox_Cruise_Velocity, msg);
    maintained_velocity = (INT16S *) msg;
    printf("current velocity: %d\n", *current_velocity);
    printf("maintained velocity: %d\n", *maintained_velocity);
    
    //printf("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh\n");
    //int * t = (int *) maintained_velocity;
    //printf("maintained velocity: %d\n", *maintained_velocity);
    
    printf("cruise control: %d, top_gear: %d, gas pedal: %d, brake pedal: %d\n", 
    cruise_control, top_gear, gas_pedal, brake_pedal);
    if (gas_pedal == on){
      throttle = 80;
    }
    else {
      throttle = 40;
    }

    if (cruise_control == on && top_gear == on && gas_pedal == off && 
        brake_pedal == off && *maintained_velocity >= 25){
        cruise_active = 1;
        printf("cruise active\n");
      if (*current_velocity < *maintained_velocity)
        throttle = throttle + 10;
      else if (*current_velocity > *maintained_velocity)
        throttle = throttle - 10;
    }
    else{
      cruise_active = 0;
      *maintained_velocity = 0;
      msg = OSMboxPend(Mbox_Cruise_Velocity, 1, &err); 
      err = OSMboxPost(Mbox_Cruise_Velocity, (void *) maintained_velocity);
    }
      
  

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
    //printf("throttlethrottletheottle: %d\n", throttle);

    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);
    show_target_velocity(*maintained_velocity);
    // OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);*/
  }
}

void ButtonIOTask(void * pdtata){
  printf("ButtonIOTask created!\n");
  INT8U err;  
  void * msg;
  int state;
  INT16S * current_velocity;
  enum active gas_pedal = off;
  enum active cruise_control = off; 
  enum active brake_pedal = off;
  enum active brake = off;
  enum active top_gear = off;
  enum active * helper_pointer; 
  int LED_Cruise_Active = 0;
  while (1){
    state = buttons_pressed();
    OSSemPend(ButtonIOSem, 0, &err);
    msg = OSMboxPend(Mbox_Gear, 1, &err);
    helper_pointer = (enum active *) msg;
    top_gear = *helper_pointer;
    msg = OSMboxPend(Mbox_Velocity, 1, &err);  
    //err = OSMboxPost(Mbox_Velocity, msg); 
    current_velocity = (INT16S *) msg;
    //err = OSMboxPost(Mbox_Velocity, msg);
    printf("current velocity in switchIO is %d\n", *current_velocity);
    if (cruise_active == 1) 
      LED_Cruise_Active = LED_GREEN_0;
    else 
      LED_Cruise_Active = 0;
    if (state & CRUISE_CONTROL_FLAG){   //cruise_control button is pressed
      printf("cruise_control button is on. \n");      
      
      
      if (*current_velocity >= 25 && top_gear == on){
        //err = OSMboxPost(Mbox_Velocity, (void *) current_velocity);
        int maintained_velocity = *current_velocity;
        printf("ButtonIOTask-maintained velocity: %d\n", maintained_velocity);
        //msg = OSMboxPend(Mbox_Cruise_Control, 1, &err);
        //err = OSMboxPost(Mbox_Cruise_Control, msg);
        msg = OSMboxPend(Mbox_Cruise_Velocity, 1, &err);
        err = OSMboxPost(Mbox_Cruise_Velocity, (void *) &maintained_velocity);
        cruise_control = on;
      }
      
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_2 | LED_Cruise_Active);
    }
    else if (state & BRAKE_PEDAL_FLAG){ //brake button is pressed
      brake = on;
      cruise_control = off;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_4 | LED_Cruise_Active);
    } 
    else if (state & GAS_PEDAL_FLAG){  //gas pedal is pressed
      gas_pedal = on;
      cruise_control = off;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_6 | LED_Cruise_Active);
    } 
    else {
      brake = off;
      gas_pedal = off;
      //msg = OSMboxPend(Mbox_Cruise_Control, 1, &err);
      //err = OSMboxPost(Mbox_Cruise_Control, msg);
      //enum active * t = (enum active *) msg;
      //printf("cruise control: %d \n", *t);
      //if (*t == on){
        //IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_0);
      //}
      //else 
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_Cruise_Active);
    }
    //msg = OSMboxPend(Mbox_Cruise_Control, 1, &err);
    //msg = OSMboxPend(Mbox_Brake, 1, &err);
    //msg = OSMboxPend(Mbox_Gas_Pedal, 1, &err);
    printf("Cruise_control_ task: %d\n", cruise_control);
    err = OSMboxPost(Mbox_Cruise_Control, (void *) &cruise_control);
    err = OSMboxPost(Mbox_Brake, (void *) &brake);
    err = OSMboxPost(Mbox_Gas_Pedal, (void *) &gas_pedal);
  }
}

void SwitchIOTask(void * pdtata){
  printf("SwitchIOTask created!\n");
  INT8U err;  
  void * msg;
  int state;
  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off; 
  enum active brake_pedal = off;
  enum active engine = off;
  while (1){
    state = switches_pressed();
    state = 0x00F & state;
    OSSemPend(SwitchIOSem, 0, &err);
    if (state == (TOP_GEAR_FLAG + ENGINE_FLAG)){
      top_gear = on;
      engine = on;      
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0 | LED_RED_1 | led_ON);
    }
    else if (state & ENGINE_FLAG){
      engine = on;
      top_gear = off;
      cruise_control = off;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0 | led_ON);
    }
    else if (state & TOP_GEAR_FLAG){
      top_gear = on;
      engine = off;
      cruise_control = off;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_1 | led_ON);
    } 
    else {
      engine = off;
      top_gear = off;
      cruise_control = off;
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,  led_ON);
    }
    //msg = OSMboxPend(Mbox_Engine, 1, &err);
    //msg = OSMboxPend(Mbox_Gear, 1, &err);
    //msg = OSMboxPend(Mbox_Cruise_Control, 1, &err);
    err = OSMboxPost(Mbox_Engine, (void *) &engine);
    err = OSMboxPost(Mbox_Gear, (void *) &top_gear);
    //led_ON = 0;
    //msg = OSMboxPend(Mbox_Cruise_Control, 1, &err);
    //err = OSMboxPost(Mbox_Cruise_Control, (void *) &cruise_control);
  }
}

//The task 'StartTask' creates all other tasks kernel objects and deletes itself afterwards. 

int OK = 1;

void OverloadDetectionTask(void * pdtata){
  INT8U err;
  printf("Overload created \n");  
  while (1){
    OSSemPend(OverloadDetectionSem, 0, &err);
    OK = 1;
    printf("Overload detection working!\n");    
  }
}

void WatchDogTask(void * pdtata){
  printf("Watchdog created.\n");
  INT8U err;
  while(1){
    OSSemPend(WatchDogSem, 0, &err);
    if (OK == 0){
      printf("Overload!\n");
    }
    else{
      printf("OK\n");
    }
    
    //printf("OK: %d\n", OK);
    OK = 0;
    //OSSemPost(OverloadDetectionSem);
  }
}

void ExtraLoadTask(void * pdtata){
  INT8U err;
  printf("Extraload created \n");
  int state;
  int delaytime = 0;
  int delayunit = EXTRALOAD_PERIOD / 50;
  float runtime = 0;
  alt_u64 t;
  float milliseconds = 0;
  int led = 0;
  while (1){
    state = switches_pressed();
    OSSemPend(ExtraLoadSem, 0, &err);
    led = state & 0x3F0;
    led_ON = led_ON | led;

    state = state >> 4;
    printf("state: %d\n", state);
    if (state > 50)
      state = 50;
    delaytime = delayunit * state;
    printf("delay time: %d\n", delaytime);
    // copied from weikai
    PERF_RESET(PERFORMANCE_COUNTER_BASE);
    t = perf_get_total_time((void*)PERFORMANCE_COUNTER_BASE);
    PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
    milliseconds = 1000 * (float)t / (float)alt_get_cpu_freq();
    while (milliseconds < delaytime){
      int i = 0;
      int j = 0;
      for (i = 0; i < 10; ++i) {
        j++;
      }
      t = perf_get_total_time((void*)PERFORMANCE_COUNTER_BASE);
      PERF_RESET(PERFORMANCE_COUNTER_BASE);
      PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
      milliseconds = milliseconds + (1000 * (float)t / (float)alt_get_cpu_freq());
      // printf("hahahahahahahahaahahah: %f\n", milliseconds);
    }
    PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
    
      PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);

    //until this
    printf("finished\n");
  } 
}

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  //Create Hardware Timer with a period of 'delay' 
  
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!n");
  }

  // Create and start Software Timer

  VehicleTmr = OSTmrCreate(0,
                          VEHICLE_PERIOD/HW_TIMER_PERIOD,
                          OS_TMR_OPT_PERIODIC,
                          VehicleTmrCallback,
                          (void *) 0,
                          "VehicleTmr",
                          &err);

  ControlTmr = OSTmrCreate(0,
                          CONTROL_PERIOD/HW_TIMER_PERIOD,
                          OS_TMR_OPT_PERIODIC,
                          ControlTmrCallback,
                          (void *) 0,
                          "ControlTmr",
                          &err);

  ButtonIOTmr = OSTmrCreate(0, 
                          BUTTONIO_PERIOD/HW_TIMER_PERIOD, 
                          OS_TMR_OPT_PERIODIC, 
                          ButtonIOCallback, 
                          (void *) 0, 
                          "ButtonIOTmr", 
                          &err);
  SwitchIOTmr = OSTmrCreate(0, 
                          SWITCHIO_PERIOD/HW_TIMER_PERIOD, 
                          OS_TMR_OPT_PERIODIC, 
                          SwitchIOCallback, 
                          (void *) 0, 
                          "SwitchIOTmr", 
                          &err);   

  OverloadDetectionTmr = OSTmrCreate(0,
                                    OVERLOAD_PERIOD/HW_TIMER_PERIOD,
                                    OS_TMR_OPT_PERIODIC,
                                    OverloadDetectionCallback,
                                    (void *) 0,
                                    "OverloadDetectionTmr",
                                    &err
                                    );

  WatchDogTmr = OSTmrCreate(0, 
                            WATCHDOG_PERIOD/HW_TIMER_PERIOD,
                            OS_TMR_OPT_PERIODIC,
                            WatchDogCallback,
                            (void *) 0,
                            "WatchDogTmr",
                            &err);

  ExtraLoadTmr = OSTmrCreate(0,
                            EXTRALOAD_PERIOD/HW_TIMER_PERIOD,
                            OS_TMR_OPT_PERIODIC,
                            ExtraLoadCallback,
                            (void *) 0,
                            "ExtraLoadTmr",
                            &err);

  OSTmrStart(VehicleTmr, &err);
  OSTmrStart(ControlTmr, &err);
  OSTmrStart(ButtonIOTmr, &err);
  OSTmrStart(SwitchIOTmr, &err);
  OSTmrStart(OverloadDetectionTmr, &err);
  OSTmrStart(WatchDogTmr, &err);
  OSTmrStart(ExtraLoadTmr, &err);                
  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 1); /* Empty Mailbox - Brake */
  Mbox_Engine = OSMboxCreate((void*) 1); /* Empty Mailbox - Engine */
  Mbox_Cruise_Control = OSMboxCreate((void*) 1);
  Mbox_Gas_Pedal = OSMboxCreate((void *) 1);
  Mbox_Gear = OSMboxCreate((void *) 1);
  Mbox_Cruise_Velocity = OSMboxCreate((void *) 0);
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

  err = OSTaskCreateExt(ButtonIOTask, 
      NULL, 
      &ButtonIOTask_Stack[TASK_STACKSIZE-1],
      BUTTONIO_PRIO,
      BUTTONIO_PRIO,
      (void *)&ButtonIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(SwitchIOTask, 
      NULL, 
      &SwitchIOTask_Stack[TASK_STACKSIZE-1],
      SWITCHIO_PRIO,
      SWITCHIO_PRIO,
      (void *)&SwitchIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(OverloadDetectionTask,
      NULL,
      &OverloadDetectionTask_Stack[TASK_STACKSIZE-1],
      OVERLOAD_PRIO,
      OVERLOAD_PRIO,
      (void *) &OverloadDetectionTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(WatchDogTask,
      NULL,
      &WatchDogTask_Stack[TASK_STACKSIZE-1],
      WATCHDOG_PRIO,
      WATCHDOG_PRIO,
      (void *) &WatchDogTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(ExtraLoadTask,
      NULL,
      &ExtraLoadTask_Stack[TASK_STACKSIZE-1],
      EXTRALOADTASK_PRIO,
      EXTRALOADTASK_PRIO,
      (void *) &ExtraLoadTask_Stack[0],
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
  VehicleTmrSem = OSSemCreate(1);
  ControlTmrSem = OSSemCreate(1);
  ButtonIOSem = OSSemCreate(1);
  SwitchIOSem = OSSemCreate(1);
  OverloadDetectionSem = OSSemCreate(1);
  WatchDogSem = OSSemCreate(1);
  ExtraLoadSem = OSSemCreate(1);

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
