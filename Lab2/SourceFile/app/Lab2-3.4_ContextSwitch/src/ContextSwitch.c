// File: TwoTasks.c 

#include <stdio.h>
#include "includes.h"
#include <string.h>
#include "altera_avalon_performance_counter.h"
#include "system.h"

#define DEBUG 1

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];
OS_STK    stat_stk[TASK_STACKSIZE];

OS_EVENT * Sem1;
OS_EVENT * Sem2;
OS_EVENT * Sem3;

/* Definition of Task Priorities */
#define TASK1_PRIORITY      6  // highest priority
#define TASK2_PRIORITY      7
#define TASK_STAT_PRIORITY 12  // lowest priority 

void printStackSize(char* name, INT8U prio) 
{
  INT8U err;
  OS_STK_DATA stk_data;
    
  err = OSTaskStkChk(prio, &stk_data);
  if (err == OS_NO_ERR) {
    if (DEBUG == 1)
      printf("%s (priority %d) - Used: %d; Free: %d\n", 
	     name, prio, stk_data.OSUsed, stk_data.OSFree);
  }
  else
    {
      if (DEBUG == 1)
	printf("Stack Check Error!\n");    
    }
}

/* Prints a message and sleeps for given time interval */
void task1(void* pdata)
{
  INT8U err;
  int Task0State = 0;
  while(1)
    { 
      OSSemPend(Sem1, 0, &err);
      printf("Task 0 - State %d\n", Task0State);

      if (Task0State == 0) {
        Task0State++;
        PERF_RESET(PERFORMANCE_COUNTER_BASE);
        PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
        OSSemPost(Sem2);
      }
      else {
        Task0State--;
        OSSemPost(Sem1);
      }
      OSTimeDlyHMSM(0, 0, 0, 11); 
          /* Context Switch to next task
				   * Task will go to the ready state
				   * after the specified delay
				   */
    }
}

/* Prints a message and sleeps for given time interval */
void task2(void* pdata)
{
  INT8U err;
  int Task1State = 0;
  alt_u64 t;
  int i = 0;
  float milliseconds = 0;
  float totaltime = 0;
  float averagetime = 0;

  while(1)
    { 
      OSSemPend(Sem2, 0, &err);


      PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
      t = perf_get_total_time((void*)PERFORMANCE_COUNTER_BASE);
      milliseconds = 1000 * (float)t / (float)alt_get_cpu_freq();
      if (i < 100) {
        if (0.9 < milliseconds && milliseconds < 1.3) {
          i++;
          totaltime += milliseconds;
        }
      }
      else {
        averagetime = totaltime / 100;
        printf("Average Time: %f\n", averagetime);
      }


      printf("Task 1 - State %d\n", Task1State);
      if (Task1State == 0) {
        Task1State++;
        OSSemPost(Sem2);
      }
      else {
        Task1State--;
        OSSemPost(Sem1);
      }
      OSTimeDlyHMSM(0, 0, 0, 4);
    }
}

/* Printing Statistics */
void statisticTask(void* pdata)
{
  INT8U err;
  while(1)
    {
      OSSemPend(Sem3, 0, &err);
      printStackSize("Task1", TASK1_PRIORITY);
      printStackSize("Task2", TASK2_PRIORITY);
      printStackSize("StatisticTask", TASK_STAT_PRIORITY);
    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
  printf("Lab 3 - Two Tasksddfdytrertertressetrestd5yiootdddddd\n");

  Sem1 = OSSemCreate(1);
  Sem2 = OSSemCreate(0);
  Sem3 = OSSemCreate(0);


  OSTaskCreateExt
    ( task1,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task1_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK1_PRIORITY,               // Desired Task priority
      TASK1_PRIORITY,               // Task ID
      &task1_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
      );
	   
  OSTaskCreateExt
    ( task2,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task2_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK2_PRIORITY,               // Desired Task priority
      TASK2_PRIORITY,               // Task ID
      &task2_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                       
      );  

  if (DEBUG == 1)
    {
      OSTaskCreateExt
	( statisticTask,                // Pointer to task code
	  NULL,                         // Pointer to argument passed to task
	  &stat_stk[TASK_STACKSIZE-1],  // Pointer to top of task stack
	  TASK_STAT_PRIORITY,           // Desired Task priority
	  TASK_STAT_PRIORITY,           // Task ID
	  &stat_stk[0],                 // Pointer to bottom of task stack
	  TASK_STACKSIZE,               // Stacksize
	  NULL,                         // Pointer to user supplied memory (not needed)
	  OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
	  OS_TASK_OPT_STK_CLR           // Stack Cleared                              
	  );
    }  

  OSStart();
  return 0;
}
