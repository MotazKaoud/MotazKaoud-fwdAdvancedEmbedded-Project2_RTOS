/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "event_groups.h"
#include "queue.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

#define B1RE	( 1 << 0 )
#define B1FE	( 2 << 0 )
#define B2RE	( 3 << 0 )
#define B2FE	( 4 << 0 )

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

pinState_t buttonstate;

TaskHandle_t task1handler = NULL;
TaskHandle_t task2handler = NULL;
TaskHandle_t task3handler = NULL;
TaskHandle_t task4handler = NULL;
TaskHandle_t task5handler = NULL;
TaskHandle_t task6handler = NULL;

pinState_t B1_Current_State;
pinState_t B1_Old_State ;
pinState_t B2_Current_State;
pinState_t B2_Old_State;

char PeriodicString[4] = {"RTOS"};
char ReceivedFromQueue [20] ;
char LineSend[4] ;



static	int i ;
static 	EventBits_t uxBits;


/* Declare a variable to hold the created event group. */
EventGroupHandle_t ButtonsEventGroup;
uint32_t EventGroupVar;
/* Declare a variable to hold the created Queue. */
QueueHandle_t xQueue1;

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/


/*------------------ Tasks ----------------------------------*/

/* task1 "Button_1_Monitor" ISR, Monitor (PORT_0-PIN0-INPUT) pin and send its state to consumer task*/
void Task_1( void * pvParameters )  
	{ for (;;) {
	// Create the xLastWakeTime variable and Initialise it with the current time for using delayuntill fn
	 TickType_t xLastWakeTime;     
   xLastWakeTime = xTaskGetTickCount();
	/*Monitor Button_1 State*/
	B1_Current_State = GPIO_read(PORT_0,PIN7);
	if 			(B1_Current_State <  B1_Old_State) // Failing Edge Detected, Set B1FE Flag and Clear B1RE Flag in ButtonsEventGroup
	{
  uxBits = xEventGroupSetBits  ( ButtonsEventGroup,   	/* The event group being updated. */
                                 B1FE );							 	/* The bits being set. */
  uxBits = xEventGroupClearBits( ButtonsEventGroup,  		/* The event group being updated. */
                                 B1RE );								/* The bits being cleared. */
  }
	else if (B1_Current_State >  B1_Old_State) // Raising Edge Detected, Set B1RE Flag and Clear B1FE Flag in ButtonsEventGroup
	{
  uxBits = xEventGroupSetBits  ( ButtonsEventGroup,   	/* The event group being updated. */
                                 B1RE );							 	/* The bits being set. */
  uxBits = xEventGroupClearBits( ButtonsEventGroup,  		/* The event group being updated. */
                                 B1FE );								/* The bits being cleared. */	 
	}
	else if (B1_Current_State == B1_Old_State) // No Edge Detected, Clear all flags (B1FE/B1RE)
	{
	  uxBits = xEventGroupClearBits( ButtonsEventGroup,  		/* The event group being updated. */
                                   B1FE | B1RE );					/* The bits being cleared. */	 
	}
	/*Send State to Consumer Task*/
	EventGroupVar = (uint32_t) (xEventGroupGetBits(ButtonsEventGroup)) ; 
  xQueueSend( xQueue1, ( void * ) &EventGroupVar,( TickType_t ) 10 );
	/*Update Variables*/
	B1_Old_State = B1_Current_State ;

GPIO_write(PORT_0,PIN1, PIN_IS_LOW);
	/*Block Task 50ticks (50ms)*/		
	vTaskDelayUntil( &xLastWakeTime, (TickType_t)50 );
GPIO_write(PORT_0,PIN1, PIN_IS_HIGH);
}
}

/* task2 "Button_2_Monitor" ISR, Monitor (PORT_0-PIN1-INPUT) pin and send its state to consumer task*/
void Task_2( void * pvParameters )  
{
	for(;;) {
	// Create the xLastWakeTime variable and Initialise it with the current time for using delayuntill fn
	 TickType_t xLastWakeTime;     
   xLastWakeTime = xTaskGetTickCount();
	/*Monitor Button_1 State*/
	B2_Current_State = GPIO_read(PORT_0,PIN8);
	if 			(B2_Current_State <  B2_Old_State) // Failing Edge Detected, Set B2FE Flag and Clear B2RE Flag in ButtonsEventGroup
	{
  uxBits = xEventGroupSetBits  ( ButtonsEventGroup,   	/* The event group being updated. */
                                 B2FE );							 	/* The bits being set. */
  uxBits = xEventGroupClearBits( ButtonsEventGroup,  		/* The event group being updated. */
                                 B2RE );								/* The bits being cleared. */
  }
	else if (B2_Current_State >  B2_Old_State) // Raising Edge Detected, Set B2RE Flag and Clear B2FE Flag in ButtonsEventGroup
	{
  uxBits = xEventGroupSetBits  ( ButtonsEventGroup,   	/* The event group being updated. */
                                 B2RE );							 	/* The bits being set. */
  uxBits = xEventGroupClearBits( ButtonsEventGroup,  		/* The event group being updated. */
                                 B2FE );								/* The bits being cleared. */	 
	}
	else if (B2_Current_State == B2_Old_State) // No Edge Detected, Clear all flags (B2FE/B2RE)
	{
	  uxBits = xEventGroupClearBits( ButtonsEventGroup,  		/* The event group being updated. */
                                   B2FE | B2RE );					/* The bits being cleared. */	 
	}
	/*Send State to Consumer Task*/
	EventGroupVar = (uint32_t) (xEventGroupGetBits(ButtonsEventGroup)) ; 
  xQueueSend( xQueue1, ( void * ) &EventGroupVar,( TickType_t ) 10 );
	/*Update Variables*/
	B2_Old_State = B2_Current_State ;
	
GPIO_write(PORT_0,PIN1, PIN_IS_LOW);
	/*Block Task 50ticks (50ms)*/		
	vTaskDelayUntil( &xLastWakeTime, (TickType_t)50 );
GPIO_write(PORT_0,PIN1, PIN_IS_HIGH);
}
}

/* task3 "Periodic_Transimitter" ISR,send periodic string every 100 ms to consumer task*/
void Task_3( void * pvParameters )  
{
	for (;;) {
	// Create the xLastWakeTime variable and Initialise it with the current time for using delayuntill fn
	 TickType_t xLastWakeTime;     
   xLastWakeTime = xTaskGetTickCount();
	
	//send the PeriodicString to the Queue
  xQueueSend( xQueue1, ( void * ) &PeriodicString,( TickType_t ) 10 );	
		
GPIO_write(PORT_0,PIN1, PIN_IS_LOW);
			/*Block Task 100ticks (100ms)*/	
	 vTaskDelayUntil( &xLastWakeTime, (TickType_t)100 );
GPIO_write(PORT_0,PIN1, PIN_IS_HIGH);	
}
}

/* Task_4 "Uart_Receiver", write on uart received string from other tasks. */
void Task_4( void * pvParameters )   
{ 
	for (;;) {
		// Create the xLastWakeTime variable and Initialise it with the current time for using delayuntill fn
			 TickType_t xLastWakeTime;     
       xLastWakeTime = xTaskGetTickCount();
/*
		// Load the Queue, Temporary for Measuring Task Load		
  		 xQueueSend( xQueue1, ( void * ) &PeriodicString,( TickType_t ) 10 );	// PeriodicString = 4Bytes, xQueue1 = 20Bytes
			 xQueueSend( xQueue1, ( void * ) &PeriodicString,( TickType_t ) 10 );	
			 xQueueSend( xQueue1, ( void * ) &PeriodicString,( TickType_t ) 10 );	
			 xQueueSend( xQueue1, ( void * ) &PeriodicString,( TickType_t ) 10 ); // xQueue1 Loaded with 16Bytes
*/		
GPIO_write(PORT_0,PIN1, PIN_IS_HIGH);;	// Start Calculating task actual execution time
	if( xQueue1 != NULL ) // if the Queue contains data
	{
	xSerialPutChar('\n');
	xQueueReceive( xQueue1, &(ReceivedFromQueue), ( TickType_t ) 10 ) ; 
	ReceivedFromQueue[19] += 0x01;
	vSerialPutString(ReceivedFromQueue,20);	
	}		
GPIO_write(PORT_0,PIN1, PIN_IS_LOW);
					/*Block Task 20ticks (20ms)*/	
	vTaskDelayUntil( &xLastWakeTime, (TickType_t)20 );
}
}

/* Load_1_Simulation Task */
void Task_5( void * pvParameters ) 
{ for (;;) {
		// Create the xLastWakeTime variable and Initialise it with the current time for using delayuntill fn
	 TickType_t xLastWakeTime;     
   xLastWakeTime = xTaskGetTickCount();
		for (i = 0 ; i < 9500 ; i++)
			{i=i;}
GPIO_write(PORT_0,PIN1, PIN_IS_LOW);
	/*Block Task 10ticks (10ms)*/		
	vTaskDelayUntil( &xLastWakeTime, (TickType_t)10 );
GPIO_write(PORT_0,PIN1, PIN_IS_HIGH);
}
}

/* Load_2_Simulation Task */
void Task_6( void * pvParameters ) 
{ for (;;) {
		// Create the xLastWakeTime variable and Initialise it with the current time for using delayuntill fn
	 TickType_t xLastWakeTime;     
   xLastWakeTime = xTaskGetTickCount();
	for (i = 0 ; i < 23000 ; i++)
		{i=i;}
GPIO_write(PORT_0,PIN1, PIN_IS_LOW);
	/*Block Task 100ticks (100ms)*/		
	vTaskDelayUntil( &xLastWakeTime, (TickType_t)100 );
GPIO_write(PORT_0,PIN1, PIN_IS_HIGH);
}
/*-----------------------------------------------------------*/
}
void vApplicationTickHook( void )
{
 GPIO_write(PORT_0,PIN0, PIN_IS_HIGH);
 GPIO_write(PORT_0,PIN0, PIN_IS_LOW);
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
    /************************ Create The Tasks *************************/
	
 xTaskCreate(    Task_1,       /* Function that implements the task. */
									"Button_1_Monitor",          /* Text name for the task. */
									(100),      /* Stack size in words, not bytes. */
									( void * ) 0,    /* Parameter passed into the task. */
									(1),/* Priority at which the task is created. */
									&task1handler );	
	xTaskCreate(    Task_2,       /* Function that implements the task. */
									"Button_2_Monitor",          /* Text name for the task. */
									(100),      /* Stack size in words, not bytes. */
									( void * ) 0,    /* Parameter passed into the task. */
									(1),/* Priority at which the task is created. */
									&task2handler );
	xTaskCreate(    Task_3,       /* Function that implements the task. */
									"Periodic_Transmitter",          /* Text name for the task. */
									(100),      /* Stack size in words, not bytes. */
									( void * ) 0,    /* Parameter passed into the task. */
									(1),/* Priority at which the task is created. */
									&task3handler );
	xTaskCreate(    Task_4,       /* Function that implements the task. */
									"Uart_Receiver",          /* Text name for the task. */
									(100),      /* Stack size in words, not bytes. */
									( void * ) 0,    /* Parameter passed into the task. */
									(1),/* Priority at which the task is created. */
									&task4handler );
	xTaskCreate(    Task_5,       /* Function that implements the task. */
									"Load_1_Simulation",          /* Text name for the task. */
									(100),      /* Stack size in words, not bytes. */
									( void * ) 0,    /* Parameter passed into the task. */
									(1),/* Priority at which the task is created. */
									&task5handler );
	xTaskCreate(    Task_6,       /* Function that implements the task. */
									"Load_2_Simulation",          /* Text name for the task. */
									(100),      /* Stack size in words, not bytes. */
									( void * ) 0,    /* Parameter passed into the task. */
									(1),/* Priority at which the task is created. */
									&task6handler );									
		
    /* Attempt to create the event group. */
    ButtonsEventGroup = xEventGroupCreate();
	  
		/* Create a queue capable of containing 100 uint32_t locations (4 bytes) to hold adress if required . */
    xQueue1 = xQueueCreate( 100, 4);		

		/*Test Uart#2 Serial Monitor*/
		xSerialPutChar('t');xSerialPutChar('e');xSerialPutChar('s');xSerialPutChar('t');
		xSerialPutChar('\n');;

		/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


