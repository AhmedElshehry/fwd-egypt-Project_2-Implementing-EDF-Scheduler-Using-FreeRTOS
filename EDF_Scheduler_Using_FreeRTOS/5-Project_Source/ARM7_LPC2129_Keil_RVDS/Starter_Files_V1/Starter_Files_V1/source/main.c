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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "semphr.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
SemaphoreHandle_t userInput;

TaskHandle_t LedTaskHandle = NULL;
TaskHandle_t ButtomTaskHandle = NULL;
TaskHandle_t Button_1TaskHandle = NULL;
TaskHandle_t Button_2TaskHandle = NULL;
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

pinState_t Buttom_stutes ;

/* Task 1 to be created. */
void Button_1_Monitor( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	pinState_t Button_1_State;
	pinState_t PrevState = GPIO_read(PORT_1 , PIN7);//read initial value the frite
  char Edge_Flag_tobe = 0;
	for( ;; )
	{
		/* Read GPIO Input pin value high or low  */
		Button_1_State = GPIO_read(PORT_1 , PIN7);
		
		/*Check for Edges stutes */
		if( Button_1_State == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			/*Positive Edge*/
			Edge_Flag_tobe = '+' ;// rising 
		}
		else if (Button_1_State == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
			/*Negative Edge*/
			Edge_Flag_tobe = '-';// falling
		}
		else
		{
			Edge_Flag_tobe = '.';// the  floating or dedection error change
		}
		/*save the ststue new that  value in Button_1_State in the old  PrevState */
		PrevState = Button_1_State;
		
		// sent the data by uart 
		    
		GPIO_write(PORT_0,PIN6,PIN_IS_LOW);
		/*Periodicity  of the task 1 equal 50*/
		vTaskDelayUntil( &xLastWakeTime , 50); 
		GPIO_write(PORT_0,PIN6,PIN_IS_HIGH);
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
		
	}
}

/* Task 1 to be created. */
void Button_2_Monitor( void * pvParameters )
{
	int xLastWakeTime = xTaskGetTickCount();
	pinState_t Button_2_State;
	pinState_t PrevState = GPIO_read(PORT_1 , PIN3);//read initial value the frite
  char Edge_Flag_tobe = 0;
	for( ;; )
	{
		/* Read GPIO Input pin value high or low  */
		Button_2_State = GPIO_read(PORT_1 , PIN3);
		
		/*Check for Edges stutes */
		if( Button_2_State == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			/*Positive Edge*/
			Edge_Flag_tobe = '+' ;// rising 
		}
		else if (Button_2_State == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
			/*Negative Edge*/
			Edge_Flag_tobe = '-';// falling
		}
		else
		{
			Edge_Flag_tobe = '.';// the  floating or dedection error change
		}
		/*save the ststue new that  value in Button_1_State in the old  PrevState */
		PrevState = Button_2_State;
		
		// sent the data by uart 
		      
			GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
		/*Periodicity  of the task 2 equal 50*/
		vTaskDelayUntil( &xLastWakeTime , 50);
		GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

///* Task to be created. */
//void Buttom_task( void * pvParameters )
//{
//int xLastWakeTime = xTaskGetTickCount();
//		for( ;; )
//		{
//				/* Task code goes here. */
//			//Buttom_stutes=GPIO_read(PORT_0,PIN0);
//			
//			int i;
//			GPIO_write(PORT_0,PIN4,PIN_IS_HIGH);
//			for( i=0;i<10000;i++)
//			{
//				
//			}	
//			GPIO_write(PORT_0,PIN4,PIN_IS_LOW);
//			GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
//			vTaskDelayUntil( &xLastWakeTime, 80 );
//			GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
//			
//			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
//		}
//}
///* Task to be created. */
//volatile int misseddeadline=0;
//void Led_task( void * pvParameters )
//{
//	
//	int time_start=0,time_end=0;
//int xLastWakeTime = xTaskGetTickCount();
//		for( ;; )
//		{
//				/* Task code goes here. */
///*			if(Buttom_stutes==PIN_IS_HIGH)
//			{
//				GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
//			}
//			else
//			{
//				GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
//			}*/
//			int u;
//			for(u=0;u<1000;u++)
//			{
//				
//			}	
//			time_end=xTaskGetTickCount();
//			if((time_end-time_start)>50){misseddeadline ++;}
//			GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
//			vTaskDelayUntil( &xLastWakeTime, 50 );
//			GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
//			time_start=xTaskGetTickCount();
//			GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
//		}
//}

void vApplicationIdleHook( void )
{
      GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);
}
void vApplicationTickHook( void )
{
      GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
userInput =xSemaphoreCreateBinary();
	
//    /* Create Tasks here */
//  /* Create the task, storing the handle. */
//   xTaskPeriodicCreate(
//                    Led_task,       /* Function that implements the task. */
//                    "Led task",          /* Text name for the task. */
//                    1000 ,      /* Stack size in words, not bytes. */
//                    ( void * ) 0,    /* Parameter passed into the task. */
//                    1,/* Priority at which the task is created. */
//                    &LedTaskHandle,50 );      /* Used to pass out the created task's handle. */
//										
//	 /* Create the task, storing the handle. */
//   xTaskPeriodicCreate(
//                    Buttom_task,       /* Function that implements the task. */
//                    "Buttom task",          /* Text name for the task. */
//                     100 ,      /* Stack size in words, not bytes. */
//                    ( void * ) 0,    /* Parameter passed into the task. */
//                    3,/* Priority at which the task is created. */
//                    &ButtomTaskHandle,80 );      /* Used to pass out the created task's handle. */									


/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1",          /* Text name for the task. */
                     500 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                    &Button_1TaskHandle,50 );      /* Used to pass out the created task's handle. */					

										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2",          /* Text name for the task. */
                     500 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    3,/* Priority at which the task is created. */
                    &Button_1TaskHandle,50 );      /* Used to pass out the created task's handle. */		
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


