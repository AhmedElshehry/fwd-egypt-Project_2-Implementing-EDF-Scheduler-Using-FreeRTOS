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
#include "queue.h"

/*the macro of period of all tasks*/
#define Button_1_PERIOD         50
#define Button_2_PERIOD         50
#define TX_PERIOD               100
#define RX_PERIOD               20
#define LOAD_1_PERIOD           10
#define LOAD_2_PERIOD           100

/*the some macro need  */
#define BUTTON1_RISING    1
#define BUTTON1_FAllING   2
#define BUTTON2_RISING    3
#define BUTTON2_FAllING   4
#define PERIODIC_STRING   5
#define BUTTON_NOT_FAllING_RISING 0
/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )
	
/*the handler of the task in the system */

TaskHandle_t Button_1TaskHandle = NULL;
TaskHandle_t Button_2TaskHandle = NULL;
TaskHandle_t Transmitter_TaskHandle = NULL;
TaskHandle_t Uart_TaskHandle = NULL;
TaskHandle_t L1TaskHandle = NULL;
TaskHandle_t L2TaskHandle = NULL;
QueueHandle_t xQueueButton_1=NULL,xQueueButton_2=NULL ,xQueuerTransmiter=NULL;
/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
/* HOOK task */
void vApplicationIdleHook( void );
void vApplicationTickHook( void );

/*  the Task 1 to check the rising and falling edge */
void Button_1_Monitor( void * pvParameters );

/*  the Task 2 to check the rising and falling edge */
void Button_2_Monitor( void * pvParameters );

/* the Task 3 transimtion periodic string  */
void Task_Transmitter( void * pvParameters );

/* the Task 4 Uart Receiver  */
void Uart_Receiver( void * pvParameters );

/* the Task 5 load 1 5ms  */
void Load_1_Simulation( void * pvParameters );

/* the Task 6 load 2 12ms  */
void Load_2_Simulation( void * pvParameters );

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	/* Create a queue capable of containing 10 unsigned long values. */
	xQueueButton_1    = xQueueCreate( 1,sizeof(char*) );
	xQueueButton_2    = xQueueCreate( 1,sizeof(char*) );	
	xQueuerTransmiter = xQueueCreate( 1,sizeof(char*) );

/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_1TaskHandle,
										Button_1_PERIOD/*period task */ );      /* Used to pass out the created task's handle. */					
			
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_2TaskHandle,
										Button_2_PERIOD /*period task */);      /* Used to pass out the created task's handle. */											
											
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Task_Transmitter,       /* Function that implements the task. */
                    "Transmitter",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Transmitter_TaskHandle,
										TX_PERIOD /*period task */  );      /* Used to pass out the created task's handle. */		
										
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "uart",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Uart_TaskHandle,
										RX_PERIOD /*period task */  );      /* Used to pass out the created task's handle. */		
										
                    /* Create the task, storing the handle. */
  xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load1",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &L1TaskHandle,
										LOAD_1_PERIOD /*period task */);      /* Used to pass out the created task's handle. */	
										
										/* Create the task, storing the handle. */
   xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load2",          /* Text name for the task. */
                     100 ,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &L2TaskHandle,	
										LOAD_2_PERIOD /*period task */ );      /* Used to pass out the created task's handle. */			
											
										
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

void vApplicationIdleHook( void )
{
      GPIO_write(PORT_0,PIN0,PIN_IS_HIGH);
}
void vApplicationTickHook( void )
{
      GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
}


/*  Implement of the Task 1 to check the rising and falling edge */
void Button_1_Monitor( void * pvParameters )
{
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	pinState_t Button_1_State;
	pinState_t PrevState = GPIO_read(PORT_1 , PIN0);//read initial value the frite
  volatile char Edge_Flag_tobe = 0;
	/* This task is going to be represented by a voltage scale of 1 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	for( ;; )
	{
		/* Read GPIO Input pin value high or low  */
		Button_1_State = GPIO_read(PORT_1 , PIN0);
		
		/*Check for Edges stutes */
		if (Button_1_State == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
			/*Negative Edge*/
			Edge_Flag_tobe = BUTTON1_FAllING;// falling
			 /* Modify for Communication Queue */
			/* sent the event in the queues */
       if(xQueueButton_1 != 0)
          {
            xQueueSend( xQueueButton_1, ( void * )&Edge_Flag_tobe, (TickType_t) 0 );
          }
		}
		else if( Button_1_State == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			/*Positive Edge*/
			Edge_Flag_tobe = BUTTON1_RISING;// rising 
			 /* Modify for Communication Queue */
			/* sent the event in the queues */
       if(xQueueButton_1 != 0)
          {
            xQueueSend( xQueueButton_1, ( void * )&Edge_Flag_tobe, (TickType_t) 0 );
          }
		}
		else
		{
			Edge_Flag_tobe = BUTTON_NOT_FAllING_RISING;// the  floating or dedection error change
			 /* Modify for Communication Queue */
			/* sent the event in the queues */
       if(xQueueButton_1 != 0)
          {
            xQueueSend( xQueueButton_1, ( void * )&Edge_Flag_tobe, (TickType_t) 0 );
          }
		}
		/*save the ststue new that  value in Button_1_State in the old  PrevState */
		PrevState = Button_1_State;
		
		/*Periodicity  of the task 1 equal 50*/
		vTaskDelayUntil( &xLastWakeTime , Button_1_PERIOD); 
		
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/*  Implement of the Task 2 to check the rising and falling edge */
void Button_2_Monitor( void * pvParameters )
{
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	pinState_t Button_2_State;
	pinState_t PrevState = GPIO_read(PORT_1 , PIN1);//read initial value the frite
  volatile char Edge_Flag_tobe = 0;
	/* This task is going to be represented by a voltage scale of 2 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	for( ;; )
	{
		/* Read GPIO Input pin value high or low  */
		Button_2_State = GPIO_read(PORT_1 , PIN1);
		
		/*Check for Edges stutes */
		if (Button_2_State == PIN_IS_LOW && PrevState == PIN_IS_HIGH)
		{
			/*Negative Edge*/
			Edge_Flag_tobe = BUTTON2_FAllING;// falling
			 /* Modify for Communication Queue */
			/* sent the event in the queues */
       if(xQueueButton_2 != 0)
          {
            xQueueSend( xQueueButton_2, ( void * )&Edge_Flag_tobe, (TickType_t) 0 );
          }
		}
		else if( Button_2_State == PIN_IS_HIGH && PrevState == PIN_IS_LOW)
		{
			/*Positive Edge*/
			Edge_Flag_tobe = BUTTON2_RISING;// rising 
			 /* Modify for Communication Queue */
			/* sent the event in the queues */
       if(xQueueButton_2 != 0)
          {
            xQueueSend( xQueueButton_2, ( void * )&Edge_Flag_tobe, (TickType_t) 0 );
          }
		}
		else
		{
			Edge_Flag_tobe = BUTTON_NOT_FAllING_RISING;// the  floating or dedection error change
			 /* Modify for Communication Queue */
			/* sent the event in the queues */
       if(xQueueButton_2 != 0)
          {
            xQueueSend( xQueueButton_2, ( void * )&Edge_Flag_tobe, (TickType_t) 0 );
          }
		}
		/*save the ststue new that  value in Button_1_State in the old  PrevState */
		PrevState = Button_2_State;

		/*Periodicity  of the task 1 equal 50*/
		vTaskDelayUntil( &xLastWakeTime , Button_2_PERIOD); 
		
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/*  Implement of the Task 3 transimtion periodic string  */
void Task_Transmitter( void * pvParameters )
{
	volatile char	Periodic_String =0;
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* This task is going to be represented by a voltage scale of 3 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	for( ;; )
	{
		Periodic_String=PERIODIC_STRING;
		  /* Modify for Communication Queue */
			/* sent the event in the queues periodic task  */
       if(xQueuerTransmiter != 0)
          {
            xQueueSend( xQueuerTransmiter, ( void * )&Periodic_String, (TickType_t) 0 );
          }
	

					
		/*Periodicity  of the task 3 equal 100*/
		vTaskDelayUntil( &xLastWakeTime , TX_PERIOD);

		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}
/*  Implement of the Task 4 to check the rising and falling edge */
void Uart_Receiver( void * pvParameters )
{		
	char* Button1_ON_String ="Button1 ON\n";
	char* Button1_OFF_String="Button1 OFF\n";
	char* Button2_ON_String ="Button2 ON\n";
	char* Button2_OFF_String="Button2 OFF\n";
	char* Periodic_String ="Periodic String\n";
	char receiverBuffer = 0;
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	/* This task is going to be represented by a voltage scale of 4 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
  for(;;)
  {
        /* Modify for Communication Queue */
        if(xQueueButton_1!=NULL)
        {
            if ((xQueueReceive(xQueueButton_1,(void *) &(receiverBuffer), (TickType_t)0))==pdPASS )
            {
               // xSerialPutChar('\n');
                if(receiverBuffer == BUTTON1_RISING) vSerialPutString((const signed char * const)Button1_ON_String, 11);
                else if (receiverBuffer == BUTTON1_FAllING) vSerialPutString((const signed char * const)Button1_OFF_String, 12);

            }
				}	
				 /* Modify for Communication Queue */
        if(xQueueButton_2!=NULL)
        {
            if ((xQueueReceive(xQueueButton_2,(void *) &(receiverBuffer), (TickType_t)0))==pdPASS )
            {
               // xSerialPutChar('\n');
                if(receiverBuffer == BUTTON2_RISING) vSerialPutString((const signed char * const)Button2_ON_String, 11);
                else if (receiverBuffer == BUTTON2_FAllING) vSerialPutString((const signed char * const)Button2_OFF_String, 12);

            }
				}
				 /* Modify for Communication Queue */
        if(xQueuerTransmiter!=NULL)
        {
            if ((xQueueReceive(xQueuerTransmiter,(void *) &(receiverBuffer), (TickType_t)0))==pdPASS )
            {
               // xSerialPutChar('\n');
                if (receiverBuffer == PERIODIC_STRING) vSerialPutString((const signed char * const)Periodic_String, 16);

            }
				}					
		
/*        if(xQueue_COMM!=NULL) //that two mathoud way of code 
//        {
//            if ((xQueueReceive(xQueue_COMM,(void *) &(receiverBuffer), (TickType_t)0))==pdPASS )
//            {
//                xSerialPutChar('\n');
//							  if(receiverBuffer == BUTTON1_RISING) vSerialPutString((const signed char * const)Button1_ON_String, 12);
//                else if (receiverBuffer == BUTTON1_FAllING) vSerialPutString((const signed char * const)Button1_OFF_String, 13);
//                else if(receiverBuffer == BUTTON2_RISING) vSerialPutString((const signed char * const)Button2_ON_String, 12);
//                else if (receiverBuffer == BUTTON2_FAllING) vSerialPutString((const signed char * const)Button2_OFF_String, 13);
//                else if (receiverBuffer == PERIODIC_STRING) vSerialPutString((const signed char * const)Periodic_String, 17);
//            }
				}*/
		
		

		/*Periodicity  of the task 4 equal 20*/
		vTaskDelayUntil( &xLastWakeTime , RX_PERIOD); 
				

		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/* the Task 5 load 1 5ms  */
void Load_1_Simulation ( void * pvParameters )
{
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t i=0, x = 12000*5;  /* (XTAL / 1000U)*time_in_ms  */
	/* This task is going to be represented by a voltage scale of 5 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );	
	for( ; ; )
	{		
		for( i=0 ; i <= x; i++){} /*5 ms delay*/
	
		/*Periodicity  of the task 5 equal 10*/
		vTaskDelayUntil( &xLastWakeTime , LOAD_1_PERIOD);
	
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}

/* the Task 6 load 2 12ms  */
void Load_2_Simulation ( void * pvParameters )
{
	volatile TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t i=0, x = 12000*12;  /* (XTAL / 1000U)*time_in_ms  */
	 /* This task is going to be represented by a voltage scale of 6 . */
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for( ; ; )
	{		
		for( i=0 ; i <= x; i++){} /*12 ms delay*/
			
		/*Periodicity  of the task 6 equal 100*/
		vTaskDelayUntil( &xLastWakeTime , LOAD_2_PERIOD);
			
		/*pin indection of idel task*/
		GPIO_write(PORT_0,PIN0,PIN_IS_LOW);
	}
}
