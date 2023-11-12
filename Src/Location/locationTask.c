/*
 * locationTask.c
 *
 *  Created on: Nov 12, 2023
 *      Author: ludovic
 */
#include <stdint.h>

#include "mcuSdkGpio.h"
#include "gpioDefs.h"

#include "FreeRTOS.h"
#include "task.h"

/* Dimensions of the buffer that the task being created will use as its stack.
  NOTE:  This is the number of words the stack will hold, not the number of
  bytes.  For example, if each stack item is 32-bits, and this is set to 100,
  then 400 bytes (100 * 32-bits) will be allocated. */
#define LOCATION_STACK_SIZE 200

    /* Structure that will hold the TCB of the task being created. */
static StaticTask_t xLocationTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
  an array of StackType_t variables.  The size of StackType_t is dependent on
  the RTOS port. */
static StackType_t xLocationTaskStack[ LOCATION_STACK_SIZE ];

static TaskHandle_t xLocationTaskHandle = NULL;



static void locationTask( void * pvParameters );

/*
 *
 */
void locationInit(void)
{
  /* Create the task without using any dynamic memory allocation. */
  xLocationTaskHandle = xTaskCreateStatic(
                          locationTask,           /* Function that implements the task. */
                          "LocationTask",         /* Text name for the task. */
                          LOCATION_STACK_SIZE,    /* Number of indexes in the xStack array. */
                          NULL,                   /* Parameter passed into the task. */
                          tskIDLE_PRIORITY +1,    /* Priority at which the task is created. */
                          xLocationTaskStack,     /* Array to use as the task's stack. */
                          &xLocationTaskBuffer ); /* Variable to hold the task's data structure. */
}


/*
 *
 */
static void locationTask( void * pvParameters )
{
  mcuSdkGpioSetAsOutput(GPIO_DEFS_LED);
  for( ;; )
  {
    vTaskDelay(pdMS_TO_TICKS(500));
    mcuSdkGpioTogglePin(GPIO_DEFS_LED);
  }
}
