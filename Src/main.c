/*
 * main.c
 *
 *  Created on: Nov 3, 2023
 *      Author: ludovic
 */
#include "FreeRTOS.h"
#include "task.h"

#include "mcuSdkGpio.h"
#include "mcuSdkSystem.h"

#include "locationTask.h"




int main(void)
{

  /*
   * Basic and common hardware initialize
   * GPIO
   * CLOCK
   */
  mcuSdkSystemInit();
  mcuSdkGpioInit();

  /*
   * Create Tasks
   */
  locationInit();



  /*
   * Start FreeRTOS
   */
  vTaskStartScheduler();

  // System should never quit
  mcuSdkSystemReset();


}


