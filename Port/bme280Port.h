/*
 * bme280Port.h
 *
 *  Created on: Nov 12, 2023
 *      Author: ludovic
 */

#ifndef BME280_PORT_H_
#define BME280_PORT_H_

#include "stm32l1xx.h"

#include "gpioDefs.h"

#define BME280_SPI_BUS  (void*)SPI1
#define BME280_CS_PIN   GPIOA,8

#define BME280_SPI_BAUDRATE_HZ  (10000000uL)


#endif
