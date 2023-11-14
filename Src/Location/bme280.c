/*
 * locationTask.c
 *
 *  Created on: Nov 12, 2023
 *      Author: ludovic
 */

#include <stdint.h>

#include "mcuSdkGpio.h"
#include "mcuSdkSpi.h"

#include "bme280Port.h"
#include "bme280.h"


#define BME280_REGISTER_CALIB_00    0x88u
#define BME280_REGISTER_CALIB_01    0x89u
#define BME280_REGISTER_CALIB_02    0x8Au
#define BME280_REGISTER_CALIB_03    0x8Bu
#define BME280_REGISTER_CALIB_04    0x8Cu
#define BME280_REGISTER_CALIB_05    0x8Du
#define BME280_REGISTER_CALIB_06    0x8Eu
#define BME280_REGISTER_CALIB_07    0x8Fu
#define BME280_REGISTER_CALIB_08    0x90u
#define BME280_REGISTER_CALIB_09    0x91u
#define BME280_REGISTER_CALIB_10    0x92u
#define BME280_REGISTER_CALIB_11    0x93u
#define BME280_REGISTER_CALIB_12    0x94u
#define BME280_REGISTER_CALIB_13    0x95u
#define BME280_REGISTER_CALIB_14    0x96u
#define BME280_REGISTER_CALIB_15    0x97u
#define BME280_REGISTER_CALIB_16    0x98u
#define BME280_REGISTER_CALIB_17    0x99u
#define BME280_REGISTER_CALIB_18    0x9Au
#define BME280_REGISTER_CALIB_19    0x9Bu
#define BME280_REGISTER_CALIB_20    0x9Cu
#define BME280_REGISTER_CALIB_21    0x9Du
#define BME280_REGISTER_CALIB_22    0x9Eu
#define BME280_REGISTER_CALIB_23    0x9Fu
#define BME280_REGISTER_CALIB_24    0xA0u
#define BME280_REGISTER_CALIB_25    0xA1u

#define BME280_REGISTER_CHIP_ID     0xD0u

#define BME280_REGISTER_RESET       0xE0u

#define BME280_REGISTER_CALIB_26    0xE1u
#define BME280_REGISTER_CALIB_27    0xE2u
#define BME280_REGISTER_CALIB_28    0xE3u
#define BME280_REGISTER_CALIB_29    0xE4u
#define BME280_REGISTER_CALIB_30    0xE5u
#define BME280_REGISTER_CALIB_31    0xE6u
#define BME280_REGISTER_CALIB_32    0xE7u
#define BME280_REGISTER_CALIB_33    0xE8u
#define BME280_REGISTER_CALIB_34    0xE9u
#define BME280_REGISTER_CALIB_35    0xEAu
#define BME280_REGISTER_CALIB_36    0xEBu
#define BME280_REGISTER_CALIB_37    0xECu
#define BME280_REGISTER_CALIB_38    0xEDu
#define BME280_REGISTER_CALIB_39    0xEEu
#define BME280_REGISTER_CALIB_40    0xEFu
#define BME280_REGISTER_CALIB_41    0xF0u

#define BME280_REGISTER_CTRL_HUM    0xF2u
#define BME280_REGISTER_STATUS      0xF3u
#define BME280_REGISTER_CTRL_MEAS   0xF4u
#define BME280_REGISTER_CONFIG      0xF5u
#define BME280_REGISTER_PRESS_MSB   0xF7u
#define BME280_REGISTER_PRESS_LSB   0xF8u
#define BME280_REGISTER_PRESS_XLSB  0xF9u
#define BME280_REGISTER_TEMP_MSB    0xFAu
#define BME280_REGISTER_TEMP_LSB    0xFBu
#define BME280_REGISTER_TEMP_XLSB   0xFCu
#define BME280_REGISTER_HUM_MSB     0xFDu
#define BME280_REGISTER_HUM_LSB     0xFEu

#define BME280_REGISTER_WRITE_MSK   0x7Fu



#define BME280_TRIM_T_BASE          BME280_REGISTER_CALIB_00
#define BME280_TRIM_T_SIZE          6u
#define BME280_TRIM_P_BASE          BME280_REGISTER_CALIB_06
#define BME280_TRIM_P_SIZE          18u
#define BME280_TRIM_H1_BASE         BME280_REGISTER_CALIB_25
#define BME280_TRIM_H1_SIZE         1u
#define BME280_TRIM_H2_BASE         BME280_REGISTER_CALIB_26
#define BME280_TRIM_H2_SIZE         7u



#define BME280_CTRL_MEAS_MODE_MASK    0x03u
#define BME280_CTRL_MEAS_MODE_SLEEP   0x00u
#define BME280_CTRL_MEAS_MODE_NORMAL  0x01u
#define BME280_CTRL_MEAS_MODE_FORCED  0x02u

#define BME280_CHIP_ID_VALUE        0x60u

typedef enum
{
  SLEEP = 0u,
  FORCED,
  NORMAL
}ChipMode_e;

typedef struct __packed
{
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  uint8_t  dig_H1;
  uint16_t dig_H2;
  uint8_t  dig_H3;
  uint16_t dig_H4;
  uint16_t dig_H5;
}CalibRegisters;


static CalibRegisters TrimmingParameter;

static uint8_t ReadChipId(void);
static void ChangeMode(ChipMode_e nextMode);

static uint8_t ReadOneRegister( uint8_t registerName );
static void WriteOneRegister( uint8_t registerName, uint8_t value );
static void ReadBurstRegisters( uint8_t startAddress, uint8_t valueList[], uint8_t number);
static void WriteBurstRegisters( uint8_t startAddress, uint8_t valueList[], uint8_t number);

/*
 *
 */
void bme280Init(void)
{
  // Init SPI

  static const McuSdkSpiConfig_s bme280SpiConfig =
  {
      .IsLsdFirst = false,
      .baudrate = BME280_SPI_BAUDRATE_HZ,
      .SpiMode = MCU_SDK_SPI_MODE0,
  };

  // Set CS line high as output
  mcuSdkGpioSetPinHigh( BME280_CS_PIN );
  mcuSdkGpioSetAsOutput( BME280_CS_PIN );

  // Initialize spi bus
  mcuSdkSpiInit( BME280_SPI_BUS, &bme280SpiConfig );

  // Read ID
  if( BME280_CHIP_ID_VALUE == ReadChipId() )
  {
    // Read trimming values
    ReadBurstRegisters(BME280_TRIM_T_BASE, (uint8_t*)&TrimmingParameter.dig_T1, BME280_TRIM_T_SIZE);
    ReadBurstRegisters(BME280_TRIM_P_BASE, (uint8_t*)&TrimmingParameter.dig_P1, BME280_TRIM_P_SIZE);
    ReadBurstRegisters(BME280_TRIM_H1_BASE, (uint8_t*)&TrimmingParameter.dig_H1, BME280_TRIM_H1_SIZE );
    ReadBurstRegisters(BME280_TRIM_H2_BASE, (uint8_t*)&TrimmingParameter.dig_H2, BME280_TRIM_H2_SIZE );

    // Reformat some registers

    //dig_H4 [11:4] / [3:0]
    uint16_t digH4Msb = TrimmingParameter.dig_H4>>8u;
    TrimmingParameter.dig_H4 &=  0x000F;
    TrimmingParameter.dig_H4 += digH4Msb<<4u;
  }
}

/*
 *
 */
static uint8_t ReadChipId(void)
{
 return ReadOneRegister(BME280_REGISTER_CHIP_ID);
}

/*
 *
 */
static void ChangeMode(ChipMode_e nextMode)
{
  uint8_t ctrlMeas = ReadOneRegister(BME280_REGISTER_CTRL_MEAS);

  // Change the value
  ctrlMeas &= ~BME280_CTRL_MEAS_MODE_MASK;
  switch( nextMode )
  {
    case NORMAL:
      ctrlMeas |= BME280_CTRL_MEAS_MODE_NORMAL;
      break;
    case FORCED:
      ctrlMeas |= BME280_CTRL_MEAS_MODE_FORCED;
      break;
    case SLEEP:
    default:
      ctrlMeas |= BME280_CTRL_MEAS_MODE_SLEEP;
      break;
  }

  // Write back the new value
  WriteOneRegister(BME280_REGISTER_CTRL_MEAS, ctrlMeas);
}

/*
 *
 */
static uint8_t ReadOneRegister( uint8_t registerName )
{
  uint8_t value;
  mcuSdkGpioSetPinLow( BME280_CS_PIN );

  mcuSdkSpiWriteAndReadChar( BME280_SPI_BUS, registerName );
  value = mcuSdkSpiWriteAndReadChar( BME280_SPI_BUS, 0x00u );

  mcuSdkGpioSetPinHigh( BME280_CS_PIN );
  return value;
}

/*
 *
 */
static void WriteOneRegister( uint8_t registerName, uint8_t value )
{
  mcuSdkGpioSetPinLow( BME280_CS_PIN );

  mcuSdkSpiWriteAndReadChar( BME280_SPI_BUS, registerName );
  mcuSdkSpiWriteAndReadChar( BME280_SPI_BUS, value );

  mcuSdkGpioSetPinHigh( BME280_CS_PIN );
}

/*
 *
 */
static void ReadBurstRegisters( uint8_t startAddress, uint8_t *ptrValues, uint8_t number)
{
  mcuSdkGpioSetPinLow( BME280_CS_PIN );
  for(uint8_t i=0u; i< number; i++)
  {
    mcuSdkSpiWriteAndReadChar(BME280_SPI_BUS, startAddress + i);
    ptrValues[i] = mcuSdkSpiWriteAndReadChar( BME280_SPI_BUS, 0x00u );
  }
  mcuSdkGpioSetPinHigh( BME280_CS_PIN );
}

/*
 *
 */
static void WriteBurstRegisters( uint8_t startAddress, uint8_t valueList[], uint8_t number)
{
  mcuSdkGpioSetPinLow( BME280_CS_PIN );
  for(uint8_t i=0u; i< number; i++)
  {
    mcuSdkSpiWriteAndReadChar(BME280_SPI_BUS, (startAddress + i) & BME280_REGISTER_WRITE_MSK );
    mcuSdkSpiWriteAndReadChar( BME280_SPI_BUS, valueList[i] );
  }
  mcuSdkGpioSetPinHigh( BME280_CS_PIN );
}
