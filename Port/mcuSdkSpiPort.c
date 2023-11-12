//------------------------------------------------------------------------------
//! \file
//! \brief      application utilities for freertos
//!
//!             Insert a blank line to add a new paragraph.
//! \copyright  Copyright (C) ITRON.
//!             All rights reserved.
//!             Developed at ITRON, Macon, France.
//!             Reproduction in whole or part is prohibited without
//!             the written permission of the copyright owner.
//------------------------------------------------------------------------------

//******************************************************************************
// Includes
//******************************************************************************
#include <stdint.h>
#include <string.h>

#include "stm32l1xx.h"

#include "mcuSdkSpiPort.h"

void McuSdkSpiPortRoutePin( SPI_TypeDef *ptrSpi )
{
  // I only uses SPI1 so far
  if( ptrSpi == SPI1 )
  {
    // SPI_1
    // Pins usage
    // PA5 <-> CLK
    // PA6 <-> MISO
    // PA7 <-> MOSI

    // Set alternate functions to pins
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5_Msk;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7_Msk;

    GPIOA->AFR[0] |= 5 << GPIO_AFRL_AFSEL5_Pos;
    GPIOA->AFR[0] |= 5 << GPIO_AFRL_AFSEL6_Pos;
    GPIOA->AFR[0] |= 5 << GPIO_AFRL_AFSEL7_Pos;

    // Then set pins as alternate
    GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER5_1;

    GPIOA->MODER &= ~GPIO_MODER_MODER6_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER6_1;

    GPIOA->MODER &= ~GPIO_MODER_MODER7_Msk;
    GPIOA->MODER |= GPIO_MODER_MODER7_1;
  }
}
