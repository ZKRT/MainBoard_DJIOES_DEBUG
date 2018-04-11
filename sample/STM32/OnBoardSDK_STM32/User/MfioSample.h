/*! @file MfioSample.h
 *  @version 1.0
 *  @date Jun 2018
 *
 *  @brief
 *  Mfio control STM32 example.
 *
 *  Copyright 2018 ZKRT. All right reserved.
 *
 * */

#ifndef MFIOSAMPLE_H
#define MFIOSAMPLE_H

#include "BspUsart.h"
#include "dji_vehicle.hpp"
#include "timer.h"
#include <math.h>
#include "dji_mfio.hpp"

using namespace DJI::OSDK;

void mfio_pwmout_config(void);
void mfio_pwmout(void);
#endif // FLIGHTCONTROLSAMPLE_H