/*! @file MfioSample.cpp
 *  @version 1.0
 *  @date Jan 2018
 *
 *  @brief
 *  Mfio control STM32 example.
 *
 *  Copyright 2018 zkrt. All right reserved.
 *
 * */

#include "MfioSample.h"

extern Vehicle  vehicle;
extern Vehicle* v;
int channel7value;
int channel7cnt;
unsigned int timeouttick;

void mfio_pwmout_config(void)
{
	channel7value = 1000;
	channel7cnt=10;
  v->mfio->config(MFIO::MODE_PWM_OUT, MFIO::CHANNEL_7, channel7value, 50);
  delay_nms(1000);
	timeouttick = v->protocolLayer->getDriver()->getTimeStamp() + 5000;
}

void mfio_pwmout(void)
{
	timeouttick = v->protocolLayer->getDriver()->getTimeStamp() + 3000;
	while(1)
	{
		if(timeouttick <=v->protocolLayer->getDriver()->getTimeStamp())
		{
			channel7value+=channel7cnt;
			if(channel7value>2000)
				channel7value = 1000;
			v->mfio->setValue(MFIO::CHANNEL_7, channel7value);
			timeouttick = v->protocolLayer->getDriver()->getTimeStamp() + 200;
		}
	}
}
