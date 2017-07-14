/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  An exmaple program of DJI-onboard-SDK portable for stm32
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 *******************************************************************************
 *                                                                             *
 *          --------               --------                 --------           *
 *         |        |   USART2    |        |    USART3     |        |          *
 *         |   PC   | <---------> | stm32  |  <----------> |  M100  |          *
 *         |        | (USB-TTL)   |        |               |        |          *
 *         |        |             |        |               |        |          *
 *          --------               --------                 --------           *
 *                                                                             *
 *                                                                             *
 *******************************************************************************
 * */

#include "main.h"


#undef USE_ENCRYPT
/*-----------------------DJI_LIB VARIABLE-----------------------------*/
using namespace DJI::onboardSDK;

HardDriver* driver = new STM32F4;
CoreAPI defaultAPI = CoreAPI(driver);
CoreAPI *coreApi = &defaultAPI;

Flight flight = Flight(coreApi);
FlightData flightData;
FlightData flightData_zkrtctrl;

VirtualRC virtualrc = VirtualRC(coreApi);
VirtualRCData myVRCdata =
{ 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,
    1024, 1024 };

extern TerminalCommand myTerminal;
extern LocalNavigationStatus droneState;
extern uint8_t myFreq[16];
extern int user_flight_ctrl;
		
int main()
{
  BSPinit();
  delay_nms(30);
  printf("This is the example App to test DJI onboard SDK on STM32F4Discovery Board! \r\n");
  printf("Refer to \r\n");
  printf("https://developer.dji.com/onboard-sdk/documentation/github-platform-docs/STM32/README.html \r\n");
  printf("for supported commands!\r\n");
  printf("Board Initialization Done!\r\n");
  delay_nms(1000);

  uint32_t runOnce = 1;
  uint32_t next500MilTick;
	uint32_t next20MilTick;
	
  while (1)
  {
    // One time automatic activation
    if (runOnce)
    {
      runOnce = 0;
      coreApi->setBroadcastFreq(myFreq);
      delay_nms(50);

      // The Local navigation example will run in broadcast call back function,
      // immediate after GPS position is updated
      coreApi->setBroadcastCallback(myRecvCallback,(DJI::UserData)(&droneState));

      //! Since OSDK 3.2.1, the new versioning system does not require you to set version.
      //! It automatically sets activation version through a call to getDroneVersion.
      coreApi->getDroneVersion();
      delay_nms(1000);

      User_Activate();      
      delay_nms(50);

      next500MilTick = driver->getTimeStamp() + 500;
			next20MilTick = driver->getTimeStamp() + 500;
    }

    if (driver->getTimeStamp() >= next500MilTick)
    {
      next500MilTick = driver->getTimeStamp() + 500;

      // Handle user commands from mobile device
      mobileCommandHandler(coreApi, &flight);

      // Handle user commands from serial (USART2)
      myTerminal.terminalCommandHandler(coreApi, &flight);
    }

		switch(user_flight_ctrl)
		{
			case VEL_USER_FLIGHT_CTRL:
				if(driver->getTimeStamp() >= next20MilTick)
				{
					user_flight_ctrl = 0;
					next20MilTick = driver->getTimeStamp() + 20;
					flightData_zkrtctrl.flag = 0x4a;
					flightData_zkrtctrl.x = 1;
					flightData_zkrtctrl.y = 0;
					flightData_zkrtctrl.z = 0;
					flight.setFlight(&flightData_zkrtctrl);					
				}
				break;
			case POS_USER_FLIGHT_CTRL:
				if(driver->getTimeStamp() >= next20MilTick)
				{
					user_flight_ctrl = 0;
					next20MilTick = driver->getTimeStamp() + 20;
					flightData_zkrtctrl.flag = 0x8a;
					flightData_zkrtctrl.x = 1;
					flightData_zkrtctrl.y = 0;
					flightData_zkrtctrl.z = 0;
					flight.setFlight(&flightData_zkrtctrl);					
				}
				break;
			default:
				break;			
		}
		
    coreApi->sendPoll();
  }
}
