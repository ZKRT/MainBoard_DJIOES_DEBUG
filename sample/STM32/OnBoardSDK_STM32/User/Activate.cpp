/*! @file Activate.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Activation process for the STM32 example App.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "Activate.h"

extern Vehicle  vehicle;
extern Vehicle* v;

void
userActivate()
{
  //! At your DJI developer account look for: app_key and app ID

  static char key_buf[65] = "ac5100a048cc4c4a08ce23200481d058d7455e69b8668fc0e49f29bd856c185a"; /*"your app_key"*/

  DJI::OSDK::Vehicle::ActivateData user_act_data;
  user_act_data.ID = 1031967; /*your app ID here*/

  user_act_data.encKey = key_buf;

  v->activate(&user_act_data);
	
}
