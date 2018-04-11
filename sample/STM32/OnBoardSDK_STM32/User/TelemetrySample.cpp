/*! @file telemetry_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "TelemetrySample.h"

extern Vehicle  vehicle;
extern Vehicle* v;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool
getBroadcastData()
{
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 2000;

  // We will listen to 14 broadcast data sets:
  /* Channels definition for A3/N3
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Statusack
   * 12 - Battery Level
   * 13 - Control Information
   */

  // Please make sure your drone is in simulation mode. You can
  // fly the drone with your RC to get different values.

  Telemetry::Status         status;
  Telemetry::GlobalPosition globalPosition;
  Telemetry::RC             rc;
  Telemetry::Vector3f       velocity;
  Telemetry::Quaternion     quaternion;

  const int TIMEOUT = 20;

  // Re-set Broadcast frequencies to their default values
  ACK::ErrorCode ack = v->broadcast->setBroadcastFreqDefaults(TIMEOUT);

  // Print in a loop for 2 seconds
  while (elapsedTimeInMs < timeToPrintInMs)
  {
    // Matrice 100 broadcasts only flight status
    status         = v->broadcast->getStatus();
    globalPosition = v->broadcast->getGlobalPosition();
    rc             = v->broadcast->getRC();
    velocity       = v->broadcast->getVelocity();
    quaternion     = v->broadcast->getQuaternion();
		
    printf("Counter = %d:\n", elapsedTimeInMs);
    printf("-------\n");
		printf("timestemp (ms/ns) = %d, %d\n", v->broadcast->getTimeStamp().time_ms, v->broadcast->getTimeStamp().time_ns); //this is the duration after last time the flight controller power cycled.
    printf("Attitude Quaternion (w,x,y,z) = %.3f, %.3f, %.3f, %.3f\n", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);		
    printf("Acceleration (vx,vy,vz) = %.3f, %.3f, %.3f\n", v->broadcast->getAcceleration().x, v->broadcast->getAcceleration().y, 
				v->broadcast->getAcceleration().z);
    printf("Velocity (vx,vy,vz) = %.3f, %.3f, %.3f\n", velocity.x, velocity.y, velocity.z);
    printf("AngularRate (vx,vy,vz) = %.3f, %.3f, %.3f\n", v->broadcast->getAngularRate().x, v->broadcast->getAngularRate().y, 
				v->broadcast->getAngularRate().z);
    printf("VelocityInfo(health/reserve) = %d, %d\n", v->broadcast->getVelocityInfo().health, v->broadcast->getVelocityInfo().reserve);					 
    printf("GlobalPosition (height/lat/longi/alt/health) = %.3f, %.3f, %.3f, %.3f, %d\n", globalPosition.height, globalPosition.latitude, 
				globalPosition.longitude, globalPosition.altitude, globalPosition.health);
		printf("gpsinfo(date/time) = %d, %d\n", v->broadcast->getGPSInfo().time.date, v->broadcast->getGPSInfo().time.time);//日期挺准，时间小时不准，分钟秒倒是挺准
    printf("RC Commands (r/p/y/thr/mode/gear) = %d, %d, %d, %d, %d, %d\n", rc.roll, rc.pitch, rc.yaw, rc.throttle, rc.mode, rc.gear);		
    printf("Gimbal (r/p/y/rl/pl/yl) = %.3f, %.3f, %.3f, %d, %d, %d\n", v->broadcast->getGimbal().roll, v->broadcast->getGimbal().pitch, 
				v->broadcast->getGimbal().yaw, v->broadcast->getGimbal().rollLimit, v->broadcast->getGimbal().pitchLimit, v->broadcast->getGimbal().yawLimit);		
    //测试发现：yaw角度回馈范围在-180度~+180度，以横轴为分割线考虑，假设上端为正，假设从左往右递增角度为0~180，下端为负，-0~-180，在交界处转换，符号变反。
/**************************************
*              90  
*         60       120    
*     30                150
*  0                         180
* -0                        -180
*    -30               -150
*		     -60      -120
*				      -90
***************************************/
    printf("Status (flight/error/gear/mode) = %d, %d, %d, %d\n", (unsigned)status.flight, status.error, status.gear, status.mode);
    printf("Battery (capacity/cur/percent/v) = %d, %d, %d, %d\n", v->broadcast->getBatteryInfo().capacity, v->broadcast->getBatteryInfo().current, 
				v->broadcast->getBatteryInfo().percentage, v->broadcast->getBatteryInfo().voltage);	
	  printf("SDKInfo(devs/ctrm/fs/vrcs/reser) = %d, %d, %d, %d, %d\n", v->broadcast->getSDKInfo().deviceStatus, v->broadcast->getSDKInfo().controlMode, 
				v->broadcast->getSDKInfo().flightStatus, v->broadcast->getSDKInfo().vrcStatus, v->broadcast->getSDKInfo().reserved);
    printf("-------\n\n");
		
    elapsedTimeInMs += 50;
		delay_nms(500); //zkrt
  }

  printf("Done printing!\n");
  return true;
}

bool
subscribeToData()
{

  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 4000;

  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 50 Hz
  // 5. Velocity at 50 Hz
  // 6. Quaternion at 200 Hz

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 2: Subscribe to RC Channel and Velocity at freq 50 Hz
  pkgIndex                  = 2;
  freq                      = 50;
  TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY };
  numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  enableTimestamp           = false;

  pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 3: Subscribe to Quaternion at freq 200 Hz.
  pkgIndex                   = 3;
  freq                       = 200;
  TopicName topicList200Hz[] = { TOPIC_QUATERNION };
  numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  enableTimestamp = false;

  pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Wait for the data to start coming in.
  delay_nms(8000);

  // Get all the data once before the loop to initialize vars
  TypeMap<TOPIC_STATUS_FLIGHT>::type flightStatus;
  TypeMap<TOPIC_GPS_FUSED>::type     latLon;
  TypeMap<TOPIC_RC>::type            rc;
  TypeMap<TOPIC_VELOCITY>::type      velocity;
  TypeMap<TOPIC_QUATERNION>::type    quaternion;

  uint32_t PRINT_TIMEOUT = 4000; // milliseconds
  uint32_t RETRY_TICK    = 500;  // milliseconds
  uint32_t nextRetryTick = 0;    // millisesonds
  uint32_t timeoutTick;

  timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + PRINT_TIMEOUT;
  do
  {
    flightStatus = v->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = v->subscribe->getValue<TOPIC_GPS_FUSED>();
    rc           = v->subscribe->getValue<TOPIC_RC>();
    velocity     = v->subscribe->getValue<TOPIC_VELOCITY>();
    quaternion   = v->subscribe->getValue<TOPIC_QUATERNION>();

    printf("Counter = %d:\n", elapsedTimeInMs);
    printf("-------\n");
    printf("Flight Status = %d\n", (int)flightStatus);
    printf("Position (LLA) = %.3f, %.3f, %.3f\n", latLon.latitude,
           latLon.longitude, latLon.altitude);
    printf("RC Commands (r/p/y/thr) = %d, %d, %d, %d\n", rc.roll, rc.pitch,
           rc.yaw, rc.throttle);
    printf("Velocity (vx,vy,vz) = %.3f, %.3f, %.3f\n", velocity.data.x,
           velocity.data.y, velocity.data.z);
    printf("Attitude Quaternion (w,x,y,z) = %.3f, %.3f, %.3f, %.3f\n",
           quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    printf("-------\n\n");

    delay_nms(500);
    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  printf("Done printing!\n");
  v->subscribe->removePackage(0);
  delay_nms(3000);
  v->subscribe->removePackage(1);
  delay_nms(3000);
  v->subscribe->removePackage(2);
  delay_nms(3000);
  v->subscribe->removePackage(3);
  delay_nms(3000);

  return true;
}