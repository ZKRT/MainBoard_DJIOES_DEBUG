/*! @file BspUsart.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Usart helper functions and ISR for board STM32F4Discovery
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "stm32f4xx.h"
#include "BspUsart.h"
#include "timer.h"

extern int Rx_Handle_Flag;

using namespace DJI::OSDK;

extern Vehicle  vehicle;
extern Vehicle* v;
extern Control  control;

extern bool           isFrame;
bool                  isACKProcessed    = false;
bool                  ackReceivedByUser = false;
extern RecvContainer  receivedFramie;
extern RecvContainer* rFrame;

// extern CoreAPI defaultAPI;
// extern CoreAPI *coreApi;
// extern Flight flight;
// extern FlightData flightData;

extern uint8_t come_data;
extern uint8_t Rx_length;
extern int     Rx_adr;
extern int     Rx_Handle_Flag;
extern uint8_t Rx_buff[];

void USART3_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);     //tx
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);     //rx

}

void USART1_Gpio_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);        //tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);        //rx
}

/*
 * USART3 is used for receiving commands from PC and
 * printing debug information to PC
 */
void USART3_Config(void)  //PC USART DEBUG PORT
{
  USART3_Gpio_Config();

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART3, ENABLE);

  while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET)
    ;
}

/*
 * USART1 is used for communicating with the DJI flight controller
 * The Baud rate needs to match the Baud rate used by the flight controller
 */
void USART1_Config(void)  //DJI USART PORT
{
  USART1_Gpio_Config();

  USART_InitTypeDef USART_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  USART_InitStructure.USART_BaudRate = 230400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART1, ENABLE);
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != SET)
    ;

}

void USARTxNVIC_Config()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitTypeDef NVIC_InitStructure_USART1;
  NVIC_InitStructure_USART1.NVIC_IRQChannelPreemptionPriority = 0x04;
  NVIC_InitStructure_USART1.NVIC_IRQChannelSubPriority = 0x03;
  NVIC_InitStructure_USART1.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure_USART1.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART1);

  NVIC_InitTypeDef NVIC_InitStructure_USART3;
  NVIC_InitStructure_USART3.NVIC_IRQChannelPreemptionPriority = 0x03;
  NVIC_InitStructure_USART3.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure_USART3.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure_USART3.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_USART3);
}

void
UsartConfig()
{
  USART3_Config();
  USART1_Config();
  USARTxNVIC_Config();
}
/*
DJI::OSDK::ACK::ErrorCode
waitForACK()
{
  ACK::ErrorCode ack;
  ack.data = ACK_NO_RESPONSE_ERROR;
  memset(&(ack.data), 0, sizeof(ack.data));
  uint32_t next500MilTick;
	uint8_t cmd[] = {rFrame->recvInfo.cmd_set, rFrame->recvInfo.cmd_id};

  //	next500MilTick = v->protocolLayer->getDriver()->getTimeStamp() + 500;

  //	while(rFrame->dispatchInfo.isCallback != true &&
  //		v->protocolLayer->getDriver()->getTimeStamp() < next500MilTick)
  while (true)
  {
    if (isACKProcessed == true)
    {
      if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_ACTIVATION &&
          rFrame->recvInfo.cmd_id == DJI::OSDK::CMD_ID_ACTIVATE)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
      else if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_SUBSCRIBE &&
               rFrame->recvInfo.cmd_id ==
                 DJI::OSDK::CMD_ID_SUBSCRIBE_VERSION_MATCH)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
      else if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_SUBSCRIBE &&
               rFrame->recvInfo.cmd_id ==
                 DJI::OSDK::CMD_ID_SUBSCRIBE_ADD_PACKAGE)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
      else if (rFrame->recvInfo.cmd_set == DJI::OSDK::CMD_SET_CONTROL &&
               rFrame->recvInfo.cmd_id == DJI::OSDK::CMD_ID_TASK)
      {
        ack.data = rFrame->recvData.ack;
        ack.info = rFrame->recvInfo;

        return ack;
      }
    }
  }

  // return ack;
}
*/
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void
USART1_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
  {
    isACKProcessed = false;
    isFrame = v->protocolLayer->byteHandler(USART_ReceiveData(USART1), rFrame);
    if (isFrame == true)
    {
      //! Trigger default or user defined callback
      v->processReceivedData(*rFrame);

      //! Reset
      isFrame        = false;
      isACKProcessed = true;
    }
  }
}

#ifdef __cplusplus
}
#endif //__cplusplus
