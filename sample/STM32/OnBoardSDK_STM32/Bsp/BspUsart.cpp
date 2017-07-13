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

#include "BspUsart.h"
#include "DJI_API.h"
#include "DJI_HardDriver.h"
#include "DJI_Flight.h"
extern int Rx_Handle_Flag;

using namespace DJI::onboardSDK;
extern CoreAPI defaultAPI;
extern CoreAPI *coreApi;
extern Flight flight;
extern FlightData flightData;

extern unsigned char come_data;
extern unsigned char Rx_length;
extern int Rx_adr;
extern int Rx_Handle_Flag;
extern unsigned char Rx_buff[];

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

  USART_InitStructure.USART_BaudRate = 57600;
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

void UsartConfig()
{
  USART3_Config();
  USART1_Config();
  USARTxNVIC_Config();
}

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

void USART1_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
  {
    coreApi->byteHandler(USART_ReceiveData(USART1)); //Data from M100 were committed to "byteHandler"
  }
}


#ifdef __cplusplus
}
#endif //__cplusplus
