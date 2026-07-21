/*
 *******************************************************************************
 *
 *******************************************************************************
 */
#ifndef _USER_USART1_H
#define _USER_USART1_H

#include  "stm32f10x.h"
#include  "port.h"
#define USART1_RX_SIZE   (100)

#define UART1_RX_DLY_MAX 50 

typedef union
{
	u8 ch[USART1_RX_SIZE];
	struct
	{
	  u32 Frameheader;
	  u16 PacketLength;
	  u16 SequenceID;
	  u16 RequestCommand;
	  u16 VersionID;
	  u32 AnchorID;
	  u32 TagID;
	  u32 Distance;
	  s16 Azimuth;
	  s16 Elevation;
	  u16 TagStatus;
	  u16 BatchSn;
	  u32 Reserve;
	  u8  Xor;
	}item;
}USART1RX;


typedef struct
{

	u8   rxstatus;
	u8   getcheck;
	u8   incnum;
	u8   inclimit;
	u8   end_flag;
	u16  commond;
	u16  rxnum;
	u32  rxtime;
	USART1RX   rxbuf;	
}USART1struct;

extern USART1struct Usart1;


void USART1_Configuration(u32 nBuadrata);
void Usart1RxTimeOverCheck(void);
void Usart1RxStatusClr(void);


#endif	//_USER_USART1_H
