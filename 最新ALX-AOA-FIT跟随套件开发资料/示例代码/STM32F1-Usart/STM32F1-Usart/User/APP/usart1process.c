/*! ----------------------------------------------------------------------------
 * @file	Usart1Process.c
 * @brief	Following car Each function realization process
 * 
 * @attention
 * Copyright 2025 (c) Suzhou Ailanxin electronic Technology Co., LTD.
 * All rights reserved.
 *
 * @author Yang
 ----------------------------------------------------------------------------*/

#include "main.h"

TAGTRANS TagTrans;

void DoCmd(u16 cmd);
void Usart1Receive(void);


/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void Usart1Process(void)
{
	Usart1Receive();
}


/*
 *******************************************************************************
 *   接收数据
 *******************************************************************************
 */
void Usart1Receive(void)
{
	if(Usart1.rxstatus!=UART_RECED)
		return ;

	DoCmd(U16HighLowByteSwap(Usart1.rxbuf.item.RequestCommand));

	Usart1RxStatusClr();
}
/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void DoCmd(u16 cmd)
{
	Usart1.end_flag = 0;
	
	switch(cmd)
	{

		case 0x2001:
			TagTrans.Distance=U32HighLowByteSwap(Usart1.rxbuf.item.Distance);
			TagTrans.Azimuth=U16HighLowByteSwap(Usart1.rxbuf.item.Azimuth);
			TagTrans.Elevation=U16HighLowByteSwap(Usart1.rxbuf.item.Elevation);
			break;

		default:
			break;
	}
}

