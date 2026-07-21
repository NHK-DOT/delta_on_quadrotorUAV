/*
 *******************************************************************************
 *
 *******************************************************************************
 */
#ifndef _USER_USART1PROCESS_H
#define _USER_USART1PROCESS_H
#include "main.h"

typedef struct
{
	u32 Distance;
	s16 Azimuth;//·½Î»
	s16 Elevation;
}TAGTRANS;
extern TAGTRANS TagTrans;

void Usart1Process(void);
void InitProcess(void);

#endif	//_USER_USART1PROCESS_H
