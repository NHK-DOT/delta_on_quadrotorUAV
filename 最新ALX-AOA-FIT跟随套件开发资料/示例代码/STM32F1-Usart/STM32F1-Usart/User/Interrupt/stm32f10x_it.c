/**
  ******************************************************************************
  * @file    stm32f10x_it.c 
  * @author  Yang
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
__IO unsigned long time32_incr;
unsigned char end_flag=0;

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
  unsigned char udata;

  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    udata = (u16)(USART1->DR & (u16)0x01FF);

    if((Usart1.rxstatus == UART_IDLE)||(Usart1.rxstatus == UART_RECVING))
    {
    	Usart1.rxbuf.ch[Usart1.rxnum++] = udata;
    	Usart1.rxtime = UART1_RX_DLY_MAX;
    	Usart1.rxstatus = UART_RECVING;

		if(Usart1.getcheck)
		{			
			Usart1.incnum++;
			if(Usart1.incnum==5)
			{
				Usart1.commond=udata;
			}
			
			if(Usart1.incnum==6)
			{
				Usart1.commond=(Usart1.commond<<8)+udata;
				if(Usart1.commond==0x2001)
				{
					Usart1.inclimit=33;
				}
				
			}
			
			if(Usart1.incnum==Usart1.inclimit)
			{
				Usart1.getcheck=0;
				Usart1.incnum=0;
				Usart1.inclimit=0;
				Usart1.commond=0;
				Usart1.rxstatus = UART_RECED;
			}
		}
		
    	if(udata == 0xFF)
    		Usart1.end_flag++;
    	else
    		Usart1.end_flag=0;
		
    	if(Usart1.end_flag >= 4)
    	{
    		Usart1.end_flag = 0;
			Usart1.getcheck = 1;
			Usart1.rxnum=4;
    	}
		
    	if(Usart1.rxnum>=USART1_RX_SIZE)
    	{
    		Usart1.rxstatus = UART_RECED;
    	}

    }

    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }

}




/**************************************END OF FILE********************************/
