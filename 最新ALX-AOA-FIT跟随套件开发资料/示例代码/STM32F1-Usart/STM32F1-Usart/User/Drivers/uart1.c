/*
 *******************************************************************************
 *                           Includes
 *******************************************************************************
 */

#include  "uart.h"
#include  "uart1.h"
#include  "dataoperation.h"

/*
 *******************************************************************************
 *
 *******************************************************************************
 */
USART1struct Usart1;

/**
  * @brief  Configures COM port.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void USART1_Configuration(u32 nBuadrata)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_InitStructure.USART_BaudRate = nBuadrata ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable the USART1 Pins Software Remapping */
	GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);// Enable the USARTx
	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

	/* Enable and set USART1 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  	//Enable USART1 TX DMA

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}
/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void Usart1RxTimeOverCheck(void)
{
    if(Usart1.rxtime)
    {
        Usart1.rxtime--;
        if(Usart1.rxtime==0)
            Usart1.rxstatus = UART_RECED;
    }
}

/*
 *******************************************************************************
 *
 *******************************************************************************
 */
void Usart1RxStatusClr(void)
{
    Usart1.rxstatus = UART_IDLE;
    Usart1.rxtime = 0;
    Usart1.rxnum = 0;
    FillString(Usart1.rxbuf.ch,sizeof(Usart1.rxbuf.ch),0);
}


