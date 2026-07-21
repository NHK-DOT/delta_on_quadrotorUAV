/*! ----------------------------------------------------------------------------
 * @file	port.c
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 * Copyright 2025 (c) Suzhou Ailanxin electronic Technology Co., LTD.
 * All rights reserved.
 *
 * @author Yang
 ----------------------------------------------------------------------------*/

#include "main.h"

int RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;
    RCC_ClocksTypeDef RCC_ClockFreq;

    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus != ERROR)
    {
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /****************************************************************/
        /* HSE= up to 25MHz (on EVB1000 is 12MHz),
         * HCLK=72MHz, PCLK2=72MHz, PCLK1=36MHz 						*/
        /****************************************************************/
        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_Latency_2);
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /*  ADCCLK = PCLK2/4 */
        //RCC_ADCCLKConfig(RCC_PCLK2_Div6);

        /* Configure PLLs *********************************************************/
        /* PLL1 configuration: PLLCLK = (HCLK / 2) * 9 = 72 MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08) {}
    }

    RCC_GetClocksFreq(&RCC_ClockFreq);

    return 0;
}



void IWDG_Configuration(void)
{
    /* IWDG timeout equal to 280 ms (the timeout may varies due to LSI frequency dispersion) */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);// Enable write access to IWDG_PR and IWDG_RLR registers
    IWDG_SetPrescaler(IWDG_Prescaler_256);       // IWDG counter clock: 40KHz(LSI) / 256 = 156.25Hz
    IWDG_SetReload(312);                         // 256 / 40KHz(LSI) * 312 = 1996.8 ms
    IWDG_ReloadCounter();                        // Reload IWDG counter */
    IWDG_Enable();                               // Enable IWDG (the LSI oscillator will be enabled by hardware) */
}


/*
 *******************************************************************************
 *
 *******************************************************************************
 */

int peripherals_init(void)
{
    RCC_Configuration();
    USART1_Configuration(115200);
	IWDG_Configuration();  
    return 0;
}



