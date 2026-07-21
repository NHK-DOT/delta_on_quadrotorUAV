/*! ----------------------------------------------------------------------------
 * @file    main.c
 * @brief   main loop for the Usart routine application
 *
 * @attention
 * Copyright 2025 (c) Suzhou Ailanxin electronic Technology Co., LTD.
 * All rights reserved.
 *
 * @author Yang
 ******************************************************************************
 * @attention
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, AiLanXin SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 ******************************************************************************
 ----------------------------------------------------------------------------*/

/* Includes */
#include "main.h"

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int main(void)
 {	
	peripherals_init();

    // main loop
    while(1)
    {
    IWDG_ReloadCounter();
		Usart1Process();
		//TagTrans.Distance:¾àÀë
		//TagTrans.Azimuth:·½Î»½Ç
		//TagTrans.Elevation:¸©Ñö½Ç
    }
	  
 }

