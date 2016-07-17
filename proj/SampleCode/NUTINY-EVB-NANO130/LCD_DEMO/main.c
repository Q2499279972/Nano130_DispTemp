/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 14/09/11 5:18p $
 * @brief    Demonstrate how to display RTC time on a LCD panel.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano100Series.h"
#include "lcd.h"
#include "LCDLIB.h"
#include "sys.h"
#include "rtc.h"
#include "adc.h"

/*!<Enable LCD for 100/128-Pin Package */
#define MFP_LCD_TYPEA() { \
                            SYS->PA_L_MFP |= 0x77770000;    /* seg 36 ~ 39 */\
                            SYS->PA_H_MFP |= 0x7777;        /* seg 20 ~ 23 */\
                            SYS->PB_L_MFP = 0x77777777;     /* seg 10 ~ 13, 4 ~ 7 */\
                            SYS->PB_H_MFP = 0x77777777;     /* LCD V1 ~ V3, seg 30 ~ 31, 24 ~ 26 */\
                            SYS->PC_L_MFP |= 0x777777;      /* LCD COM3 ~ COM0, DH1/DH2 */\
                            SYS->PC_H_MFP |= 0x77000000;    /* seg 32 ~ 33 */\
                            SYS->PD_L_MFP |= 0x77770000;    /* seg 2 ~ 3, 34 ~ 35 */\
                            SYS->PD_H_MFP = 0x77777777;     /* seg 0 ~ 1, 14 ~ 19 */\
                            SYS->PE_L_MFP |= 0x70000000;    /* seg 8 */\
                            SYS->PE_H_MFP |= 0x77700007;    /* seg 9, 27 ~ 29 */\
                        }

/*!<Enable LCD for 64-Pin Package */
#define MFP_LCD_TYPEB() { \
                            SYS->PA_L_MFP |= 0x77777700;    /* seg 18 ~ 23 */\
                            SYS->PA_H_MFP = 0x77777777;     /* seg 6 ~ 9, 24 ~ 27 */\
                            SYS->PB_L_MFP = 0x77777777;     /* COM2, COM3, seg 0 ~ 5 */\
                            SYS->PB_H_MFP = 0x77777777;     /* LCD V1 ~ V3, seg 10 ~ 14 */\
                            SYS->PC_L_MFP |= 0x70007777;    /* LCD COM1 ~ COM0, DH1/DH2, seg 17 */\
                            SYS->PC_H_MFP |= 0x77007777;    /* seg 28 ~ 31, 15 ~ 16 */\
                        }

__IO uint32_t g_u32RTC_Count  = 0;

volatile uint8_t u8ADF;

void ADC_IRQHandler(void)
{
    uint32_t u32Flag;

    // Get ADC conversion finish interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT);

    if(u32Flag & ADC_ADF_INT)
        u8ADF = 1;

    ADC_CLR_INT_FLAG(ADC, u32Flag);
}

volatile static unsigned int cnt=0;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_LXT_EN_Pos); // LXT Enable

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);
    /* Waiting for 32KHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_LXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    CLK->CLKSEL1 &= ~CLK_CLKSEL1_LCD_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_LCD_S_LXT);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_LCD_EN;
    CLK->APBCLK |= CLK_APBCLK_RTC_EN;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD  */
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA14_MFP_Msk|SYS_PA_H_MFP_PA15_MFP_Msk);
    SYS->PA_H_MFP |=  (SYS_PA_H_MFP_PA14_MFP_UART0_RX|SYS_PA_H_MFP_PA15_MFP_UART0_TX);

    /* Select LCD COMs, SEGs, V1 ~ V3, DH1, DH2 */
    MFP_LCD_TYPEA();

    /* Digital Input Path Disable */
    /* V1, V2 and V3 */
    PB->OFFD |= (0xE000000);
    /* DH1 and DH2 */
    PC->OFFD |= (0x30000);
    /* COM0~3 */
    PC->OFFD |= (0x3C0000);
    /* SEG0~39 */
    PA->OFFD |= (0x0FF00000);
    PB->OFFD |= (0xF1FF0000);
    PC->OFFD |= (0xC0000000);
    PD->OFFD |= (0xFFF00000);
    PE->OFFD |= (0xE1800000);


		*(volatile unsigned int *)(0x50000000+0x20) = 1;
		*(volatile unsigned int *)(0x50000000+0x6c)	|=7;
    /* Lock protected registers */
    SYS_LockReg();

}

/**
  * @brief  Main routine.
  * @param  None.
  * @return None.
  */
int32_t main(void)
{
	int i;
	SYS_Init();


	/* Do LCD Initializaton */
	LCD_Open(LCD_C_TYPE, 4, LCD_BIAS_THIRD, LCD_FREQ_DIV64, LCD_CPVOl_3V);
	LCD_EnableDisplay();
	LCD_DisableBlink();
	/* Enable ADC clock */
	CLK_EnableModuleClock(ADC_MODULE);

	ADC_Open(ADC, ADC_INPUT_MODE_SINGLE_END, ADC_OPERATION_MODE_SINGLE, ADC_CH_14_MASK);
	ADC_SET_REF_VOLTAGE(ADC, ADC_REFSEL_INT_VREF);

	ADC_EnableInt(ADC, ADC_ADF_INT);
	NVIC_EnableIRQ(ADC_IRQn);
	ADC_POWER_ON(ADC);
	
	for(i=0;i<64;i++)
	{
		u8ADF = 0;
		ADC_START_CONV(ADC);
		while (u8ADF == 0);
		ADC_GET_CONVERSION_DATA(ADC,14);
	}


	while (1)
	{
		float advalueoffset;
		unsigned int adcvalue;
		{
			adcvalue=0;
			for(i=0;i<64;i++)
			{
				u8ADF = 0;
				ADC_START_CONV(ADC);
			  while (u8ADF == 0);
				adcvalue+=ADC_GET_CONVERSION_DATA(ADC,14);
			}
			adcvalue/=64;
		}

		advalueoffset = adcvalue/4096.0*2.5-0.740;
		{
			float d = advalueoffset*1000/-1.73;
			if(d>999.0 || d<-999.0) 
			{
				LCDLIB_PutChar(0,0,' ');
				LCDLIB_PutChar(0,1,'E');
				LCDLIB_PutChar(0,2,'R');
				LCDLIB_PutChar(0,3,'R');
				LCDLIB_PutChar(0,4,' ');
				LCDLIB_PutChar(0,5,' ');
				LCDLIB_PutChar(0,6,' ');
				LCDLIB_PutChar(0,7,' ');
			}
			else if(d<0.0)
			{
				int dd = (int)d;
				LCDLIB_PutChar(0,0,'-');
				LCDLIB_PutChar(0,1,dd/100+'0');
				LCDLIB_PutChar(0,2,dd/10%10+'0');
				LCDLIB_PutChar(0,3,dd%10+'0');
				LCDLIB_PutChar(0,4,' ');
				LCDLIB_PutChar(0,5,' ');
				LCDLIB_PutChar(0,6,' ');
				LCDLIB_PutChar(0,7,' ');
			}
			else
			{
				int dd = (int)d;
				LCDLIB_PutChar(0,0,' ');
				LCDLIB_PutChar(0,1,dd/100+'0');
				LCDLIB_PutChar(0,2,dd/10%10+'0');
				LCDLIB_PutChar(0,3,dd%10+'0');
				LCDLIB_PutChar(0,4,' ');
				LCDLIB_PutChar(0,5,' ');
				LCDLIB_PutChar(0,6,' ');
				LCDLIB_PutChar(0,7,' ');
			}				
		}
		{
			cnt++;
			if(cnt>=5) cnt=0;
			if(cnt==4)
			{
				LCDLIB_PutChar(1,0,'-');
				LCDLIB_PutChar(1,1,'-');
				LCDLIB_PutChar(1,2,'-');
				LCDLIB_PutChar(1,3,'-');
			}
			if(cnt==3)
			{
				LCDLIB_PutChar(1,0,'-');
				LCDLIB_PutChar(1,1,'-');
				LCDLIB_PutChar(1,2,'-');
				LCDLIB_PutChar(1,3,' ');
			}
			if(cnt==2)
			{
				LCDLIB_PutChar(1,0,'-');
				LCDLIB_PutChar(1,1,'-');
				LCDLIB_PutChar(1,2,' ');
				LCDLIB_PutChar(1,3,' ');
			}
			if(cnt==1)
			{
				LCDLIB_PutChar(1,0,'-');
				LCDLIB_PutChar(1,1,' ');
				LCDLIB_PutChar(1,2,' ');
				LCDLIB_PutChar(1,3,' ');
			}
			if(cnt==0)
			{
				LCDLIB_PutChar(1,0,' ');
				LCDLIB_PutChar(1,1,' ');
				LCDLIB_PutChar(1,2,' ');
				LCDLIB_PutChar(1,3,' ');
			}
		}
		CLK_SysTickDelay(1000*1000);
	}
}


