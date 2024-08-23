/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:27p $
 * @brief
 *           Transmit and receive data from PC terminal through RS232 interface.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NUC029xGE.h"
#include "modbus_crc.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HXT
#define PLL_CLOCK       72000000

#define RXBUFSIZE 256

static uint16_t Holding_Registers_Database[50]={
		0000,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   40001-40010
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 40011-40020
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 40021-40030
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 40031-40040
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 40041-40050
};

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t RXDATA[RXBUFSIZE]  = {0};
uint8_t TXDATA[RXBUFSIZE];

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);
void sendData(uint8_t *data, int size);
uint8_t readHoldingRegs(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Wait for HXT clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk|CLK_APBCLK0_TMR0CKEN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HXT | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UARTDIV_Msk)) | CLK_CLKDIV0_UART(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_UART0_RXD | SYS_GPA_MFPL_PA2MFP_UART0_TXD);
}

void Timer_Init(void){
    TIMER0->CMP = __HXT;
    TIMER0->CTL = TIMER_CTL_INTEN_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER0, 2);
    NVIC_EnableIRQ(TMR0_IRQn);
    TIMER_Start(TIMER0);
}

void TMR0_IRQHandler(void){
	if(TIMER_GetIntFlag(TIMER0) == 1)
	    {
	        /* Clear Timer0 time-out interrupt flag */
	        TIMER_ClearIntFlag(TIMER0);
	        readHoldingRegs();
	        // UART_Write(UART0, RXDATA, 8);
 }
}

void sendData (uint8_t *data, int size)
{
	// we will calculate the CRC in this function itself
	uint16_t crc = crc16(data, size);
	data[size] = crc&0xFF;   // CRC LOW
	data[size+1] = (crc>>8)&0xFF;  // CRC HIGH

	UART_Write(UART0, data, size+2);
}

uint8_t readHoldingRegs(void) {
	TXDATA[0] = RXDATA[0];
	TXDATA[1] = RXDATA[1];
	uint16_t startAddr = ((RXDATA[2] << 8) | RXDATA[3]); // start Register Address
	uint16_t numRegs = ((RXDATA[4] << 8) | RXDATA[5]); // number to registers master has requested
	TXDATA[2] = numRegs * 2;
	int indx = 3; // we need to keep track of how many bytes has been stored in TXDATA Buffer

	for (int i = 0; i < numRegs; i++) // Load the actual data into TXDATA buffer
			{
		TXDATA[indx++] = (Holding_Registers_Database[startAddr] >> 8) & 0xFF; // extract the higher byte
		TXDATA[indx++] = (Holding_Registers_Database[startAddr]) & 0xFF; // extract the lower byte
		startAddr++;  // increment the register address
	}
	sendData(TXDATA, indx); // send data... CRC will be calculated in the function itself
	return 1;   // success
}



void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 9600);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_2;
    UART0->FIFO = UART_FIFO_RFITL_8BYTES ;
    UART_SetTimeoutCnt(UART0, 40);
    UART0->INTEN |= UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk;
    NVIC_EnableIRQ(UART02_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

	Timer_Init();

    /* Init UART0 for printf and test */
    UART0_Init();

    while(1){
//    	UART_Write(UART0,RXDATA,8);
//    	for(volatile int t=0; t < 0x10000; t++){}
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
	uint8_t u8InChar[16] = { 0 };
    uint32_t u32IntSts = UART0->INTSTS;
	uint8_t fifotx = 0;

    if((u32IntSts & UART_INTSTS_RDAINT_Msk)) //|| (u32IntSts & UART_INTSTS_RXTOINT_Msk))
    {
    	int index = 0;

        /* Get all the input characters */
        //while(UART0->INTSTS & UART_INTSTS_RDAIF_Msk)
    	while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            /* Get the character from UART Buffer */
            u8InChar[index] = UART0->DAT;
            /* Check if buffer full */
            if(g_u32comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                RXDATA[g_u32comRtail] = u8InChar[index];
                g_u32comRtail = (g_u32comRtail == (RXBUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
                g_u32comRbytes++;
                index++;
            }
        }
    }
    if(u32IntSts & UART_INTSTS_THREINT_Msk)
    {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp)
        {
            fifotx = RXDATA[g_u32comRhead];
           while(UART_IS_TX_FULL(UART0));  /* Wait Tx is not full to transmit data */
           // UART_Write(UART0, RXDATA, 8);
            g_u32comRhead = (g_u32comRhead == (RXBUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{

    /* Enable UART RDA and THRE Interrupt */

    while(g_bWait);

    /* Disable UART RDA and THRE Interrupt */
    UART0->INTEN &= ~(UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk);
    NVIC_DisableIRQ(UART02_IRQn);
    g_bWait = TRUE;
}
