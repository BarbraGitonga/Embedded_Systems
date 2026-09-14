#include "stm32f746xx.h"
//AHB : 216MHz
//APB1 : 54MHz
//APB2 : 54Mhz

void Initialize_MCU(void) /* initialize STM32F767VGT6 MCU */
{
    // (1) Configure the instruction cache and data cache
    SCB_EnableICache(); // enable L1 instruction cache
    SCB_EnableDCache(); // enable L1 data cache

    // (2) Configure the ART accelerator, prefetch buffer and wait cycles
    FLASH->ACR = 0x00000307; // 7 waits, enable ART accelerator and prefetch

    // (3) Configure HSE and PLL (system clock SYSCLK = 216MHz)
    RCC->CR |= 0x00010001; // HSE on, HSI on
    while((RCC->CR & 0x00000002) == 0); // wait until HSIRDY = 1

    RCC->CFGR = 0x00000000; // SYSCLK = HSI
    while((RCC->CFGR & 0x0000000C) != 0); // wait until SYSCLK = HSI

    //PLL configuration
    RCC->CR = 0x00010001; // PLL off, HSE on, HSI on
    RCC->PLLCFGR = 0x09403608;  //1001 0100 0000 0011 0110 0000 1000
    							// Divide the 16MHz HSE by 8 to make 2MHz --> the VCO circuit inside the PLL is stable at 1~2MHz
    							// Multiply 2MHz by 216 to get 432MHz --> divide by 2 again to generate the 216MHz clock
    							// SYSCLK = HSE*PLLN/PLLM/PLLP = 16MHz*216/8/2 = 216MHz
                                // PLL48CK = HSE*PLLN/PLLM/PLLQ = 16MHz*216/8/9 = 48MHz
    RCC->CR = 0x01010001; // PLL on, HSE on, HSI on
    while((RCC->CR & 0x02000000) == 0); // wait until PLLRDY = 1

    // (4) Configure over-drive
    // Without over-drive the maximum speed of the STM32F767 is 180MHz
    // Enable over-drive to reach 216MHz --> raises the MCU internal voltage regulator output

    RCC->APB1ENR |= 0x10000000; // power module clock(PWREN = 1)
    PWR->CR1 |= 0x00010000; // over-drive enable(ODEN = 1)
    while((PWR->CSR1 & 0x00010000) == 0); // ODRDY = 1 ?
    PWR->CR1 |= 0x00020000; // over-drive switching enable(ODSWEN = 1)
    while((PWR->CSR1 & 0x00020000) == 0); // ODSRDY = 1 ?
    //CPU 216MHz configuration complete

    // (5) Configure the peripheral clocks(APB1CLK = APB2CLK = 54MHz)
    RCC->CFGR = 0x3040B402; // SYSCLK = PLL, AHB = 216MHz, APB1 = APB2 = 54MHz
    RCC->DCKCFGR1 = 0x01000000; // TIMxCLK = 216MHz
    while((RCC->CFGR & 0x0000000C) != 0x00000008); // wait until SYSCLK = PLL
    RCC->CR |= 0x00080000; // CSS on

    // (6) Configure I/O compensation
    RCC->APB2ENR |= 0x00004000; // peripheral clock(SYSCFG = 1)
    SYSCFG->CMPCR = 0x00000001; // enable compensation cell
}
