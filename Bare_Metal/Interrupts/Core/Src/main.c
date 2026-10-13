#include "stm32f7xx.h"

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
    RCC->PLLCFGR = 0x09406C19; // PLLM=25, PLLN=432, PLLP=2, PLLQ=9, source=HSE  //1001 0100 0000 0011 0110 0000 1000
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

/* ----- GPIO & EXTI initialization ----- */
void GPIO_EXTI_Init(void)
{
    /* 1) Enable clocks for GPIOI (bit 8) and SYSCFG (bit 14) */
    RCC->AHB1ENR |= 0x00000100;
    RCC->APB2ENR |= 0x00004000;

    /* 2) Configure PI1 as Output (01b) and PI11 as Input (00b) */
    // PI1 is at bits [3:2], PI11 is at bits [23:22]
    GPIOI->MODER &= 0xFF3FFFF3;  // Clear both (PI11 becomes input 00b)
    GPIOI->MODER |= 0x00000004;  // Set PI1 to output (01b)

    /* 3) Output settings for PI1 (push-pull, high speed, no pull) */
    GPIOI->OTYPER  &= 0xFFFFFFFD;
    GPIOI->OSPEEDR |= 0x0000000C;
    GPIOI->PUPDR   &= 0xFFFFFFF3;

    /* 4) Configure PI11 with Pull-Down (02b at bits [23:22]) */
    // Keeps line stable low until button is pressed
    GPIOI->PUPDR &= 0xFF3FFFFF;
    GPIOI->PUPDR |= 0x00800000;

    /* 5) Route EXTI11 to Port I in SYSCFG */
    // EXTICR[2] controls lines 8-11; EXTI11 is bits [15:12]
    SYSCFG->EXTICR[2] &= 0x00000FFF;  // Clear bits [15:12]
    SYSCFG->EXTICR[2] |= 0x00008000;  // 0x8 = Port I

    /* 6) Configure EXTI Line 11 */
    EXTI->IMR  |= 0x00000800;         // Unmask line 11 (bit 11)
    EXTI->RTSR |= 0x00000800;         // Rising-edge trigger (button press)
    EXTI->FTSR &= 0xFFFFF7FF;         // Disable falling-edge trigger
    EXTI->PR    = 0x00000800;         // Clear any pending flag

    /* 7) Enable EXTI15_10 IRQ in NVIC */
    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ----- EXTI Line 11 ISR ----- */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & 0x00000800)        // Check line 11 flag
    {
        EXTI->PR = 0x00000800;        // Clear flag (write 1 to clear)
        GPIOI->ODR ^= 0x00000002;     // Toggle LED on PI1
    }
}

/* ----- main ----- */
int main(void)
{
    Initialize_MCU(); //for now to run safely on default clock
    GPIO_EXTI_Init();

    // Turn LED off initially
    GPIOI->BSRR = 0x00020000; // Reset PI1 (bit 17)

    while (1)
    {
        /* Idle - interrupt handles the toggling */
    }
}
