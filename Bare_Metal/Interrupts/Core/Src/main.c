#include "stm32f7xx.h"

void Initialize_MCU(void)
{
    /* 1) Enable Instruction and Data Caches */
    SCB_EnableICache();
    SCB_EnableDCache();

    /* 2) Enable Power Controller Clock */
    RCC->APB1ENR |= 0x10000000; // PWREN = 1

    /* 3) Configure Flash: 7 Wait States, Prefetch, ART Accelerator */
    FLASH->ACR = 0x00000707;    // Latency 7 (216MHz @ 3.3V), PRFTEN, ARTEN

    /* 4) Ensure HSI is ON and ready */
    RCC->CR |= 0x00000001;      // HSION
    while ((RCC->CR & 0x00000002) == 0); // Wait for HSIRDY

    /* 5) Configure PLL using HSI (16 MHz):
          VCO in  = 16 MHz / PLLM(16) = 1 MHz
          VCO out = 1 MHz * PLLN(432) = 432 MHz
          SYSCLK  = 432 MHz / PLLP(2) = 216 MHz
          USB/SD  = 432 MHz / PLLQ(9) = 48 MHz
          PLLSRC  = 0 (HSI selected) */
    RCC->PLLCFGR = 0x0940D810;  // PLLQ=9, PLLP=2 (00b), PLLN=432 (0x1B0), PLLM=16, PLLSRC=HSI

    /* 6) Enable PLL and wait for lock */
    RCC->CR |= 0x01000000;      // PLLON
    while ((RCC->CR & 0x02000000) == 0); // Wait for PLLRDY

    /* 7) Enable Over-Drive Mode (Required for frequencies > 180 MHz) */
    PWR->CR1 |= 0x00010000;     // ODEN = 1
    while ((PWR->CSR1 & 0x00010000) == 0); // Wait for ODRDY

    PWR->CR1 |= 0x00020000;     // ODSWEN = 1
    while ((PWR->CSR1 & 0x00020000) == 0); // Wait for ODSRDY

    /* 8) Set Bus Prescalers before switching SYSCLK:
          AHB  = SYSCLK / 1  = 216 MHz
          APB1 = SYSCLK / 4  = 54 MHz (Max allowed is 54 MHz)
          APB2 = SYSCLK / 2  = 108 MHz */
    RCC->CFGR &= 0xFFFFFFFF0;
    RCC->CFGR |= 0x00009400;    // PPRE2 = div2 (100b), PPRE1 = div4 (101b), HPRE = div1

    /* 9) Switch SYSCLK to PLL */
    RCC->CFGR |= 0x00000002;    // SW = PLL (10b)
    while ((RCC->CFGR & 0x0000000C) != 0x00000008); // Wait until SWS indicates PLL
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
