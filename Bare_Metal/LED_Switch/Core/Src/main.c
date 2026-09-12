#include "stm32f746xx.h"

int main(void) {
    // 1. Enable AHB1 clock for GPIOI (Bit 8 = 0x00000100)
    RCC->AHB1ENR |= 0x00000100;

    // 2. Clear Pin 1 (bits [3:2]) and Pin 11 (bits [23:22])
    //    Mask: ~(0x00C0000C) = 0xFF3FFFF3
    GPIOI->MODER &= 0xFF3FFFF3;
    //    Set Pin 1 as general purpose output (01b -> bit 2 = 0x00000004)
    //    Pin 11 remains 00b (Input)
    GPIOI->MODER |= 0x00000004;

    // 3. Set Pin 1 output type to push-pull (clear bit 1)
    GPIOI->OTYPER &= 0xFFFFFFFD;

    // 4. Configure Speed (OSPEEDR)
    //    Clear bits [3:2] and [23:22]
    GPIOI->OSPEEDR &= 0xFF3FFFF3;
    //    Set Pin 1 to medium speed (01b -> bit 2 = 0x00000004)
    GPIOI->OSPEEDR |= 0x00000004;

    // 5. Configure Pull-up/Pull-down (PUPDR)
    //    Clear bits [3:2] and [23:22]
    GPIOI->PUPDR &= 0xFF3FFFF3;
    //    Set Pin 11 pull-up (01b -> bit 22 = 0x00400000)
    GPIOI->PUPDR |= 0x00400000;

    // 6. Superloop
    while (1) {
        // Read Pin 11 (Bit 11 = 0x00000800)
        // Active-low: 0 when pressed, non-zero when released
        if ((GPIOI->IDR & 0x00000800) == 0x00000000) {
            // Button pressed -> Set Pin 1 HIGH (Bit 1 = 0x00000002)
            GPIOI->ODR = 0x00000002;
        } else {
            // Button released -> Reset Pin 1 LOW (Bit 17 = 0x00020000)
            GPIOI->ODR = 0x00020000;
        }
    }
}
