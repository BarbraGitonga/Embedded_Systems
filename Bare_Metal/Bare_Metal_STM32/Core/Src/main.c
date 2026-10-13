#include "stm32f746xx.h"

volatile uint8_t Test = 0;

int main(){
	RCC->AHB1ENR |= 0x00000100; // enable port I(bit 8) of the clock
	// Clear bits [3:2] first, then set bit 2 (01 = General purpose output)
	GPIOI->MODER &= ~0x0000000C;
	GPIOI->MODER |=  0x00000004;

	// Optional: Output speed to medium (01)
	GPIOI->OSPEEDR &= ~0x0000000C;
	GPIOI->OSPEEDR |=  0x00000004;

	while(1){
		if (Test == 0){
			GPIOI->ODR &=~ 0x00000002; // sets pin 1 of port I to 0 to turn OFF
		}
		else{
			GPIOI->ODR |= 0x00000002; // sets pin 1 of port I to 1 to turn ON
		}
	}
}
