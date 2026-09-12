#define GPIOI_BASE 					0x40022000UL
#define RCC_BASE					0x40023800UL

// GPIO register offsets
#define GPIO_MODER_OFFSET			0x00UL
#define GPIO_OTYPER_OFFSET			0x04UL
#define GPIO_OSPEEDR_OFFSET			0x08UL
#define GPIO_PUPDR_OFFSET			0x0CUL
#define GPIO_IDR_BASE				0x10UL
#define GPIO_ODR_OFFSET				0x14UL

//rcc REGISTERS
#define RCC_AHB1ENR_OFFSET			0x30UL

// register addresses
#define RCC_AHB1ENR			(*((volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET)))

//#define GPIOI_MODER			(*((volatile uint32_t *)(GPIOI_BASE + GPIO_MODER_OFFSET)))
//
//#define GPIOI_OTPER			(*((volatile uint32_t *)(GPIOI_BASE + GPIO_OTYPER_OFFSET)))
//#define GPIOI_OSPEEDR		(*((volatile uint32_t *)(GPIOI_BASE + GPIO_OSTPEEDR_OFFSET)))
//#define GPIOI_PUPDR			(*((volatile uint32_t *)(GPIOI_BASE + GPIO_PUPDR_OFFSET)))
//#define GPIOI_IDR			(*((volatile uint32_t *)(GPIOI_BASE + GPIO_IRD_OFFSET)))
//#define GPIOI_ODR			(*((volatile uint32_t *)(GPIOI_BASE + GPIO_ODR_OFFSET)))

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} GPIO_TypeDef;

#define GPIOI		((GPIO_TypeDef *)GPIOI_BASE)

