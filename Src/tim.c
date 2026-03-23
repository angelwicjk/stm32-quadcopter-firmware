#include "stm32f4xx.h"
#include "tim.h"

/*---------- TIMER CONTROL REGISTER MACROS ----------*/
#define TIMx_CENR                (1U<<0)
#define CR1_ARPE                 (1U<<7)
#define EGR_UG                   (1U<<0)

/*---------- CCMR REGISTER MACROS (PWM Configuration) ----------*/
// Channel 1 (OC1)
#define CCMR1_OC1M_PWM1_BIT1     (1U<<6)   // PWM Mode 1: bit 6
#define CCMR1_OC1M_PWM1_BIT2     (1U<<5)   // PWM Mode 1: bit 5
#define CCMR1_OC1PE              (1U<<3)   // Output Compare 1 Preload Enable

// Channel 2 (OC2)
#define CCMR1_OC2M_PWM1_BIT1     (1U<<14)  // PWM Mode 1: bit 14
#define CCMR1_OC2M_PWM1_BIT2     (1U<<13)  // PWM Mode 1: bit 13
#define CCMR1_OC2PE              (1U<<11)  // Output Compare 2 Preload Enable

// Channel 3 (OC3)
#define CCMR2_OC3M_PWM1_BIT1     (1U<<6)   // PWM Mode 1: bit 6
#define CCMR2_OC3M_PWM1_BIT2     (1U<<5)   // PWM Mode 1: bit 5
#define CCMR2_OC3PE              (1U<<3)   // Output Compare 3 Preload Enable

// Channel 4 (OC4)
#define CCMR2_OC4M_PWM1_BIT1     (1U<<14)  // PWM Mode 1: bit 14
#define CCMR2_OC4M_PWM1_BIT2     (1U<<13)  // PWM Mode 1: bit 13
#define CCMR2_OC4PE              (1U<<11)  // Output Compare 4 Preload Enable

/*---------- CCER REGISTER MACROS (Output Enable) ----------*/
#define CCER_CC1E                (1U<<0)   // Capture/Compare 1 Output Enable
#define CCER_CC2E                (1U<<4)   // Capture/Compare 2 Output Enable
#define CCER_CC3E                (1U<<8)   // Capture/Compare 3 Output Enable
#define CCER_CC4E                (1U<<12)  // Capture/Compare 4 Output Enable

/*---------- GPIO CLOCK ENABLE ----------*/
#define GPIOAEN                  (1U<<0)
#define GPIOBEN                  (1U<<1)

/*---------- TIMER CLOCK ENABLE ----------*/
#define APB1ENR_TIM2EN           (1U<<0)
#define APB1ENR_TIM3EN           (1U<<1)
#define APB1ENR_TIM4EN           (1U<<2)
#define APB1ENR_TIM5EN           (1U<<3)

/*---------- GPIO MODER MACROS ----------*/
// PA6 (bits 13:12)
#define MODER_PA6_AF_BIT1        (1U<<13)
#define MODER_PA6_AF_BIT0        (1U<<12)

// PA7 (bits 15:14)
#define MODER_PA7_AF_BIT1        (1U<<15)
#define MODER_PA7_AF_BIT0        (1U<<14)

// PB0 (bits 1:0)
#define MODER_PB0_AF_BIT1        (1U<<1)
#define MODER_PB0_AF_BIT0        (1U<<0)

// PB1 (bits 3:2)
#define MODER_PB1_AF_BIT1        (1U<<3)
#define MODER_PB1_AF_BIT0        (1U<<2)

/*---------- GPIO AFR MACROS ----------*/
// AF2 = 0b0010 for TIM3
#define AFR_AF2_BIT1             (1U<<1)   // Bit 1 of AF value

// PA6 uses AFR[0] bits [27:24]
#define AFR0_PA6_CLEAR           (0xFU<<24)
#define AFR0_PA6_AF2             (AFR_AF2_BIT1<<24)

// PA7 uses AFR[0] bits [31:28]
#define AFR0_PA7_CLEAR           (0xFU<<28)
#define AFR0_PA7_AF2             (AFR_AF2_BIT1<<28)

// PB0 uses AFR[0] bits [3:0]
#define AFR0_PB0_CLEAR           (0xFU<<0)
#define AFR0_PB0_AF2             (AFR_AF2_BIT1<<0)

// PB1 uses AFR[0] bits [7:4]
#define AFR0_PB1_CLEAR           (0xFU<<4)
#define AFR0_PB1_AF2             (AFR_AF2_BIT1<<4)




void tim3_4ch_pwm_init(void) {

    // Enable TIM3 Clock
    RCC->APB1ENR |= APB1ENR_TIM3EN;

    /*Set prescaler value*/
    TIM3->PSC = 16 - 1;     // 1MHz clock

    /*Set AutoReload value*/
    TIM3->ARR = 2500 - 1;  // 400Hz period

    /*Clear Counter*/
    TIM3->CNT = 0;

    // --- Configure Channel 1 (PWM Mode 1) ---
    TIM3->CCMR1 |= CCMR1_OC1M_PWM1_BIT1 | CCMR1_OC1M_PWM1_BIT2;
    TIM3->CCMR1 |= CCMR1_OC1PE;  // Preload Enable

    // --- Configure Channel 2 (PWM Mode 1) ---
    TIM3->CCMR1 |= CCMR1_OC2M_PWM1_BIT1 | CCMR1_OC2M_PWM1_BIT2;
    TIM3->CCMR1 |= CCMR1_OC2PE;  // Preload Enable

    // --- Configure Channel 3 (PWM Mode 1) ---
    TIM3->CCMR2 |= CCMR2_OC3M_PWM1_BIT1 | CCMR2_OC3M_PWM1_BIT2;
    TIM3->CCMR2 |= CCMR2_OC3PE;  // Preload Enable

    // --- Configure Channel 4 (PWM Mode 1) ---
    TIM3->CCMR2 |= CCMR2_OC4M_PWM1_BIT1 | CCMR2_OC4M_PWM1_BIT2;
    TIM3->CCMR2 |= CCMR2_OC4PE;  // Preload Enable

    // --- Enable all 4 Channel Outputs ---
    TIM3->CCER |= CCER_CC1E;  // Channel 1 output enable
    TIM3->CCER |= CCER_CC2E;  // Channel 2 output enable
    TIM3->CCER |= CCER_CC3E;  // Channel 3 output enable
    TIM3->CCER |= CCER_CC4E;  // Channel 4 output enable

    /*Enable auto-reload preload*/
    TIM3->CR1 |= CR1_ARPE;

    /*Enable Update Generation*/
    TIM3->EGR |= EGR_UG;

    /*Enable Timer*/
    TIM3->CR1 |= TIMx_CENR;
}




void enableMotorPWM_Pins_TIM3(void) {

    RCC->AHB1ENR |= GPIOAEN;
    RCC->AHB1ENR |= GPIOBEN;

    // --- Configure PA6 for TIM3_CH1 (AF2) ---
    // Set to Alternate Function Mode (0b10)
    GPIOA->MODER |= MODER_PA6_AF_BIT1;
    GPIOA->MODER &= ~MODER_PA6_AF_BIT0;
    // Set to AF2 (0010) in bits [27:24]
    GPIOA->AFR[0] &= ~AFR0_PA6_CLEAR;  // Clear all 4 bits
    GPIOA->AFR[0] |= AFR0_PA6_AF2;     // Set AF2

    // --- Configure PA7 for TIM3_CH2 (AF2) ---
    // Set to Alternate Function Mode (0b10)
    GPIOA->MODER |= MODER_PA7_AF_BIT1;
    GPIOA->MODER &= ~MODER_PA7_AF_BIT0;
    // Set to AF2 (0010) in bits [31:28]
    GPIOA->AFR[0] &= ~AFR0_PA7_CLEAR;  // Clear all 4 bits
    GPIOA->AFR[0] |= AFR0_PA7_AF2;     // Set AF2

    // --- Configure PB0 for TIM3_CH3 (AF2) ---
    // Set to Alternate Function Mode (0b10)
    GPIOB->MODER |= MODER_PB0_AF_BIT1;
    GPIOB->MODER &= ~MODER_PB0_AF_BIT0;
    // Set to AF2 (0010) in bits [3:0]
    GPIOB->AFR[0] &= ~AFR0_PB0_CLEAR;  // Clear all 4 bits
    GPIOB->AFR[0] |= AFR0_PB0_AF2;     // Set AF2

    // --- Configure PB1 for TIM3_CH4 (AF2) ---
    // Set to Alternate Function Mode (0b10)
    GPIOB->MODER |= MODER_PB1_AF_BIT1;
    GPIOB->MODER &= ~MODER_PB1_AF_BIT0;
    // Set to AF2 (0010) in bits [7:4]
    GPIOB->AFR[0] &= ~AFR0_PB1_CLEAR;  // Clear all 4 bits
    GPIOB->AFR[0] |= AFR0_PB1_AF2;     // Set AF2
}




void tim4_Angle_500hz_interrupt_init(void) {

    // Enable TIM4 Clock
    RCC->APB1ENR |= APB1ENR_TIM4EN;

    // Set Prescaler
    // 16MHz / (160) = 100,000 Hz (0.01ms tick)
    TIM4->PSC = 160 - 1;

    // Set Auto-Reload Register
    // 100,000 Hz / (200) = 500 Hz (2ms period)
    TIM4->ARR = 1000 - 1;

    // Enable Update Interrupt (UIE)
    TIM4->DIER |= (1U << 0);

    // Enable the TIM4 interrupt in the NVIC
    NVIC_EnableIRQ(TIM4_IRQn);

    // Enable Timer
    TIM4->CR1 |= TIMx_CENR;
}


