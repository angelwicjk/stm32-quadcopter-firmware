#include "stm32f4xx.h"
#include <stdint.h>

#define CYCLES_PER_MS      16000
#define CTRL_ENABLE        (1U<<0)
#define CTRL_CLKSRC        (1U<<2)
#define CTRL_COUNTFLAG     (1U<<16)
#define CTRL_TICKINT       (1U<<1)  // ADD THIS - Enable interrupt

// Global millisecond counter
static volatile uint32_t systick_millis = 0;

// Initialize SysTick for 1ms interrupts
void SysTickInit(void) {
    SysTick->LOAD = CYCLES_PER_MS - 1;  // 1ms period
    SysTick->VAL = 0;                    // Clear current value
    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC | CTRL_TICKINT;  // Enable with interrupt

    systick_millis = 0;  // Reset counter
}

// Get current milliseconds
uint32_t SysTickGetMillis(void) {
    return systick_millis;
}

// SysTick interrupt handler (increments every 1ms)
void SysTick_Handler(void) {
    systick_millis++;
}

// Your existing delay function (blocking)
void SysTickDelayMS(int delay) {
    SysTick->LOAD = CYCLES_PER_MS - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

    for(int i = 0; i < delay; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0) {}
    }

    SysTick->CTRL = 0;
}
