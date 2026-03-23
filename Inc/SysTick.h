/*
 * SysTick.h
 *
 *  Created on: Nov 7, 2025
 *      Author: Bertan
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include <stdint.h>

void SysTickInit(void);           // Initialize SysTick timer
uint32_t SysTickGetMillis(void);  // Get milliseconds since startup
void SysTickDelayMS(int delay);   // Blocking delay

#endif /* SYSTICK_H_ */
