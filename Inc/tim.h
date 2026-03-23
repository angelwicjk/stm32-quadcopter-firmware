/*
 * tim.h
 *
 *  Created on: Oct 21, 2025
 *      Author: Bertan
 */

#ifndef TIM_H_
#define TIM_H_


void tim3_4ch_pwm_init(void);
void enableMotorPWM_Pins_TIM3(void);
void tim4_Angle_500hz_interrupt_init(void);


#endif /* TIM_H_ */
