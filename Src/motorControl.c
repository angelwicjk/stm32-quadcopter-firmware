#include "stm32f4xx.h"
#include "tim.h"


void motor1SetSpeed(uint16_t motorspeed){

    if(motorspeed < 1000) motorspeed = 1000;
    if(motorspeed > 2000) motorspeed = 2000;

    TIM3->CCR1 = motorspeed;

}
void motor2SetSpeed(uint16_t motorspeed){


    if(motorspeed < 1000) motorspeed = 1000;
    if(motorspeed > 2000) motorspeed = 2000;

    TIM3->CCR2 = motorspeed;

}
void motor3SetSpeed(uint16_t motorspeed){


    if(motorspeed < 1000) motorspeed = 1000;
    if(motorspeed > 2000) motorspeed = 2000;

    TIM3->CCR3 = motorspeed;

}
void motor4SetSpeed(uint16_t motorspeed){


    if(motorspeed < 1000) motorspeed = 1000;
    if(motorspeed > 2000) motorspeed = 2000;

    TIM3->CCR4 = motorspeed;

}
