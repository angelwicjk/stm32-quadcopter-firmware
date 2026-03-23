#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    // Tuning Gains
    float Kp;
    float Ki;
    float Kd;

    // Memory (States)
    float integral;
    float prevError;

    // Limits
    float outputLimit;  // Max output (e.g., 400 for motor PWM mixing)
    float integralLimit; // Anti-windup limit (prevents I-term from growing too big)

} PID_TypeDef;

// Function Prototypes
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float limit);
float PID_Compute(PID_TypeDef *pid, float setpoint, float measured, float dt);
void PID_Reset(PID_TypeDef *pid);

#endif
