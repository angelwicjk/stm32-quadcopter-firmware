#include "pid.h"

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float limit) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->outputLimit = limit;
    pid->integralLimit = limit / 2.0f; // Usually safer to cap I-term lower than total output

    PID_Reset(pid);
}

void PID_Reset(PID_TypeDef *pid) {
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

float PID_Compute(PID_TypeDef *pid, float setpoint, float measured, float dt) {

    // 1. Calculate Error
    float error = setpoint - measured;

    // 2. Proportional Term
    float P = pid->Kp * error;

    // 3. Integral Term (Accumulate Error)
    pid->integral += error * dt;

    // --- ANTI-WINDUP (Clamping I-term) ---
    if (pid->integral > pid->integralLimit)  pid->integral = pid->integralLimit;
    if (pid->integral < -pid->integralLimit) pid->integral = -pid->integralLimit;

    float I = pid->Ki * pid->integral;

    // 4. Derivative Term (Rate of Change)
    float derivative = (error - pid->prevError) / dt;
    float D = pid->Kd * derivative;

    // 5. Total Output
    float output = P + I + D;

    // 6. Save Error for next loop
    pid->prevError = error;

    // 7. Final Output Limiting (Saturation)
    if (output > pid->outputLimit)  output = pid->outputLimit;
    if (output < -pid->outputLimit) output = -pid->outputLimit;

    return output;
}
