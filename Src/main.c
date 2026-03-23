#include "stm32f4xx.h"
#include "uart.h"
#include "tim.h"
#include "SysTick.h"
#include "motorControl.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pid.h"
#include "spi.h"
#include "LSM9DS1.h"
#include "dma.h"
/*------MACROS--------*/
#define GPIOAEN            (1U<<0)
#define	SCALE_FACTOR250_DPS		131.0
#define dt (1.0 / 500.0)
#define LSM9DS1_GYRO_SCALE   0.00875f
#define LSM9DS1_ACCEL_SCALE  0.000061f

/*------MACROS--------*/

/*-------------FUNCTION PROTOTYPES-------------*/
void systemPreperation(void);
void iBUS_ParsePacket(void);
void controlLoop(void);
void mixMotors(uint16_t throttle, int16_t roll, int16_t pitch, int16_t yaw);
/*-------------FUNCTION PROTOTYPES-------------*/

/*---------------VARIABLES---------------------*/
double final_roll_angle = 0.0;
double final_pitch_angle = 0.0;
double final_yaw_rate = 0.0;
char SystemArmed = 0;


volatile float gyro_x_rate,gyro_y_rate,gyro_z_rate;


volatile uint8_t arm_event = 0;
volatile uint8_t disarm_event = 0;

extern volatile uint8_t data_ready_flag;
extern uint8_t rx_buffer[6];


/*--- NEW: GLOBAL MOTOR VARIABLES FOR DEBUGGING ---*/
volatile int16_t global_motor1 = 1050;
volatile int16_t global_motor2 = 1050;
volatile int16_t global_motor3 = 1050;
volatile int16_t global_motor4 = 1050;

/*--------------PID Variables----------------*/
float roll_setpoint = 0.0f;
float pitch_setpoint = 0.0f;
float yaw_setpoint = 0.0f;

PID_TypeDef pid_roll_angle;
PID_TypeDef pid_pitch_angle;

PID_TypeDef pid_roll_rate;
PID_TypeDef pid_pitch_rate;

// RC Channel mapping
#define RC_ROLL_CH         0
#define RC_PITCH_CH        1
#define RC_THROTTLE_CH     2
#define RC_YAW_CH          3
#define RC_ARM_CH          4
#define RC_ARM_THRESHOLD   1500


// iBUS Variables
#define IBUS_PACKET_SIZE 32
static uint8_t ibus_buffer[IBUS_PACKET_SIZE];
static uint8_t ibus_index = 0;
static volatile uint8_t ibus_packet_ready = 0;
uint16_t rc_channels[10] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500, 1500, 1500};
/*---------------VARIABLES END---------------------*/

int main(void)
{
    systemPreperation();

    // Check connection (Safety check)
    if (LSM9DS1_ReadWhoAmI_AG() != 0x68) {
        while(1) { printf("Sensor Fail\n"); }
    }

    // Kickstart the first Read
    LSM9DS1_StartReadGyro_DMA();

    uint32_t last_print = 0;

    while(1)
    {
        // RC Handling (Non-blocking)
        iBUS_ParsePacket();

        // Print Loop (Display @ 20Hz)
        if((SysTickGetMillis() - last_print) > 50) {

            // Only print if armed or debugging
             printf("%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f\r\n",
                   global_motor1, global_motor2,
                   global_motor3, global_motor4,
                   final_pitch_angle, final_roll_angle,pitch_setpoint, roll_setpoint);

        	//printf("%.2f, %.2f, %.2f\r\n", final_pitch_angle, final_roll_angle, final_yaw_rate);

            if (arm_event) {
                printf("*** MOTORS ARMED ***\r\n");
                arm_event = 0;
            }
            if (disarm_event) {
                printf("*** MOTORS DISARMED ***\r\n");
                disarm_event = 0;
            }
            last_print = SysTickGetMillis();
        }
    }
}

void mixMotors(uint16_t throttle, int16_t roll, int16_t pitch, int16_t yaw){

    // Calculate mix
    int32_t m1 = throttle + roll - pitch - yaw;  // Front Left (CW)
    int32_t m2 = throttle - roll - pitch + yaw;  // Front Right (CCW)
    int32_t m3 = throttle + roll + pitch + yaw;  // Rear Left (CCW)
    int32_t m4 = throttle - roll + pitch - yaw;  // Rear Right (CW)

    // Clamp values to keep them safe for motors
    if(m1 < 1050) m1 = 1050; if(m1 > 2000) m1 = 2000;
    if(m2 < 1050) m2 = 1050; if(m2 > 2000) m2 = 2000;
    if(m3 < 1050) m3 = 1050; if(m3 > 2000) m3 = 2000;
    if(m4 < 1050) m4 = 1050; if(m4 > 2000) m4 = 2000;

    // Save to Globals so Main can see them
    global_motor1 = (int16_t)m1;
    global_motor2 = (int16_t)m2;
    global_motor3 = (int16_t)m3;
    global_motor4 = (int16_t)m4;

    // Send to Hardware
    motor1SetSpeed(global_motor1);
    motor2SetSpeed(global_motor2);
    motor3SetSpeed(global_motor3);
    motor4SetSpeed(global_motor4);
}

void controlLoop(void)
{
    static uint8_t last_arm_state = 0;
    uint8_t current_arm_state = (rc_channels[RC_ARM_CH] > RC_ARM_THRESHOLD) ? 1 : 0;

    // --- ARM/DISARM LOGIC ---
    if (current_arm_state && !last_arm_state) {
        // Rising edge → ARM
        if (!SystemArmed && rc_channels[RC_THROTTLE_CH] < 1050) {
            SystemArmed = 1;
            PID_Reset(&pid_pitch_angle);
            PID_Reset(&pid_pitch_rate);
            PID_Reset(&pid_roll_angle);
            PID_Reset(&pid_roll_rate);

            arm_event = 1;
        }
    }
    else if (!current_arm_state && last_arm_state) {
        // Falling edge → DISARM
        if (SystemArmed) {
            SystemArmed = 0;
            disarm_event = 1;
        }
    }

    last_arm_state = current_arm_state;


    // --- 1. ALWAYS CALCULATE SETPOINTS ---
    roll_setpoint  = (rc_channels[RC_ROLL_CH]  - 1500) / 10.0f;
    pitch_setpoint = (rc_channels[RC_PITCH_CH] - 1500) / 10.0f;
    yaw_setpoint   = (rc_channels[RC_YAW_CH]   - 1500) / 5.0f;

    // Calculate PID output for Pitch (using your new PID function)
    double pitch_angle_output = PID_Compute(&pid_pitch_angle, pitch_setpoint, -final_pitch_angle, dt);
    double roll_angle_output = PID_Compute(&pid_roll_angle, roll_setpoint, final_roll_angle, dt);


    double pitch_angleRate_output = PID_Compute(&pid_pitch_rate,pitch_angle_output, -gyro_x_rate,dt);
    double roll_angleRate_output = PID_Compute(&pid_roll_rate,roll_angle_output, gyro_y_rate,dt);

    /*clamp output*/
    if (pitch_angleRate_output > 150) pitch_angleRate_output = 150;
    if (pitch_angleRate_output < -150) pitch_angleRate_output = -150;

    if (roll_angleRate_output > 150) roll_angleRate_output = 150;
    if (roll_angleRate_output < -150) roll_angleRate_output = -150;




    // --- 2. APPLY TO MOTORS -> now we are going to feed the output of this to the 2nd PID which is rate. ---
    if(SystemArmed) {
        uint16_t throttle = rc_channels[RC_THROTTLE_CH];
        if(throttle < 1050) throttle = 1050;
        if(throttle > 2000) throttle = 2000;

        // Passing: Throttle, Roll(Direct), Pitch(PID), Yaw(Direct)
        mixMotors(throttle,
				roll_angleRate_output,
				pitch_angleRate_output,
				(int16_t)yaw_setpoint);
    } else {
        // Update globals even when off so you see "1000" in the logs
        mixMotors(1050, 0, 0, 0);
    }
}

// ... (Keep systemPreperation, CalculateAngles, TIM4_IRQHandler, USART1_IRQHandler, iBUS_ParsePacket exactly as they were) ...
// Copy them here from your previous code
void systemPreperation(void){
    RCC->AHB1ENR |= GPIOAEN;
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    SysTickInit();
    uart2_rx_interrupt_init();
    uart1_rx_interrupt_init();

    printf("Starting __init__...\r\n");




    spi2_gpio_init_pb13_14_15();
     spi2_init();
     dma1_init();
     spi2_enable_dma();
     LSM9DS1_Init_AG();

    tim3_4ch_pwm_init();
    enableMotorPWM_Pins_TIM3();

   NVIC_SetPriority(USART1_IRQn, 0);
   NVIC_SetPriority(USART2_IRQn, 1);
   NVIC_SetPriority(TIM4_IRQn, 3);

    tim4_Angle_500hz_interrupt_init();

    PID_Init(&pid_pitch_angle, 1.7f, 0.1f, 0.2f, 400.0f); //outer pid pitch
    PID_Init(&pid_roll_angle, 1.7f, 0.1f, 0.2f, 400.0f);	//outer pid roll

    PID_Init(&pid_pitch_rate, 0.2f, 0.001f, 0.002f, 250.0f);
    PID_Init(&pid_roll_rate,  0.2f, 0.001f, 0.002f, 250.0f);


   motor1SetSpeed(1000);
   motor2SetSpeed(1000);
   motor3SetSpeed(1000);
   motor4SetSpeed(1000);

    printf("__Init__ complete!\r\n");
    SystemArmed = 0;
}

void CalculateAngles(int16_t *XAccel, int16_t *YAccel, int16_t *ZAccel, double *pitch_angle, double *roll_angle){
    *roll_angle = atan2(*YAccel, *ZAccel) * (180.0 / 3.14159);
    *pitch_angle = atan2(-(*XAccel), sqrt(*YAccel * *YAccel + *ZAccel * *ZAccel)) * (180.0 / 3.14159);
}

void TIM4_IRQHandler(void) {
    // Clear Interrupt Flag
    TIM4->SR &= ~(1U << 0);

    // 1. Check if Gyro Data is Ready (DMA finished?)
    if (data_ready_flag == 1)
    {
        data_ready_flag = 0; // Clear flag immediately

        // --- A. Process Raw Gyro ---
        // Defined LOCALLY here because Main doesn't need them
        int16_t gyro_x = (int16_t)((rx_buffer[1] << 8) | rx_buffer[0]);
        int16_t gyro_y = (int16_t)((rx_buffer[3] << 8) | rx_buffer[2]);
        int16_t gyro_z = (int16_t)((rx_buffer[5] << 8) | rx_buffer[4]);

        // Convert to dps
        gyro_x_rate = (double)gyro_x * LSM9DS1_GYRO_SCALE;
        gyro_y_rate = (double)gyro_y * LSM9DS1_GYRO_SCALE;
        gyro_z_rate = (double)gyro_z * LSM9DS1_GYRO_SCALE;

        // --- B. Read Accel (Polling) ---
        // It's fast enough (~50us) to do inside ISR
        int16_t accel_x, accel_y, accel_z;
        LSM9DS1_ReadAccel_RAW(&accel_x, &accel_y, &accel_z);

        // --- C. Calculate Angles ---
        double accel_pitch = atan2((double)accel_y, (double)accel_z) * (180.0 / 3.14159);
        double accel_roll  = atan2(-(double)accel_x, sqrt((double)accel_y*accel_y + (double)accel_z*accel_z)) * (180.0 / 3.14159);

        // Complementary Filter
        final_pitch_angle = 0.98 * (final_pitch_angle + gyro_x_rate * dt) + 0.02 * accel_pitch;
        final_roll_angle  = 0.98 * (final_roll_angle  + gyro_y_rate * dt) + 0.02 * accel_roll;
        final_yaw_rate    = 0.9  * final_yaw_rate + 0.1 * gyro_z_rate;

       // printf("%d,%d,%d\n\r,",final_pitch_angle,final_roll_angle,final_yaw_rate);

        // --- D. Run PID ---
        controlLoop();

        // --- E. Start NEXT Read ---
        // This ensures data is ready for the NEXT Timer interrupt (2ms later)
        LSM9DS1_StartReadGyro_DMA();
    }
}

void USART1_IRQHandler(void)
{
    if(USART1->SR & SR_RXNE)
    {
        uint8_t received_byte = USART1->DR;
        if(ibus_index == 0 && received_byte != 0x20) return;
        if(ibus_index == 1 && received_byte != 0x40) { ibus_index = 0; return; }
        ibus_buffer[ibus_index++] = received_byte;
        if(ibus_index >= IBUS_PACKET_SIZE) {
            ibus_packet_ready = 1;
            ibus_index = 0;
        }
    }
}

void iBUS_ParsePacket(void)
{
    if(!ibus_packet_ready) return;
    uint16_t checksum = 0xFFFF;
    for(int i = 0; i < 30; i++) checksum -= ibus_buffer[i];
    uint16_t packet_checksum = (ibus_buffer[31] << 8) | ibus_buffer[30];
    if(checksum != packet_checksum) { ibus_packet_ready = 0; return; }
    for(int i = 0; i < 10; i++) rc_channels[i] = (ibus_buffer[2*i + 3] << 8) | ibus_buffer[2*i + 2];
    ibus_packet_ready = 0;
}
