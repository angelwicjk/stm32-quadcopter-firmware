# STM32F411RE Quadcopter Flight Controller
### Bare-Metal Cascaded PID | 500 Hz Control Loop | SPI-DMA IMU

A fully bare-metal quadcopter flight controller developed from scratch on the **STM32F411RE** (Nucleo-64), with no HAL or external flight control libraries. All peripheral drivers, sensor interfaces, and control algorithms are implemented at register level.

---

## Features

- **Cascaded PID control** — Outer angle loop + inner rate loop for Roll and Pitch
- **500 Hz control loop** via TIM4 hardware interrupt
- **LSM9DS1 IMU** (Gyroscope + Accelerometer) over SPI with **DMA transfer** (non-blocking)
- **Complementary filter** for attitude estimation (98% gyro / 2% accelerometer)
- **iBUS RC protocol parser** — FlySky FS-i6 / FS-iA6B receiver support
- **4-channel PWM motor output** via TIM3 (1000–2000 µs pulse width)
- **Anti-windup** integral clamping on PID
- **Arm/Disarm logic** with throttle-low safety check
- **UART debug stream** at 20 Hz (motor outputs + estimated angles)

---

## Hardware

| Component | Detail |
|---|---|
| MCU | STM32F411RET6 (Nucleo-64) |
| IMU | LSM9DS1 (Gyro: 476 Hz ODR, 245 dps FS / Accel: 476 Hz ODR, ±2g FS) |
| IMU Interface | SPI2 + DMA1 Stream3 |
| RC Receiver | FlySky FS-iA6B (iBUS protocol via USART1) |
| Motor Output | 4× PWM via TIM3 CH1–CH4 |
| Debug UART | USART2 (ST-Link Virtual COM Port) |

---

## Software Architecture

```
TIM4 IRQ (500 Hz)
    │
    ├─ DMA flag check (LSM9DS1 gyro data ready)
    ├─ Raw gyro conversion → dps
    ├─ Accel read (polling, ~50µs)
    ├─ Complementary Filter → final_pitch_angle, final_roll_angle
    ├─ controlLoop()
    │       ├─ Arm/Disarm state machine
    │       ├─ RC setpoint mapping
    │       ├─ Outer PID (angle)  → rate setpoint
    │       ├─ Inner PID (rate)   → motor correction
    │       └─ mixMotors() → TIM3 PWM
    └─ LSM9DS1_StartReadGyro_DMA() → next DMA transfer
```

---

## PID Tuning Parameters

| Controller | Kp | Ki | Kd | Output Limit |
|---|---|---|---|---|
| Pitch Angle (outer) | 1.70 | 0.10 | 0.200 | ±400 |
| Roll Angle (outer) | 1.70 | 0.10 | 0.200 | ±400 |
| Pitch Rate (inner) | 0.20 | 0.001 | 0.002 | ±250 |
| Roll Rate (inner) | 0.20 | 0.001 | 0.002 | ±250 |

---

## Motor Mixing (X-Frame)

```
      Front
  M1(CW)  M2(CCW)
    \      /
     \    /
    M3(CCW) M4(CW)
      Rear

M1 = throttle + roll - pitch - yaw   (Front Left)
M2 = throttle - roll - pitch + yaw   (Front Right)
M3 = throttle + roll + pitch + yaw   (Rear Left)
M4 = throttle - roll + pitch - yaw   (Rear Right)
```

---

## Project Structure

```
DroneDevelopment_2.4_Cascaded/
├── Src/
│   ├── main.c          # Main loop, control loop, ISR handlers
│   ├── pid.c           # Generic PID controller (init, reset, compute)
│   ├── LSM9DS1.c       # SPI + DMA driver for LSM9DS1 IMU
│   ├── motorControl.c  # PWM motor output abstraction
│   ├── tim.c           # TIM3 (PWM) + TIM4 (500 Hz interrupt) setup
│   ├── i2c.c           # I2C driver (reserved for future use)
│   ├── spi.c           # SPI2 register-level driver
│   ├── dma.c           # DMA1 Stream3 setup for SPI RX
│   ├── uart.c          # USART1 (iBUS) + USART2 (debug) init
│   └── SysTick.c       # 1 ms SysTick timer
├── Inc/
│   ├── pid.h
│   ├── LSM9DS1.h
│   ├── motorControl.h
│   └── ...
├── Startup/
│   └── startup_stm32f411retx.s
└── STM32F411RETX_FLASH.ld
```

---

## Build & Flash

**Toolchain:** STM32CubeIDE (GCC ARM)  
**Debug:** ST-Link/SWD

1. Clone the repository
2. Open in STM32CubeIDE: `File → Import → Existing Projects into Workspace`
3. Build: `Project → Build All`
4. Flash: `Run → Debug` (ST-Link)

---

## Debug Output Format

UART2 streams telemetry at 20 Hz:

```
M1, M2, M3, M4, pitch_angle, roll_angle, pitch_setpoint, roll_setpoint
1050,1050,1050,1050,0.12,-0.34,0.00,0.00
```

---

## Development History

This firmware was developed incrementally, each version building on the previous:

| Version | Milestone |
|---|---|
| 1.2 | MPU6050 + basic PWM |
| 1.4 | MPU6050 with complementary filter |
| 1.5 | Motor calibration routine |
| 1.6 | Controlled throttle mapping |
| 1.8 | Full motor control |
| 1.9 | Single-axis PID (pitch & roll) |
| 2.0 | PID optimization |
| 2.1 | I2C → SPI migration + DMA integration |
| 2.1 | LSM9DS1 IMU swap |
| **2.4** | **Cascaded PID (current)** |

---

## Related Projects

This firmware is the software foundation for the **ICAROS** autonomous UAV project, developed for the TEKNOFEST 2026 UAV competition. ICAROS uses a custom STM32H743VIT6-based 4-layer PCB flight controller designed in Altium Designer.

---

## Author

**Hüseyin Bertan Acar**  
Embedded Systems Engineer  
[LinkedIn](https://www.linkedin.com/in/huseyin-bertan-acar/) · [GitHub](https://github.com/angelwicjk)
