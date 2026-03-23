/*
 * LSM9DS1.c
 *
 *  Created on: Dec 30, 2025
 *      Author: Bertan
 */
#include "stm32f4xx.h"
#include "spi.h"
#include <stdint.h>

#define WHO_AM_I_REG_AG     0x0F
#define WHO_AM_I_EXPECTED   0x68
#define SPI_READ            0x80
#define SPI_INC    0x40   // later for burst

#define CTRL_REG1_G   0x10
#define CTRL_REG6_XL  0x20
#define CTRL_REG8     0x22
#define OUT_X_L_G 0x18
#define OUT_X_L_XL 0x28


#define CSAG_LOW()   (GPIOB->ODR &= ~(1U<<6))
#define CSAG_HIGH()  (GPIOB->ODR |=  (1U<<6))
#define CSM_HIGH()   (GPIOB->ODR |=  (1U<<7))


extern volatile uint8_t data_ready_flag;
extern uint8_t rx_buffer[6];

uint8_t LSM9DS1_ReadWhoAmI_AG(void)
{
    CSM_HIGH();
    CSAG_LOW();

    // tiny delay (~1 µs)
    for (volatile int i = 0; i < 50; i++);

    spi_txrx_byte(SPI2, SPI_READ | WHO_AM_I_REG_AG);
    uint8_t id = spi_txrx_byte(SPI2, 0xFF);

    CSAG_HIGH();
    return id;
}
void LSM9DS1_WriteReg_AG(uint8_t reg, uint8_t val)
{
    CSM_HIGH();
    CSAG_LOW();
    spi_txrx_byte(SPI2, reg & 0x7F);      // ensure write
    spi_txrx_byte(SPI2, val);
    CSAG_HIGH();
}

uint8_t LSM9DS1_ReadReg_AG(uint8_t reg)
{
    CSM_HIGH();
    CSAG_LOW();
    spi_txrx_byte(SPI2, SPI_READ | (reg & 0x7F)); // read, no inc
    uint8_t v = spi_txrx_byte(SPI2, 0xFF);
    CSAG_HIGH();
    return v;
}
void LSM9DS1_Init_AG(void)
{
    // Enable BDU (Bit 6) and IF_ADD_INC (Bit 2)
    // 0x40 | 0x04 = 0x44
    LSM9DS1_WriteReg_AG(CTRL_REG8, 0x44);

    // Gyro: ODR=476Hz, FS=245 dps
    LSM9DS1_WriteReg_AG(CTRL_REG1_G, 0x60);

    // Accel: ODR=476Hz, FS=±2g
    LSM9DS1_WriteReg_AG(CTRL_REG6_XL, 0x60);
}


int16_t LSM9DS1_ReadGyroX_RAW(void)
{
    uint8_t xl = LSM9DS1_ReadReg_AG(OUT_X_L_G);
    uint8_t xh = LSM9DS1_ReadReg_AG(OUT_X_L_G + 1);
    return (int16_t)((xh<<8) | xl);
}

void LSM9DS1_ReadMulti_AG(uint8_t start_reg, uint8_t *dst, uint16_t len)
{
    CSM_HIGH();
    CSAG_LOW();

    // REMOVED SPI_INC (0x40).
    // Since CTRL_REG8 has IF_ADD_INC=1, the sensor increments automatically.
    spi_txrx_byte(SPI2, SPI_READ | (start_reg & 0x7F));

    for(uint16_t i = 0; i < len; i++)
        dst[i] = spi_txrx_byte(SPI2, 0xFF);

    CSAG_HIGH();
}

void LSM9DS1_ReadGyro_RAW(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t b[6];
    LSM9DS1_ReadMulti_AG(OUT_X_L_G, b, 6);

    *gx = (int16_t)((b[1]<<8) | b[0]);
    *gy = (int16_t)((b[3]<<8) | b[2]);
    *gz = (int16_t)((b[5]<<8) | b[4]);
}
void LSM9DS1_ReadAccel_RAW(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t b[6];
    LSM9DS1_ReadMulti_AG(OUT_X_L_XL, b, 6);

    *ax = (int16_t)((b[1]<<8) | b[0]);
    *ay = (int16_t)((b[3]<<8) | b[2]);
    *az = (int16_t)((b[5]<<8) | b[4]);
}


void LSM9DS1_StartReadGyro_DMA(void)
{
    data_ready_flag = 0; // Reset flag

    CSM_HIGH();
    CSAG_LOW(); // Chip Select Low

    // 1. Send Address (Polling)
    // OUT_X_L_G (0x18) | READ (0x80)
    spi_txrx_byte(SPI2, 0x80 | 0x18);

    // 2. Hand over to DMA
    dma1_stream3_spi_start(0, (uint32_t)rx_buffer, 6);
}

