/*
 * i2c.h
 *
 *  Created on: Oct 22, 2025
 *      Author: Bertan
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

void I2C1_init(void);

void I2C1_byteRead(char slave, char reg, char *data);
void I2C1_burstRead(char slave, char reg, int n, char *data);
void I2C1_burstWrite(char slave, char reg, int n, char *data);

/* DMA layer */
void I2C1_DMA_RX_Init(void);

/* Generic async "mem read" */
int I2C1_MemRead_DMA_Start(uint8_t addr7, uint8_t reg, uint8_t *dst, uint16_t len);

uint8_t I2C1_DMA_IsBusy(void);
uint8_t I2C1_DMA_HasNewRx(void);
void I2C1_DMA_ClearNewRx(void);

#endif

