/*
 * LSM9DS1.h
 *
 *  Created on: Dec 30, 2025
 *      Author: Bertan
 */

#ifndef LSM9DS1_H_
#define LSM9DS1_H_


uint8_t LSM9DS1_ReadWhoAmI_AG(void);
void LSM9DS1_WriteReg_AG(uint8_t reg, uint8_t val);
uint8_t LSM9DS1_ReadReg_AG(uint8_t reg);
void LSM9DS1_Init_AG(void);
int16_t LSM9DS1_ReadGyroX_RAW(void);
void LSM9DS1_ReadGyro_RAW(int16_t *gx, int16_t *gy, int16_t *gz);
void LSM9DS1_ReadMulti_AG(uint8_t start_reg, uint8_t *dst, uint16_t len);
void LSM9DS1_ReadAccel_RAW(int16_t *ax, int16_t *ay, int16_t *az);
void LSM9DS1_StartReadGyro_DMA(void);
#endif /* LSM9DS1_H_ */
