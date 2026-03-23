/*
 * spi.h
 *
 *  Created on: Dec 30, 2025
 *      Author: Bertan
 */

#ifndef SPI_H_
#define SPI_H_

void spi2_gpio_init_pb13_14_15(void);
void spi_config(void);
uint8_t spi_txrx_byte(SPI_TypeDef *SPIx, uint8_t tx);
void spi2_enable_dma(void);
#endif /* SPI_H_ */
