/*
 * dma.h
 *
 *  Created on: Jan 2, 2026
 *      Author: Bertan
 */

#ifndef DMA_H_
#define DMA_H_


void dma1_init(void);
void dma1_stream3_spi_start(uint32_t src, uint32_t dst, uint32_t len);


#endif /* DMA_H_ */
