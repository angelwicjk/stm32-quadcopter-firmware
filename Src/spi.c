/*
 * spi.c
 *
 *  Created on: Dec 30, 2025
 *      Author: Bertan
 */

#include "stm32f4xx.h"

#define SPI2_ENR (1U<<14)
#define SPI2_MSTR (1U<<2)

/*Since I enable the GPIOB from my tim.c i will not be enablingn it again
 *
 *
 *
 * We have to set the selected pins ; PB13(SPI2_SKC AF5) PB14 (SPI2_MISO(AF5) and PB15 (SPI2_MOSI(AF5)
 * 1-Enable the Alternate Function -> MODER+
 * 2- AF5 -> AFR+
 * 3- OSPEEDR -> HIGH+
 * 4- OTYPER -> PUSH PULL+
 * 5- > PUPDR none , or pull down the MISO - will select it later on.
 * */


void spi2_gpio_init_pb13_14_15(void)
{
    // Enable GPIOB
    RCC->AHB1ENR |= (1U<<1);

    // Tiny delay to allow clock to settle (good practice)
    __asm("nop");
    __asm("nop");

    // 1. Set CS pins HIGH (Inactive) in ODR *before* making them outputs
    // This prevents a "glitch low" when switching mode
    GPIOB->ODR |= (1U<<6) | (1U<<7);

    // 2. Configure PB6 (CS_AG) & PB7 (CS_M) as Output
    GPIOB->MODER &= ~((3U<<(6*2)) | (3U<<(7*2)));
    GPIOB->MODER |=  ((1U<<(6*2)) | (1U<<(7*2)));

    // 3. Configure PB13, PB14, PB15 for SPI2 (AF5)
    // Alternate Function Mode (10)
    GPIOB->MODER &= ~((3U<<26) | (3U<<28) | (3U<<30));
    GPIOB->MODER |=  ((2U<<26) | (2U<<28) | (2U<<30));

    // AF5 (0101)
    GPIOB->AFR[1] &= ~((0xFU<<20) | (0xFU<<24) | (0xFU<<28));
    GPIOB->AFR[1] |=  ((0x5U<<20) | (0x5U<<24) | (0x5U<<28));

    // High Speed & Push-Pull
    GPIOB->OSPEEDR |= ((3U<<26) | (3U<<28) | (3U<<30));
    GPIOB->OTYPER &= ~((1U<<13) | (1U<<14) | (1U<<15));
    GPIOB->PUPDR &= ~((3U<<26) | (3U<<28) | (3U<<30));
}

void spi2_init(void)
{
    // Enable SPI2 clock
    RCC->APB1ENR |= SPI2_ENR;
    (void)RCC->APB1ENR; // read-back barrier

    // Disable SPI before config
    SPI2->CR1 &= ~(1U<<6);

    // Master
    SPI2->CR1 |= (1U<<2);   // MSTR

    // Software NSS management
    SPI2->CR1 |= (1U<<9);   // SSM
    SPI2->CR1 |= (1U<<8);   // SSI

    // Baud prescaler: /32 (safe bring-up) -> 500MHz
    SPI2->CR1 &= ~(7U<<3);
    SPI2->CR1 |=  (4U<<3);

    // Mode: start with Mode 3 (CPOL=1, CPHA=1) (we will verify)
    // Mode 0: CPOL = 0, CPHA = 0
    SPI2->CR1 |= (1U<<1);  // CPOL = 1
    SPI2->CR1 |= (1U<<0);  // CPHA = 1


    // MSB first (default), 8-bit (DFF=0 default)
    SPI2->CR1 &= ~(1U<<7);  // LSBFIRST=0
    SPI2->CR1 &= ~(1U<<11); // DFF=0

    // Enable SPI
    SPI2->CR1 |= (1U<<6);
}

uint8_t spi_txrx_byte(SPI_TypeDef *SPIx, uint8_t tx)
{
    while(!(SPIx->SR & (1U<<1))) {}                  // TXE
    *((volatile uint8_t*)&SPIx->DR) = tx;

    while(!(SPIx->SR & (1U<<0))) {}                  // RXNE
    uint8_t rx = *((volatile uint8_t*)&SPIx->DR);

    while(SPIx->SR & (1U<<7)) {}                     // BSY
    return rx;
}

void spi2_enable_dma(void)
{
    // SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN
    SPI2->CR2 |= (1U << 1) | (1U << 0);
}
