/* dma.c */
#include "stm32f4xx.h"
#include "dma.h"
#include "spi.h" // Needed for SPI2 definition if not in stm32f4xx.h

// --- 1. ACTUAL VARIABLE ALLOCATION ---
volatile uint8_t data_ready_flag = 0;
uint8_t rx_buffer[6];
uint8_t dummy_byte = 0xFF;

// --- 2. EXTERNAL FUNCTION for CS (We need to raise CS in the ISR) ---
// Simple macro to raise Chip Select (matches your LSM9DS1.c)
#define CSAG_HIGH()  (GPIOB->ODR |=  (1U<<6))

void dma1_init(void)
{
    // Enable DMA1 Clock
    RCC->AHB1ENR |= (1U << 21);

    // --- CONFIG RX STREAM (Stream 3) ---
    DMA1_Stream3->CR &= ~1U; // Disable
    while(DMA1_Stream3->CR & 1U);

    DMA1_Stream3->CR = 0;
    DMA1_Stream3->CR |= (0U << 25); // Channel 0
    DMA1_Stream3->CR |= (1U << 10); // MINC Enable (Memory Increment)
    DMA1_Stream3->CR |= (1U << 4);  // TCIE (Interrupt Enable)

    // --- CONFIG TX STREAM (Stream 4) ---
    DMA1_Stream4->CR &= ~1U; // Disable
    while(DMA1_Stream4->CR & 1U);

    DMA1_Stream4->CR = 0;
    DMA1_Stream4->CR |= (0U << 25); // Channel 0
    DMA1_Stream4->CR |= (1U << 6);  // Dir: Memory to Peripheral
    DMA1_Stream4->CR &= ~(1U << 10); // MINC DISABLE (Always send same byte)

    // Enable Interrupt in NVIC
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    NVIC_SetPriority(DMA1_Stream3_IRQn, 1);
}

void dma1_stream3_spi_start(uint32_t src, uint32_t dst, uint32_t len)
{
    // Clear flags
    DMA1->LIFCR |= (1U << 27) | (1U << 22) | (1U << 21) | (1U << 16);
    DMA1->HIFCR |= (1U << 5) | (1U << 0);

    // Setup Addresses
    DMA1_Stream3->PAR = (uint32_t)&(SPI2->DR);
    DMA1_Stream3->M0AR = dst;
    DMA1_Stream3->NDTR = len;

    DMA1_Stream4->PAR = (uint32_t)&(SPI2->DR);
    DMA1_Stream4->M0AR = (uint32_t)&dummy_byte;
    DMA1_Stream4->NDTR = len;

    // Enable Streams
    DMA1_Stream3->CR |= 1U;
    DMA1_Stream4->CR |= 1U;
}

// --- 3. INTERRUPT HANDLER (Moved Here) ---
void DMA1_Stream3_IRQHandler(void)
{
    // Check Transfer Complete Flag for Stream 3
    if (DMA1->LISR & (1U << 27))
    {
        // Clear flag
        DMA1->LIFCR |= (1U << 27);

        // Deselect Sensor (CS HIGH)
        CSAG_HIGH();

        // Signal Main Loop
        data_ready_flag = 1;
    }
}
