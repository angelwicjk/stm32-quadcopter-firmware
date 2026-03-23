#include "stm32f4xx.h"
#include <stdint.h>

/* ---------------- GPIO / I2C MACROS ---------------- */
#define GPIOBEN                 (1U<<1)
#define I2C1_APB1ENR            (1U<<21)

#define CPU_CLKMHZ              (1U<<4)   /* 16 MHz (CFGR=0 confirmed) */

#define I2C_100KHZ              80
#define SD_MODE_MAX_RISE_TIME   17

#define I2C_400KHZ              13
#define FS_MODE_MAX_RISE_TIME   6

#define I2C1_CR1_PE             (1U<<0)
#define I2C1_CCR_FastMode       (1U<<15)

#define SR2_BUSY                (1U<<1)
#define CR1_START               (1U<<8)
#define SR1_SB                  (1U<<0)
#define SR1_ADDR                (1U<<1)
#define SR1_TXE                 (1U<<7)
#define CR1_ACK                 (1U<<10)
#define CR1_STOP                (1U<<9)
#define SR1_RXNE                (1U<<6)
#define SR1_BTF                 (1U<<2)

/* ---------------- DMA MACROS (I2C1_RX) ---------------- */
#define DMA1EN                  (1U<<21)      /* RCC->AHB1ENR DMA1EN */

#define DMA_SxCR_EN             (1U<<0)
#define DMA_SxCR_TCIE           (1U<<4)
#define DMA_SxCR_DIR_P2M        (0U<<6)       /* DIR=00 */
#define DMA_SxCR_MINC           (1U<<10)

/* CHSEL bits [27:25] = 001 => Channel 1 */
#define DMA_S0_CHSEL_CH1        (1U<<25)

#define DMA_LISR_TCIF0          (1U<<5)
#define DMA_LIFCR_CTCIF0        (1U<<5)

#define I2C_CR2_DMAEN           (1U<<11)
#define I2C_CR2_LAST            (1U<<12)

/* Forward decl */
static inline void i2c_clear_addr(void);

/* ---------------- DMA state ---------------- */
static volatile uint8_t  i2c1_dma_busy = 0;
static volatile uint8_t  i2c1_dma_new  = 0;

/* Optional: keep last dst/len for debug */
static volatile uint8_t *i2c1_dma_dst  = 0;
static volatile uint16_t i2c1_dma_len  = 0;

/* ---------------- I2C init ---------------- */
/* pinout:
 * PB8 ---> SCL
 * PB9 ---> SDA
 * AF04
 */
void I2C1_init(void)
{
    /* Enable clock access to GPIOB */
    RCC->AHB1ENR |= GPIOBEN;

    /* PB8, PB9 alternate function mode */
    GPIOB->MODER &= ~(3U<<16);
    GPIOB->MODER |=  (2U<<16);
    GPIOB->MODER &= ~(3U<<18);
    GPIOB->MODER |=  (2U<<18);

    /* Open-drain */
    GPIOB->OTYPER |= (1U<<8);
    GPIOB->OTYPER |= (1U<<9);

    /* Pull-up (internal) - works, but external pull-ups are usually better for I2C */
    GPIOB->PUPDR &= ~(3U<<16);
    GPIOB->PUPDR |=  (1U<<16);
    GPIOB->PUPDR &= ~(3U<<18);
    GPIOB->PUPDR |=  (1U<<18);

    /* AF4 for PB8/PB9 */
    GPIOB->AFR[1] &= ~(0xFU<<0);   /* PB8 -> AFRH[3:0] */
    GPIOB->AFR[1] |=  (0x4U<<0);
    GPIOB->AFR[1] &= ~(0xFU<<4);   /* PB9 -> AFRH[7:4] */
    GPIOB->AFR[1] |=  (0x4U<<4);

    /* Enable I2C1 clock */
    RCC->APB1ENR |= I2C1_APB1ENR;

    /* Reset I2C1 */
    I2C1->CR1 |= (1U<<15);
    I2C1->CR1 &= ~(1U<<15);

    /* PCLK1 = 16 MHz => CR2.FREQ = 16 */
    I2C1->CR2 = 16U;

    /* Fast-mode, CCR ~ 13 for 16MHz/400kHz, Duty=2 */
    I2C1->CCR = I2C1_CCR_FastMode | I2C_400KHZ;

    /* TRISE for fast mode: 16MHz => ~6 */
    I2C1->TRISE = FS_MODE_MAX_RISE_TIME;

    /* Enable I2C1 */
    I2C1->CR1 |= I2C1_CR1_PE;
}

/* ---------------- Polling I2C funcs (unchanged logic) ---------------- */

void I2C1_byteRead(char SlaveAdress, char MemmoryAdress, char *Data)
{
    volatile int tmp;

    while(I2C1->SR2 & (SR2_BUSY)) {}

    I2C1->CR1 |= CR1_START;
    while(!(I2C1->SR1 & (SR1_SB))) {}

    I2C1->DR = (uint8_t)(SlaveAdress << 1);

    while(!(I2C1->SR1 & (SR1_ADDR))) {}
    tmp  = I2C1->SR2;

    I2C1->DR = (uint8_t)MemmoryAdress;
    while(!(I2C1->SR1 & SR1_TXE)) {}

    I2C1->CR1 |= CR1_START;
    while(!(I2C1->SR1 & (SR1_SB))) {}

    I2C1->DR = (uint8_t)((SlaveAdress << 1) | 1U);
    while(!(I2C1->SR1 & (SR1_ADDR))) {}

    I2C1->CR1 &= ~CR1_ACK;

    tmp = I2C1->SR2;

    I2C1->CR1 |= CR1_STOP;

    while(!(I2C1->SR1 & SR1_RXNE)) {}

    *Data++ = (char)I2C1->DR;
}

void I2C1_burstRead(char SlaveAdress, char MemmoryAdress,int n ,char *Data)
{
    volatile int tmp;

    while(I2C1->SR2 & (SR2_BUSY)) {}

    I2C1->CR1 |= CR1_START;
    while(!(I2C1->SR1 & (SR1_SB))) {}

    I2C1->DR = (uint8_t)(SlaveAdress << 1);
    while(!(I2C1->SR1 & (SR1_ADDR))) {}
    tmp  = I2C1->SR2;

    I2C1->DR = (uint8_t)MemmoryAdress;
    while(!(I2C1->SR1 & SR1_TXE)) {}

    I2C1->CR1 |= CR1_START;
    while(!(I2C1->SR1 & (SR1_SB))) {}

    I2C1->DR = (uint8_t)((SlaveAdress << 1) | 1U);
    while(!(I2C1->SR1 & (SR1_ADDR))) {}

    tmp = I2C1->SR2;

    I2C1->CR1 |= CR1_ACK;

    while(n > 0U)
    {
        if(n == 1U)
        {
            I2C1->CR1 &= ~CR1_ACK;
            I2C1->CR1 |= CR1_STOP;

            while(!(I2C1->SR1 & SR1_RXNE)) {}
            (*Data++) = (char)I2C1->DR;
            break;
        }
        else
        {
            while(!(I2C1->SR1 & SR1_RXNE)) {}
            (*Data++) = (char)I2C1->DR;
            n--;
        }
    }
}

void I2C1_burstWrite(char SlaveAdress, char MemmoryAdress,int n ,char *Data)
{
    volatile int tmp;

    while(I2C1->SR2 & (SR2_BUSY)) {}

    I2C1->CR1 |= CR1_START;
    while(!(I2C1->SR1 & (SR1_SB))) {}

    I2C1->DR = (uint8_t)(SlaveAdress << 1);
    while(!(I2C1->SR1 & (SR1_ADDR))) {}
    tmp  = I2C1->SR2;

    while(!(I2C1->SR1 & (SR1_TXE))) {}

    I2C1->DR = (uint8_t)MemmoryAdress;

    for(int i = 0; i < n; i++)
    {
        while(!(I2C1->SR1 & (SR1_TXE))) {}
        I2C1->DR = (uint8_t)(*Data++);
    }

    while(!(I2C1->SR1 & (SR1_BTF))) {}

    I2C1->CR1 |= CR1_STOP;
}

/* ---------------- DMA init ---------------- */

void I2C1_DMA_RX_Init(void)
{
    RCC->AHB1ENR |= DMA1EN;

    /* Disable Stream0 */
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream0->CR & DMA_SxCR_EN) {}

    /* Peripheral address = I2C1->DR */
    DMA1_Stream0->PAR = (uint32_t)&I2C1->DR;

    /* Configure: CH1, P2M, MINC, TCIE */
    DMA1_Stream0->CR = 0;
    DMA1_Stream0->CR |= DMA_S0_CHSEL_CH1;
    DMA1_Stream0->CR |= DMA_SxCR_DIR_P2M;
    DMA1_Stream0->CR |= DMA_SxCR_MINC;
    DMA1_Stream0->CR |= DMA_SxCR_TCIE;

    /* Clear pending IRQ flags (just in case) */
    DMA1->LIFCR = DMA_LIFCR_CTCIF0;

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

/* ---------------- DMA generic mem-read start ---------------- */
/* Returns 0 if started, -1 if busy, -2 if bad args */
int I2C1_MemRead_DMA_Start(uint8_t addr7, uint8_t reg, uint8_t *dst, uint16_t len)
{
    if (dst == 0 || len == 0) return -2;
    if (i2c1_dma_busy) return -1;

    i2c1_dma_busy = 1;
    i2c1_dma_new  = 0;

    i2c1_dma_dst = dst;
    i2c1_dma_len = len;

    /* -------- Phase A: write register pointer -------- */
    while (I2C1->SR2 & SR2_BUSY) {}

    I2C1->CR1 |= CR1_START;
    while (!(I2C1->SR1 & SR1_SB)) {}

    I2C1->DR = (uint8_t)(addr7 << 1);              /* write */
    while (!(I2C1->SR1 & SR1_ADDR)) {}
    i2c_clear_addr();

    I2C1->DR = reg;
    while (!(I2C1->SR1 & SR1_TXE)) {}

    /* -------- Phase B: repeated start + read -------- */
    I2C1->CR1 |= CR1_START;
    while (!(I2C1->SR1 & SR1_SB)) {}

    I2C1->DR = (uint8_t)((addr7 << 1) | 1U);       /* read */
    while (!(I2C1->SR1 & SR1_ADDR)) {}

    /* ACK yönetimi READ fazında yapılır */
    if (len > 1) {
        I2C1->CR1 |= CR1_ACK;
    } else {
        I2C1->CR1 &= ~CR1_ACK;
    }

    /* -------- Prepare DMA -------- */
    DMA1_Stream0->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream0->CR & DMA_SxCR_EN) {}

    /* Clear any pending flags for Stream0 */
    DMA1->LIFCR = DMA_LIFCR_CTCIF0;   // en az TC temizle (istersen diğerlerini de temizlersin)

    DMA1_Stream0->M0AR = (uint32_t)dst;
    DMA1_Stream0->NDTR = len;

    /* Enable I2C DMA requests + LAST */
    I2C1->CR2 |= I2C_CR2_DMAEN;
    I2C1->CR2 |= I2C_CR2_LAST;

    /* Clear ADDR AFTER DMA is ready */
    i2c_clear_addr();

    /* Start DMA */
    DMA1_Stream0->CR |= DMA_SxCR_EN;

    return 0;
}

/* ---------------- DMA status helpers ---------------- */

uint8_t I2C1_DMA_IsBusy(void)
{
    return (uint8_t)i2c1_dma_busy;
}

uint8_t I2C1_DMA_HasNewRx(void)
{
    return (uint8_t)i2c1_dma_new;
}

void I2C1_DMA_ClearNewRx(void)
{
    i2c1_dma_new = 0;
}

/* ---------------- DMA ISR ---------------- */

void DMA1_Stream0_IRQHandler(void)
{
    if (DMA1->LISR & DMA_LISR_TCIF0)
    {
        /* Clear TC flag */
        DMA1->LIFCR = DMA_LIFCR_CTCIF0;

        /* Generate STOP to end reception */
        I2C1->CR1 |= CR1_STOP;

        /* Disable DMA stream */
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;

        /* Disable I2C DMA requests */
        I2C1->CR2 &= ~I2C_CR2_DMAEN;
        I2C1->CR2 &= ~I2C_CR2_LAST;

        i2c1_dma_new  = 1;
        i2c1_dma_busy = 0;
    }
}

/* ---------------- helpers ---------------- */

static inline void i2c_clear_addr(void)
{
    volatile uint32_t tmp = I2C1->SR2;
    (void)tmp;
}
