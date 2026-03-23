#include "uart.h"
#define GPIOAEN    		(1U<<0)
#define UART2EN			(1U<<17)


#define CR1_TE			(1U<<3)
#define CR1_UE			(1U<<13)
#define CR1_RE			(1U<<2)


#define SR_TXE			(1U<<7)
#define CR1_RXNEIE		(1U<<5)

#define UART1EN        (1U<<4)   // USART1 is bit 4 on APB2ENR
#define APB2_CLK       SYS_FREQ  // 16MHz (same as system clock)
#define IBUS_BAUDRATE  115200    // iBUS protocol baudrate


#define SYS_FREQ		16000000
#define APB1_CLK		SYS_FREQ


#define UART_BAUDRATE	115200


void uart1_rx_interrupt_init(void);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClock, uint32_t BaudRate );
static uint16_t compute_uart_bd(uint32_t PeriphClock, uint32_t BaudRate);
void uart2_write(int charYouWantToWrite);





void uart2_rx_interrupt_init(void)
{
	/* *** CONFIGURE UART GPIO PIN *** */
	/* Enable clock access to gpioA */
	RCC->AHB1ENR |= GPIOAEN;

	/* Set PA2 mode to alternate function mode */
	GPIOA->MODER |=(1UL<<5); // '1'
	GPIOA->MODER &=~(1UL<<4); // '0'

	/* Set PA2 alternate function type to UART_TX(AF7) */
	GPIOA->AFR[0] &=~(1UL<<11); //'0'
	GPIOA->AFR[0] |=(1UL<<10); //'1'
	GPIOA->AFR[0] |=(1UL<<9); //'1'
	GPIOA->AFR[0] |=(1UL<<8);//'1'

	/* Set PA3 mode to alternate function mode */
	GPIOA->MODER |=(1UL<<7); // '1'
	GPIOA->MODER &=~(1UL<<6); // '0'


	/* Set PA3 alternate function type to UART_TX(AF7) */
	GPIOA->AFR[0] &=~(1UL<<15); //'0'
	GPIOA->AFR[0] |=(1UL<<14); //'1'
	GPIOA->AFR[0] |=(1UL<<13); //'1'
	GPIOA->AFR[0] |=(1UL<<12);//'1'




	/* **** Configure UART Module *** */
	/* Enable clock access to UART2 */
	RCC->APB1ENR |= UART2EN;

	/* Configure baudrate */
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);


	/* Configure the transfer direction */
	USART2->CR1 = (CR1_TE | CR1_RE); // this overwrites all bits to 0, except the one in the desired position (3) => thus all the parameters of the communication are set as per CR1 bits values
	/*enable rxne interrupt*/
	USART2->CR1 |= CR1_RXNEIE; // interrupt mode enabled
	/*Enable UART2  Interrupt in  NVIC*/
	NVIC_EnableIRQ(USART2_IRQn);

	/* Enable the UART module */
	USART2->CR1 |= CR1_UE; // |= so to say, write only that particular bit, and leave the others unchanged cause we already set the bit 3 at the previous line of code
}




void uart1_rx_interrupt_init(void)
{
    /* *** CONFIGURE UART GPIO PIN *** */
    /* Enable clock access to GPIOA (if not already enabled) */
    RCC->AHB1ENR |= GPIOAEN;

    /* Set PA10 mode to alternate function mode (USART1_RX) */
    GPIOA->MODER |= (1UL << 21);   // Bit 21 = '1'
    GPIOA->MODER &= ~(1UL << 20);  // Bit 20 = '0'

    /* Set PA10 alternate function type to USART1_RX (AF7) */
    // PA10 uses AFR[1] (high register), bits [11:8]
    GPIOA->AFR[1] &= ~(1UL << 11); // '0'
    GPIOA->AFR[1] |= (1UL << 10);  // '1'
    GPIOA->AFR[1] |= (1UL << 9);   // '1'
    GPIOA->AFR[1] |= (1UL << 8);   // '1'
    // Result: AF7 = 0b0111

    /* **** Configure UART Module *** */
    /* Enable clock access to USART1 (on APB2, not APB1!) */
    RCC->APB2ENR |= UART1EN;

    /* Configure baudrate (115200 for iBUS) */
    uart_set_baudrate(USART1, APB2_CLK, IBUS_BAUDRATE);

    /* Configure the transfer direction (RX only for receiver) */
    USART1->CR1 = CR1_RE;  // Only enable receiver

    /* Enable RXNE interrupt */
    USART1->CR1 |= CR1_RXNEIE;

    /* Enable USART1 Interrupt in NVIC */
    NVIC_EnableIRQ(USART1_IRQn);

    /* Enable the UART module */
    USART1->CR1 |= CR1_UE;
}

char uart2_read(void){

	/*Make sure recieve data is not empty*/
	while(!(USART2->SR & SR_RXNE ));
	/*Return the data */
	return USART2->DR;


}


void uart2_write(int charYouWantToWrite)
{
	/* Make sure the transmit data register is empty */
	while(!(USART2->SR & SR_TXE ));  //execute this until data is transmitted , then go to the next step

	/* Write to transmit data register */
	USART2->DR = (charYouWantToWrite & 0xFF);
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClock, uint32_t BaudRate )
{
	USARTx->BRR = compute_uart_bd(PeriphClock, BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClock, uint32_t BaudRate)
{
	return ((PeriphClock + (BaudRate/2UL))/BaudRate);
}

int __io_putchar(int myCharacter)
{
	uart2_write(myCharacter);
	return myCharacter;
}



