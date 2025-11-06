/* --Assignment Details--
 * William Miller
 * ECGR4101
 * Project 4 - SPI LCD Controller
 *
 * --Hardware--
 * Microcontroller: STM32F091RC
 * Expansion Board: X-NUCLEO-GFX01M2
 * LCD Controller: ST7789V
 *
 * --Project Goal--
 * Create a program to interface with a LCD screen using Serial Peripheral Interface (SPI).
 * Upon resetting the program, the microcontroller should write the color red on every pixel.
 * */

//------------------------------------------------
/* LIBRARIES */
#include "main.h"
#include "stm32f0xx.h"


//------------------------------------------------
/* MACROS */
#define LCD_WIDTH 240
#define LCD_HEIGHT 320


//------------------------------------------------
/* FUNCTION DECLARATIONS */

void spi1_init(void);
void lcd_gpio_init(void);
static void spi_send8(uint8_t data);
void lcd_send_command(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init(void);


//------------------------------------------------
/* MAIN() */

int main(void) {

	lcd_init();

	lcd_send_command(0x2C);
	GPIOA->BSRR = (1u << (9 + 16)); // Write CS pin low
	GPIOB->BSRR = (1u << 10);		// Write DC pin high
	for (uint32_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
	    spi_send8(0xF8);
	    spi_send8(0x00);
	}
	GPIOA->BSRR = (1u << 9); // Drive CS pin high

	while (1) { }

}


//------------------------------------------------
/* FUNCTION DEFINITIONS */

void spi1_init(void) {
	// Enable SPI1 and GPIOA clocks
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

	// Clear PA5 and PA7
	GPIOA->MODER &= ~(3u << (5 * 2));
	GPIOA->MODER &= ~(3u << (7 * 2));

	// Configure PA5 (SCK) and PA7(MOSI) as GPIO alternate function
	GPIOA->MODER |= (2u << (5 * 2));
	GPIOA->MODER |= (2u << (7 * 2));

	// PA5 as SCK
	GPIOA->AFR[1] &= ~(0xF << (5 * 4));

	// PA7 as MOSI
	GPIOA->AFR[1] &= ~(0xF << (7 * 4));

	// Disable SPI1 for config
	SPI1->CR1 &= ~(1u << (6 * 1));

	// Config SPI Control REG 1 for Master, Software Slave Management, internal NSS high, and moderate speed clock prescaler
	SPI1->CR1 |= (1u << (2 * 1)); // Master
	SPI1->CR1 |= (1u << (9 * 1)); // Software Slave Management
	SPI1->CR1 |= (1u << (8 * 1)); // Internal NSS high (SSI)
	SPI1->CR1 &= ~(7 << (3 * 1)); // Clear BR to set f_PCLK / 4
	SPI1->CR1 |= (1u << (3 * 1)); // Set BR to f_PCLK / 4

	// Config SPI Control REG 2 for 8-bit Frame Size
	SPI1->CR2 &= ~(0xF << (8 * 1));	// Clear DS for 8-bit frame
	SPI1->CR2 |= (0x7 << (8 * 1)); // Set DS for 8-bit frame (Need 0111 in bits 11:8)

	// Enable SPI1
	SPI1->CR1 |= (1u << (6 * 1));
}


void lcd_gpio_init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

	/* PA1: LCD_RST
	 * PA8: Backlight
	 * PA9: Chip Select
	 * PB10: LCD_DC
	 */

	// PA1 Output
	GPIOA->MODER &= ~(3u << (1 * 2));
	GPIOA->MODER |=  (1u << (1 * 2));

	// PA8 Output
	GPIOA->MODER &= ~(3u << (8 * 2));
	GPIOA->MODER |=  (1u << (8 * 2));

	// PA9 Output
	GPIOA->MODER &= ~(3u << (9 * 2));
	GPIOA->MODER |=  (1u << (9 * 2));

	// PB10 Output
	GPIOB->MODER &= ~(3u << (10 * 2));
	GPIOB->MODER |=  (1u << (10 * 2));

}


static void spi_send8(uint8_t data) {

	// Check TXE status; wait if buffer not empty
	while((SPI1->SR & (1u << (1 * 1))) == 0) { /* Do nothing */ }

	*((uint8_t*)&SPI1->DR) = data;

	// Check BSY status; wait if buffer not empty
	while((SPI1->SR & (1u << (7 * 1))) != 0) { /* Do nothing */ }
}


// BSRR write-only so have to use =. Using |= would require reading the current value.
void lcd_send_command(uint8_t cmd) {

    // Write DC and CS low
	GPIOB->BSRR = (1u << (10 + 16)); // PB10 DC
	GPIOA->BSRR = (1u << (9 + 16)); // PA9 CS

    spi_send8(cmd);

    // Set CS back high
    GPIOA->BSRR = (1u << 9); // PA9 CS
}


void lcd_send_data(uint8_t data) {

    // Write DC high and CS low
	GPIOB->BSRR = (1u << 10); // PB10 DC
	GPIOA->BSRR = (1u << (9 + 16)); // PA9 CS

    spi_send8(data);

    // Set CS back high
    GPIOA->BSRR = (1u << 9); // PA9 CS

}


void lcd_init(void) {
    lcd_gpio_init();
    spi1_init();

    // Hardware reset
    // Drive reset line low
    GPIOA->BSRR = (1u << (1 * (1 + 16)));

    // Short delay
    for (volatile int i=0; i<10000; ++i);

    // Drive reset line high
    GPIOA->BSRR = (1u << (1 * 1));

    // Short delay
    for (volatile int i=0; i<10000; ++i);

    // LCD Initialization Sequence
    lcd_send_command(0x11);

    for (volatile int i=0; i<10000; ++i); // short delay

    // Configure 16-bits per pixel
    lcd_send_command(0x3A);
    lcd_send_data(0x55);

    // Send MADCTL command
    lcd_send_command(0x36);
    lcd_send_data(0x48);

    // Send display ON command
    lcd_send_command(0x29);

    // Turn backlight on (PA8)
    GPIOA->BSRR = (1u << 8);
}
