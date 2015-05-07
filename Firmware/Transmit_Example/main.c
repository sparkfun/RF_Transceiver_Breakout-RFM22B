/*
    Created on: Mar. 30, 2010
	Under project: RFM22
	Author: Jim Lindblom
	RFM22 Transmit Example
	
	Hardware: Arduino Pro (ATmega328) running at 8Mhz, 3.3V
	Wiring:
	RFM22 BOB	|	Arduino Pro
	----------------------------
		GND		|	GND
		3.3V	|	3.3V
		GPIO0	|	NC
		GPIO1	|	NC
		GPIO2	|	NC
		RXANT	|	PD6 (D6)
		TXANT	|	PD5 (D5)
		IRQ		|	PD4 (D4)
		CSN		|	PB0 (D8)
		SCK		|	PB4 (D13)
		SDI		|	PB2 (D11)
		SDO		|	PB3 (D12)
		
	Antenna is a 17cm piece of wire sticking straight up into the air.
*/

//======================//
//======================//
#include <avr/io.h>
#include <stdio.h>
#include "RFM22.h"
//======================//
//======================//

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

//======================//
//Arduino Pro Specific Defines
#define LED 5

//RFM22 Defines
#define CS 0 //pin for chip select
#define NIRQ 4
#define TXANT 5	// PD5
#define RXANT 6	// PD6

//======================//
//======================//

void ioinit(void);
void delay_ms(uint16_t x);
void delay_us(uint16_t x);

//======================//
//======================//

static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//======================//
//======================//

void init_SPI(void);
void init_RFM22(void);
void to_tx_mode(void);
void checkINT(void);
void get_packet(void);
void txdata(char data);
char rxdata(void);
char read(uint8_t address);
void write(uint8_t address, char data);

//======================//
//Global Variables
unsigned char tx_buf[17] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x78};

int main(void)
{
	char temp, inkey;
	unsigned char i;

	ioinit();
	init_SPI();
	sbi(PORTB,CS);
	printf("/n/n********RFM22 Communication Test********\n");

	//====================//
	//Communications Test
	
	temp = read(DTYPE);
	temp = read(DVERS);
	temp = read(INTEN1);
	temp = read(INTEN2);
	temp = read(OMFC1);
	temp = read(OMFC2);
	
	printf("*****************************************\n\n");
	
	init_RFM22();	// Initialize all RFM22 registers
	to_tx_mode();	// Send test packet	'0123456789:;<=>?"
	
	/*// This exampes sends '0123456789:;<=>?' once a second
	while(1)
	{
		to_tx_mode();
		printf("Transmit Done\n");
		delay_ms(1000);
	}
	*/
	
	// This example allows you to enter a 16-byte packet to send
	printf("Entering TX Mode...Give me a 16-byte packet\n\n");
	while(1)
	{
		
		get_packet();
		printf("Transmitting: ");
		for (i=0; i<17; i++)
			printf("%c", tx_buf[i]);
		printf("\n");
		to_tx_mode();
		printf("Transmit done...Press any key to transmit again\n\n");
	}
	
	/*
	// This example sends a 'beep' to the terminal every second
	// You'll love it...
	int count = 0;
	unsigned char chksum;
	
	for (i=0; i<15; i++)
		tx_buf[i] = 0x07;
	tx_buf[15] = 0x31;
	
	chksum = 0;
	for(i=0; i<16; i++)
		chksum += tx_buf[i];
	
	tx_buf[16] = chksum;
	printf("chksum == %x\r", tx_buf[16]);
	
	while(1)
	{
		to_tx_mode();
		delay_ms(1000);
		
		printf("Transmission Complete...%d\n", count);
		count++;
		
	}*/
}

// Initialize the RFM22 for transmitting
void init_RFM22(void)
{
	write(INTEN2, 0x00);	// Disable all interrupts
	write(OMFC1, 0x01);		// Set READY mode
	write(0x09, 0x7F);		// Cap = 12.5pF
	write(0x0A, 0x05);		// Clk output is 2MHz
	
	write(0x0B, 0xF4);		// GPIO0 is for RX data output
	write(0x0C, 0xEF);		// GPIO1 is TX/RX data CLK output
	write(0x0D, 0x00);		// GPIO2 for MCLK output
	write(0x0E, 0x00);		// GPIO port use default value
	
	write(0x0F, 0x70);		// NO ADC used
	write(0x10, 0x00);		// no ADC used
	write(0x12, 0x00);		// No temp sensor used
	write(0x13, 0x00);		// no temp sensor used
	
	write(0x70, 0x20);		// No manchester code, no data whiting, data rate < 30Kbps
	
	write(0x1C, 0x1D);		// IF filter bandwidth
	write(0x1D, 0x40);		// AFC Loop
	//write(0x1E, 0x0A);	// AFC timing
	
	write(0x20, 0xA1);		// clock recovery
	write(0x21, 0x20);		// clock recovery
	write(0x22, 0x4E);		// clock recovery
	write(0x23, 0xA5);		// clock recovery
	write(0x24, 0x00);		// clock recovery timing
	write(0x25, 0x0A);		// clock recovery timing
	
	//write(0x2A, 0x18);
	write(0x2C, 0x00);
	write(0x2D, 0x00);
	write(0x2E, 0x00);
	
	write(0x6E, 0x27);		// TX data rate 1
	write(0x6F, 0x52);		// TX data rate 0
	
	write(0x30, 0x8C);		// Data access control
	
	write(0x32, 0xFF);		// Header control
	
	write(0x33, 0x42);		// Header 3, 2, 1, 0 used for head length, fixed packet length, synchronize word length 3, 2,
	
	write(0x34, 64);		// 64 nibble = 32 byte preamble
	write(0x35, 0x20);		// 0x35 need to detect 20bit preamble
	write(0x36, 0x2D);		// synchronize word
	write(0x37, 0xD4);
	write(0x38, 0x00);
	write(0x39, 0x00);
	write(0x3A, 's');		// set tx header 3
	write(0x3B, 'o');		// set tx header 2
	write(0x3C, 'n');		// set tx header 1
	write(0x3D, 'g');		// set tx header 0
	write(0x3E, 17);		// set packet length to 17 bytes
	
	write(0x3F, 's');		// set rx header
	write(0x40, 'o');
	write(0x41, 'n');
	write(0x42, 'g');
	write(0x43, 0xFF);		// check all bits
	write(0x44, 0xFF);		// Check all bits
	write(0x45, 0xFF);		// check all bits
	write(0x46, 0xFF);		// Check all bits
	
	write(0x56, 0x01);
	
	write(0x6D, 0x07);		// Tx power to max
	
	write(0x79, 0x00);		// no frequency hopping
	write(0x7A, 0x00);		// no frequency hopping
	
	write(0x71, 0x22);		// GFSK, fd[8]=0, no invert for TX/RX data, FIFO mode, txclk-->gpio
	
	write(0x72, 0x48);		// Frequency deviation setting to 45K=72*625
	
	write(0x73, 0x00);		// No frequency offset
	write(0x74, 0x00);		// No frequency offset
	
	write(0x75, 0x53);		// frequency set to 434MHz
	write(0x76, 0x64);		// frequency set to 434MHz
	write(0x77, 0x00);		// frequency set to 434Mhz
	
	write(0x5A, 0x7F);
	write(0x59, 0x40);
	write(0x58, 0x80);
	
	write(0x6A, 0x0B);
	write(0x68, 0x04);
	write(0x1F, 0x03);
}

void to_tx_mode(void)
{
	unsigned char i;
	
	write(0x07, 0x01);	// To ready mode
	cbi(PORTD, RXANT);
	sbi(PORTD, TXANT);
	delay_ms(50);
	
	write(0x08, 0x03);	// FIFO reset
	write(0x08, 0x00);	// Clear FIFO
	
	write(0x34, 64);	// preamble = 64nibble
	write(0x3E, 17);	// packet length = 17bytes
	for (i=0; i<17; i++)
	{
		write(0x7F, tx_buf[i]);	// send payload to the FIFO
	}

	write(0x05, 0x04);	// enable packet sent interrupt
	i = read(0x03);		// Read Interrupt status1 register
	i = read(0x04);
	
	write(0x07, 9);	// Start TX
	
	while ((PIND & (1<<NIRQ)) != 0)
		; 	// need to check interrupt here
	
	write(0x07, 0x01);	// to ready mode
	
	cbi(PORTD, RXANT);	// disable all interrupts
	cbi(PORTD, TXANT);
}

void get_packet(void)
{
	unsigned char i, chksum;
	
	for(i=0; i<16; i++)
	{
		tx_buf[i] = uart_getchar();
		printf("Received %c, %d characters remaining for packet\n", tx_buf[i], 15-i);
	}
	
	chksum = 0;
	for(i=0; i<16; i++)
		chksum += tx_buf[i];
	
	tx_buf[16] = chksum;
}

void checkINT(void)
{
	if ((PIND & (1<<NIRQ)) == 0)
		printf("INT == 0\n");
	else
		printf("INT == 1\n");
}

void write(uint8_t address, char data)
{
	//write any data byte to any single address
	//adds a 0 to the MSB of the address byte (WRITE mode)
	
	address |= 0x80;

	cbi(PORTB,CS);
	delay_ms(1);
	txdata(address);
	delay_ms(1);
	txdata(data);
	delay_ms(1);
	sbi(PORTB,CS);
}

char read(uint8_t address)
{
	//returns the contents of any 1 byte register from any address
	//sets the MSB for every address byte (READ mode)

	char byte;
	
	address &= 0x7F;

	cbi(PORTB,CS);
	txdata(address);
	byte = rxdata();
	sbi(PORTB,CS);

	return byte;
}

char rxdata(void)
{
	SPDR = 0x55;
	while((SPSR&0x80) == 0x00)
		;

	return SPDR;
}

void txdata(char data)
{
	SPDR = data;
	while((SPSR&0x80) == 0x00);
}

void init_SPI(void)
{
	// enable SPI
	// make SPI master
	// SCLK idle low
	// sample data on rising edge
	// Fosc/4 is SPI frequency = 2MHz
	//SPCR |= 0b01010000;	// Fosc/4
	SPCR |= 0b01010011;		// Fosc/128
}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);

    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;

    return 0;
}

uint8_t uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}


void ioinit (void)
{
    //1 = output, 0 = input
    DDRB = 0b11101111; //MISO input
    DDRC = 0b11111111; //All outputs
    DDRD = 0b11101110; //PORTD (RX on PD0), PD4 input
	stdout = &mystdout; //Required for printf init
	int MYUBRR = 103;
	UBRR0H = (MYUBRR) >> 8;
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
	UCSR0A = (1<<U2X0);

	TCCR2B = (1<<CS21);
	
	cbi(PORTD, TXANT);
	cbi(PORTD, RXANT);
}

//General short delays
void delay_ms(uint16_t x)
{
    for (; x > 0 ; x--)
        delay_us(1000);
}

//General short delays
void delay_us(uint16_t x)
{
    while(x > 256)
    {
        TIFR2 = (1<<TOV2); //Clear any interrupt flags on Timer2
        TCNT2 = 0; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click
        while( (TIFR2 & (1<<TOV2)) == 0);

        x -= 256;
    }

    TIFR2 = (1<<TOV2); //Clear any interrupt flags on Timer2
    TCNT2= 256 - x; //256 - 125 = 131 : Preload timer 2 for x clicks. Should be 1us per click
    while( (TIFR2 & (1<<TOV2)) == 0);
}

