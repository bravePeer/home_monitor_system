#ifdef ATMEGA328P
#pragma once
extern "C" 
{
  #include "avr/io.h"
  #include "util/delay.h"
}

constexpr uint32_t fosc = 16000000;
constexpr uint16_t baudrate = 9600;
constexpr uint16_t usartUBRR = fosc/16/baudrate-1;

void initUSART(uint16_t ubrr)
{
    /*Set baud rate */
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter 
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    /* Set frame format: 8data, 2stop bit */
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void transmitUART(uint8_t data)
{
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1<<UDRE0)))
    ;

    /* Put data into buffer, sends the data */
    UDR0 = data;
}
#endif