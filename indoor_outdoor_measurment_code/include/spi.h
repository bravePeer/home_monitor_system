#pragma once
#include <stdint.h>

void intSPI(uint_fast16_t baud);

template <typename T, typename K>
uint8_t simpletransmitSPI(uint8_t cData, T csPort, K csPin);

uint8_t transmitSPI_New(uint8_t* sendBuf, uint8_t* receiveBuf, uint8_t cmd, uint8_t len);

template <typename K, typename T>
uint8_t transmitSPI(uint8_t* sendBuf, uint8_t* receiveBuf, uint8_t cmd, uint8_t len, K csnPort, T csnPin)
{
  *csnPort &= (~(1<<csnPin));
  uint8_t ret = transmitSPI_New(sendBuf, receiveBuf, cmd, len);
  *csnPort |= (1<<csnPin);
  return ret;
}


#if defined(ATTINY2313A)
extern "C" 
{
  #include "avr/io.h"
  #include "util/delay.h"
}


constexpr uint8_t misoPin = PD0;
constexpr uint8_t mosiPin = PD1;
constexpr uint8_t sckPin = PD2;
constexpr uint8_t csnPIN = PD4;

// extern volatile uint8_t misoPort;
// extern volatile uint8_t csnPort;

// uint8_t simpletransmitSPI(uint8_t cData, uint8_t csPin)

inline uint8_t receiveSPIcmd(uint8_t* receiveBuf, uint8_t cmd, uint8_t len)
{
  PORTD &= ~(1<<csnPIN);
  while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
  UDR = cmd;
  while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
  uint8_t statusReg = UDR;

  for (uint8_t i = 0; i < len - 1; i++)
  {
    while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
    UDR = 0xff;
    while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
    receiveBuf[i] = UDR;
  }
  
  PORTD |= (1<<csnPIN);

  return statusReg;
}

inline uint8_t receiveSPIcmd(uint8_t* receiveBuf, uint8_t cmd, uint8_t len, volatile uint8_t csPort, uint8_t csPin)
{
  PORTD &= ~(1<<csPin);
  // csPort &= ~(1<<csPin);
  while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
  UDR = cmd;
  while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
  uint8_t statusReg = UDR;

  for (uint8_t i = 0; i < len - 1; i++)
  {
    while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
    UDR = 0xff;
    while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
    receiveBuf[i] = UDR;
  }
  
  // csPort |= (1<<csPin);
  PORTD |= (1<<csPin);

  return statusReg;
}

inline uint8_t sendSPIcmd(uint8_t* sendBuf, uint8_t cmd, uint8_t len)
{
  PORTD &= ~(1<<csnPIN);
  while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
  UDR = cmd;
  while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
  uint8_t statusReg = UDR;

  for (uint8_t i = 0; i < len - 1; i++)
  {
    while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
    UDR = sendBuf[i];
    while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
    statusReg = UDR;
  }
  
  PORTD |= (1<<csnPIN);

  return statusReg;
}

inline uint8_t sendSPIcmd(uint8_t* sendBuf, uint8_t cmd, uint8_t len, volatile uint8_t* csPort, uint8_t csPin)
{
  *csPort &= ~(1<<csPin);
  while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
  UDR = cmd;
  while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
  uint8_t statusReg = UDR;

  for (uint8_t i = 0; i < len - 1; i++)
  {
    while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
    UDR = sendBuf[i];
    while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
    statusReg = UDR;
  }
  
  *csPort |= (1<<csPin);

  return statusReg;
}



#elif defined(ATMEGA328P)
//DDRB
constexpr uint8_t misoPin = PB4;
//DDRB
constexpr uint8_t mosiPin = PB3;
//DDRB
constexpr uint8_t sckPin = PB5;
//DDRB
constexpr uint8_t csnPIN = PB2;

void initSPI()
{
  /* Set MOSI and SCK output, all others input */
  DDRB |= (1<<mosiPin)|(1<<sckPin)|(1<<csnPIN);
  DDRB &= ~(1<<misoPin);
  PORTD |= (1<<csnPIN);
  
  /* Enable SPI, Master, set clock rate fck/16 */
  SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

void simpletransmitSPI(uint8_t data)
{
  PORTB &= ~(1<<csnPIN);
  /* Start transmission */
  SPDR = data;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
  PORTB |= (1<<csnPIN);
}

uint8_t transmitSPI(uint8_t* sendBuf, uint8_t* receiveBuf, uint8_t cmd, uint8_t len)
{
  //Enable
  PORTB &= ~(1<<csnPIN);

  SPDR = cmd;
  while(!(SPSR & (1<<SPIF))); /* Wait for data to be received */
  uint8_t statusReg = SPDR;

  for (uint8_t i = 0; i < len; i++)
  {
    SPDR = sendBuf[i];
    while(!(SPSR & (1<<SPIF))); /* Wait for data to be received */
    receiveBuf[i] = SPDR;
  }
  
  //Disable
  PORTB |= (1<<csnPIN);
  _delay_ms(5);
  return statusReg;
}


#endif
