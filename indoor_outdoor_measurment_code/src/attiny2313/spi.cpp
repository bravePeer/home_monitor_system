#if defined(ATTINY2313A)

#include "spi.h"

void intSPI(uint_fast16_t baud)
{
  /* Init SPI */
  UBRRL = 0;
  UBRRH = 0;
  /* Setting the XCK port pin as output, enables master mode. */
  XCK_DDR |= (1<<XCK_BIT);
  /* Set MSPI mode of operation and SPI data mode 0. */
  UCSRC = (1<<UMSEL1)|(1<<UMSEL0)|(0<<UCPHA)|(0<<UCPOL);
  /* Enable receiver and transmitter. */
  UCSRB = (1<<RXEN) | (1<<TXEN);
  /* Set baud rate. */
  /* IMPORTANT: The Baud Rate must be set after the transmitter is enabled*/
  UBRRL = static_cast<uint8_t>(baud);
  UBRRH = static_cast<uint8_t>(baud>>8);
}

template <typename T, typename K>
uint8_t simpletransmitSPI(uint8_t cData, T csPort, K csPin)
{
  csPort &= ~(1<<csPin);
  /* Wait for empty transmit buffer */
  while ( !( UCSRA & (1<<UDRE)) );
  /* Put data into buffer, sends the data */
  UDR = cData;
  /* Wait for data to be received */
  while ( !(UCSRA & (1<<RXC)) );
  /* Get and return received data from buffer */
  csPort |= (1<<csPin);

  return UDR;
}

uint8_t transmitSPI(uint8_t* sendBuf, uint8_t* receiveBuf, uint8_t cmd, uint8_t len)
{
  //Enable
  PORTD &= ~(1<<csnPIN);

  while (!( UCSRA & (1<<UDRE)));/* Wait for empty transmit buffer */
  UDR = cmd; // Wpisanie wartości do bufora
  while (!(UCSRA & (1<<RXC))); /* Wait for data to be received */
  uint8_t statusReg = UDR;

  for (uint8_t i = 0; i < len; i++)
  {
    while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
    UDR = sendBuf[i];
    while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
    receiveBuf[i] = UDR;
  }

  while(!(UCSRA &(1<<TXC))) { } // Wait for end transmission
  
  UCSRA |= (1<<TXC);

  //Disable
  PORTD |= (1<<csnPIN);
  
  return statusReg;
}

uint8_t transmitSPI_New(uint8_t* sendBuf, uint8_t* receiveBuf, uint8_t cmd, uint8_t len)
{
  while (!( UCSRA & (1<<UDRE)));/* Wait for empty transmit buffer */
  UDR = cmd; // Wpisanie wartości do bufora
  while (!(UCSRA & (1<<RXC))); /* Wait for data to be received */
  uint8_t statusReg = UDR;

  for (uint8_t i = 0; i < len; i++)
  {
    while ( !( UCSRA & (1<<UDRE)) );/* Wait for empty transmit buffer */
    if(sendBuf == nullptr)
      UDR = 0xFF;
    else
      UDR = sendBuf[i];
    while ( !(UCSRA & (1<<RXC)) ); /* Wait for data to be received */
    receiveBuf[i] = UDR;
  }

  while(!(UCSRA &(1<<TXC))) { } // Wait for end transmission
  
  UCSRA |= (1<<TXC);
  return statusReg;
}

#endif