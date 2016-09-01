#include <spi.h>
#include "nRF24L01p.h"


void nRF24L01p_PTX_config(void)
{
   char *c_ptr, data;
   unsigned char n_bytes;
   
   // CONFIG: Power up
   data = 0b00001010;   
   c_ptr = &data;
   n_bytes = 1;
   SPI_ReadWriteAddr(W_REGISTER, CONFIG, c_ptr, n_bytes, SPI_WRITE);
   
   // SETUP_RETR: 500 us for auto-retransmit delay
   data = 0b00010011;
   c_ptr = &data;
   SPI_ReadWriteAddr(W_REGISTER, SETUP_RETR, c_ptr, n_bytes, SPI_WRITE);
   
   // TX_ADDR: Only LSB writen
   data = AIR_NODE_ADDR;   //
   c_ptr = &data;
   SPI_ReadWriteAddr(W_REGISTER, TX_ADDR, c_ptr, n_bytes, SPI_WRITE);   
   
   // RX_ADDR_P0
   data = AIR_NODE_ADDR;   //
   c_ptr = &data;
   SPI_ReadWriteAddr(W_REGISTER, RX_ADDR_P0, c_ptr, n_bytes, SPI_WRITE);

   // RX_ADDR_P1
   data = EARTH_NODE_ADDR;   //
   c_ptr = &data;
   SPI_ReadWriteAddr(W_REGISTER, RX_ADDR_P1, c_ptr, n_bytes, SPI_WRITE);      
}

void startRF_TXRX(void)
{
   CE_PIN = 0x01;  // Assert the CE pin on nRF24L01+
   IEC0bits.INT0IE = 1;		// Habilitar interrupcion externa
}
