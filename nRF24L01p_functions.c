#include <spi.h>
#include <timer.h>
#include "nRF24L01p.h"

extern char SPI_buffer[];
extern unsigned char n;
extern volatile struct status SPI_status;

void nRF24L01p_PTX_config(void)
{
   // CONFIG: PRIM_RX bit set LOW
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = 0b00001010;   
   SPI_ReadWriteAddr(W_REGISTER, CONFIG, SPI_buffer, 0x01, SPI_WRITE);
}

void nRF24L01p_PRX_config(void)
{
   // CONFIG: PRIM_RX bit set HIGH 
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = 0b00001011;   
   SPI_ReadWriteAddr(W_REGISTER, CONFIG, SPI_buffer, 0x01, SPI_WRITE);   
}

// Power up module and set the transmiting, receiving and ACK addresses
void nRF24L01p_PowerUp(void)
{
   
   // SETUP_RETR: 500 us for auto-retransmit delay
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = 0b00010011;
   SPI_ReadWriteAddr(W_REGISTER, SETUP_RETR, SPI_buffer, 0x01, SPI_WRITE);

   // TX_ADDR: Only LSB writen
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = AIR_NODE_ADDR;   //
   SPI_ReadWriteAddr(W_REGISTER, TX_ADDR, SPI_buffer, 0x01, SPI_WRITE);   
   
   // RX_ADDR_P0: This should be the same TX_ADDR to in order for the AKC packet to be received
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = AIR_NODE_ADDR;
   SPI_ReadWriteAddr(W_REGISTER, RX_ADDR_P0, SPI_buffer, 0x01, SPI_WRITE);

   // RX_ADDR_P1: This is the address the air node will be sending to.
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = EARTH_NODE_ADDR;   //
   SPI_ReadWriteAddr(W_REGISTER, RX_ADDR_P1, SPI_buffer, 0x01, SPI_WRITE);   

   // DYNPD: Enable dynamic payloads on P0 and P1
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = 0x03;   //
   SPI_ReadWriteAddr(W_REGISTER, DYNPD, SPI_buffer, 0x01, SPI_WRITE);    
   
   // FEATURE: 
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = 0b00000111;   //
   SPI_ReadWriteAddr(W_REGISTER, FEATURE, SPI_buffer, 0x01, SPI_WRITE);
   
   // CONFIG: Power up  
   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   SPI_buffer[0] = 0b00001010;
   SPI_ReadWriteAddr(W_REGISTER, CONFIG, SPI_buffer, 0x01, SPI_WRITE);    
}

void startRF_TXRX(void)
{
   unsigned char cnt;
   // Send 10 us pulse at CE_PIN 
   CE_PIN = 0x01;  // Assert the CE pin on nRF24L01+
   ConfigIntTimer1(T1_INT_OFF);   // Disable timer 1 interrupt
   WriteTimer1(0);
   do{
      cnt = ReadTimer1();
   }while(cnt < CE_PERIOD);
   CE_PIN = 0x00;  // Set low the CE pin on nRF24L01+
   ConfigIntTimer1(T1_INT_ON);   // Enable timer 1 interrupt
}
