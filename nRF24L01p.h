// nRF24L01+ register address, commands and other configurations


// Registers addres
#define CONFIG       0x00
#define EN_AA        0x01
#define EN_RXADDR    0x02
#define SETUP        0x03
#define SETUP_RETR   0x04
#define RF_CH        0x05
#define RF_SETUP     0x06
#define STATUS       0x07
#define OBSERVE_TX   0x08
#define RPD          0x09
#define RX_ADDR_P0   0x0A
#define RX_ADDR_P1   0x0B
#define FEATURE      0x1D

// Commands
#define R_REGISTER   0x00
#define W_REGISTER   0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX     0b11100001 
#define FLUSH_RX     0b11100010
#define REUSE_TX_PL  0b11100011
#define R_RX_PL_WID  0b01100000
#define W_ACK_PAYLOAD   0b10101000
#define W_TX_PAYLOAD_NOACK 0b10110000
#define NOP          0b11111111
