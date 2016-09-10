// nRF24L01+ register address, commands and other configurations

typedef struct status {
   char status;
   unsigned char data;
   unsigned char cmd;
   } STATUS; // Se define el tipo "struct status"
//typedef struct status STATUS; // Se abrevia STATUS


// Registers addres
#define CONFIG       0x00
#define EN_AA        0x01
#define EN_RXADDR    0x02
#define SETUP        0x03
#define SETUP_RETR   0x04
#define RF_CH        0x05
#define RF_SETUP     0x06
#define STATUS_REG   0x07
#define OBSERVE_TX   0x08
#define RPD          0x09
#define RX_ADDR_P0   0x0A
#define RX_ADDR_P1   0x0B
#define RX_ADDR_P2   0x0C
#define RX_ADDR_P3   0x0D
#define RX_ADDR_P4   0x0E
#define RX_ADDR_P5   0x0F
#define TX_ADDR      0x10
#define RX_PW_P0     0x11
#define RX_PW_P1     0x12
#define DYNPD        0x1C
#define FEATURE      0x1D
#define FIFO_STATUS  0x17

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
#define NOP_CMD          0b11111111

// Pin assigments
#define CSN_PIN LATBbits.LATB10
#define CE_PIN  LATBbits.LATB11
#define D4_PIN  LATBbits.LATB15
#define D5_PIN  LATBbits.LATB14
#define D6_PIN  LATBbits.LATB13
#define D7_PIN  LATBbits.LATB12

// Other constans
#define SPI_READ        0x00
#define SPI_WRITE       0x01
#define AIR_NODE_ADDR   0xE5
#define EARTH_NODE_ADDR 0xAB
#define CE_PERIOD       0x16
#define GET_GPS_DATA    0x11

// Public function prototypes
void nRF24L01p_PTX_config(void);
void nRF24L01p_PRX_config(void);
void nRF24L01p_PowerUp(void);
void startRF_TXRX(void);
char SPI_ReadWriteAddr(unsigned char cmd, unsigned char addr,
    char * d_ptr, unsigned char n_data, unsigned char rw);

