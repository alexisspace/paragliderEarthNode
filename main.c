#include <p33FJ12GP202.h>
#include <i2c.h>
#include <libq.h>
#include <dsp.h>
#include <string.h>
#include <spi.h>
#include "nRF24L01p.h"


//#define XTFREQ          7370000         //FRC frequency 
#define FCY             39613000        //Instruction Cycle Frequency
#define BAUDRATE         9600		      
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 

// Constantes de la implementacion
#define DEG2RAD				17.4533e-003
#define USART_TX_BUFFER_SIZE	10
#define SPI_BUFFER_SIZE	10
#define BYTES_TO_READ 10

// Variables globales
char SPI_buffer[SPI_BUFFER_SIZE], U1TX_buffer[USART_TX_BUFFER_SIZE], *SPI_buffer_ptr;
unsigned char sys_cmd, U1TX_byte_counter;
unsigned char SPI_byte_counter, SPI_max_bytes;

typedef struct status {
   char status;
   char data;
   }; // Se define el tipo "struct status"
//typedef struct status STATUS; // Se abrevia STATUS

volatile struct status UART_status = {0,0};
volatile struct status SPI_status = {0,0}; // Estado del SPI
volatile struct status SYS_status = {0,0}; // Estado del sistema

// Prototipos de funciones
char UARTbufferSend(char* c_ptr);
char SPI_WriteAddr(unsigned char cmd, unsigned char addr, char * d_ptr, unsigned char n_data);
char SPI_ReadAddr(unsigned char cmd, unsigned char addr, char * d_ptr, unsigned char n_data);
void startRF_TXRX(void);

int main(void)
{
   unsigned char cmd, addr, n_bytes, k; 
	unsigned int config1, config2, config3;

// Configure Oscillator to operate the device at 40Mhz
// Fosc = Fin*M/(N1*N2), Fcy=Fosc/2
// Fosc = 7.37*43/(2*2)=79.23MHz
	PLLFBD = 41;					// M=43
	CLKDIVbits.PLLPOST = 0;		// N1=2
	CLKDIVbits.PLLPRE = 0;		// N2=2

// Disable Watch Dog Timer
	RCONbits.SWDTEN=0;

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};

// Configurar pines
   // Desbloquear registros
   //__builtin_write_OSCCONL(0x46); // unlock sequence - step 1
   //__builtin_write_OSCCONH(0x57); // unlock sequence - step 2
   //_IOLOCK = 0;   // Unlock
	RPINR18bits.U1RXR = 9;			// Make Pin RP9 U1RX
	RPOR4bits.RP8R = 3;		// Make Pin RP8 U1TX
	RPINR20bits.SDI1R = 0x01;          // SDI assigned to RP1
	RPOR1bits.RP2R = 0X07;  // SDO assigned to RP2
	RPOR1bits.RP3R = 0X08;  // SCK assigned to RP3
	// Volver a bloquear registros
   //__builtin_write_OSCCONL(0x46); // unlock sequence - step 1
   //__builtin_write_OSCCONH(0x57); // unlock sequence - step 2
   //_IOLOCK = 1; // re-lock the ports	
	
	AD1PCFGL = 0xFFFF;		// Make analog pins digital
	// Config LED's as output and U1RX as Input; RB10 is SS (slave select);
	// RB11 is nRF24L01+ CE (Chip enable, activates RX or TX mode)
	TRISB = 0x0293;
	LATB = 0xFC0F;
	
// Configurar UART
	U1BRG  = BRGVAL;
	U1MODE = 0x8000; /* Reset UART to 8-n-1, alt pins, and enable */
	U1STA  = 0x0440; /* Reset status register and enable TX & RX*/
	U1STAbits.UTXISEL1 = 0;
	U1STAbits.UTXISEL0 = 1; // Interrupt after one TX character is transmitted
	U1MODEbits.UARTEN = 1; // Enable UART
	U1STAbits.UTXEN = 1; // Enable UART TX
	IEC0bits.U1TXIE = 0; // Disable UART TX interrupt
	IEC0bits.U1RXIE = 1; // Enable RX interrupt

// Configurar interrupcion externa INT0 (RB7, PIN16)
	INTCON2bits.INT0EP = 1;		// Interrupcion en 0:flanco ascendente 1: flanco descendente
	IFS0bits.INT0IF = 0;		// Clear flag
	CNPU2bits.CN23PUE = 0;	// pull-up on RB7: 0 = Disable; 1 = Enable
	//	IEC0bits.INT0IE = 1;		// Habilitar interrupcion

// Configuar Timer1. Habilitar interrupcion
	TMR1 = 0;				// clear timer 1
	PR1 = 0x78E4;//0x3C72;			// interrupt every 100ms
	T1CON = 0x8030;			// 1:256 prescale, start TMR1
	IFS0bits.T1IF = 0;		// clr interrupt flag
	IEC0bits.T1IE = 1;		// set interrupt enable bit

// Configurar SPI
// PRI_PRESCAL_1_1 & SEC_PRESCAL_6_1 for ~6 MHz
   config1 = 0x0000;
   config1 = ENABLE_SCK_PIN & ENABLE_SDO_PIN & SPI_MODE16_OFF & 
   SPI_SMP_OFF & SPI_CKE_ON & SLAVE_ENABLE_OFF & 
   CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON & SEC_PRESCAL_6_1 & PRI_PRESCAL_1_1;
   
   config2 = 0x0000;
   config2 = FRAME_ENABLE_OFF;
   
   config3 = 0x0000;
   config3 = SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR;
   
   OpenSPI1(config1, config2, config3);
   ConfigIntSPI1(SPI_INT_PRI_3 & SPI_INT_EN);   // Enable interrupt
   
   CSN_PIN = 0x01;  // SS idle


	while(1)
	{
		
		switch(sys_cmd){
   		case 'p': // Read the first 10 nRF24L01+ (8 bit) registers  		   
   		   // Fill data buffer with addres to be read
   		   cmd = R_REGISTER; // nRF24L01+ command
   		   addr = CONFIG;
   		   n_bytes = 1;
   		   for(k = 0; k < BYTES_TO_READ; k++){
      		   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   		      SPI_ReadAddr(cmd, addr, &SPI_buffer[k], n_bytes);
   		      addr++; // Increment register address
            }
            sys_cmd = 0; // Reset the command as is already executed
            LATBbits.LATB14 = !LATBbits.LATB14; // Toggle LED (D5)
   		break;
   		case 'q': // Read one nRF24L01+ (5 byte) registers
   		   cmd = R_REGISTER; // nRF24L01+ command
   		   addr = RX_ADDR_P0; // RX_ADDR_P0: RX address Pipe 0
   		   n_bytes = 5;
   		   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   		   SPI_ReadAddr(cmd, addr, SPI_buffer, n_bytes);
   		   sys_cmd = 0; // Reset the command as is already executed
   		break;
   		case 'r': // Read usign an specific address read command
   		   cmd = R_RX_PL_WID; // nRF24L01+ command, R_RX_PL_WID: Read Payload Width
   		   addr = 0x00;   // No address with this command
   		   n_bytes = 1;
   		   while(SPI_status.status != 0 ); // Wait for SPI to be idle
   		   SPI_ReadAddr(cmd, addr, SPI_buffer, n_bytes);   	
   		   sys_cmd = 0; // Reset the command as is already executed	   
   		break;
		}
		
		// Despues de recibir los datos del nRF24L01+ enviarlos a la PC
		if(SPI_status.status == 0 && UART_status.data != 0 && UART_status.status == 0 && sys_cmd == 0){
   		LATBbits.LATB15 = !LATBbits.LATB15;		//Toggle LED (D4)
   		UARTbufferSend(SPI_buffer);
		}
		
		//while(!U1STAbits.TRMT);	// Esperar a que se transmita el byte antes de leer el otro de la IMU
		//U1TXREG = cmd;

	}//while(1)

	return 0;
}

/**************************************************************************/
// Funciones adicionales
/**************************************************************************/
char UARTbufferSend(char* c_ptr)
{
   // Check if UART is not already transmiting
   if(UART_status.status == 0){
      UART_status.status = 1; // Update to transmiting status
	   memcpy(U1TX_buffer, c_ptr, USART_TX_BUFFER_SIZE);

	   // Iniciar transmision de mensaje por el UART
	   U1TX_byte_counter = 1;		// Inicializar valor de contador
	   U1TXREG = U1TX_buffer[0];
	   IEC0bits.U1TXIE = 1; // Enable UART TX interrupt
	   return 1;
	}else{
	// UART is already transmiting
	   return 0;
	}
}

// commands that writes specific memory locations: W_TX_PAYLOAD, W_TX_PAYLOAD_NOACK (addr = 0x00)
// commands that writes variable memory locations: W_REGISTER, W_ACK_PAYLOAD
char SPI_WriteAddr(unsigned char cmd, unsigned char addr, char * d_ptr, unsigned char n_data)
{
   // NOTA:UNIR ESTA FUNCION CON SPI_ReadAddr, YA QUE SON PARECIDAS Y SE DUPLICA EL CODIGO
   // status: 0: Idle; 1: Sending; 2: Receiving
   if(SPI_status.status == 0){
      SPI_max_bytes = n_data;
      SPI_buffer_ptr = d_ptr;
      SPI_byte_counter = 0;
      SPI_status.status = 1; // Set the sending status
      // Assert Chip select  pin
      CSN_PIN = 0x00;  //
      WriteSPI1(cmd | addr); // The first byte is the command byte
   }else{
      return 0; // Transaction in process
   }
}
// commands that reads specific locations: R_RX_PAYLOAD, R_RX_PL_WID (addr = 0x00)
// commands that reads variable locations: R_REGISTER
char SPI_ReadAddr(unsigned char cmd, unsigned char addr, char * d_ptr, unsigned char n_data)
{
   // status: 0: Idle; 1: Sending; 2: Receiving
   if(SPI_status.status == 0){
      SPI_max_bytes = n_data;
      SPI_buffer_ptr = d_ptr;
      SPI_byte_counter = 0;
      SPI_status.status = 2; // Set the receiving status
      // Assert Chip Select  pin
      CSN_PIN = 0x00;  //
      WriteSPI1(cmd | addr); // The first byte is the command byte
   }else{
      return 0; // Error: Transaction in process
   }
}

void startRF_TXRX(void)
{
   LATBbits.LATB11 = 0x01;  // Assert the CE pin on nRF24L01+
   IEC0bits.INT0IE = 1;		// Habilitar interrupcion externa
}
/**************************************************************************/
// Vectores de interrupcion
/**************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{

	IFS0bits.T1IF = 0;	// clear interrupt flag	
	LATBbits.LATB15 = !LATBbits.LATB15;		//Toggle LED (D4)
	

}

void __attribute__((__interrupt__, no_auto_psv)) _INT0Interrupt(void)
{
	// Manejo de la interrupcion externa: RB7, PIN16
	// nRF24L01+ has sended an interruption request
	
	LATBbits.LATB11 = 0x00; // CE pin. desactivar la transmisión de datos del nRF24L01+
	
   // Leer el registro STATUS para verificar la fuente de interrupcion
   
   // Escribir el registro STATUS para terminar la condicion de interrupcion


	LATBbits.LATB14 = !LATBbits.LATB14;		// On LED5 (D5) (inverted)
	IFS0bits.INT0IF = 0;		// Clear flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
	// Aqui se transmite los datos por el puerto serial

	
	IFS0bits.U1TXIF = 0;	// Limpiar bandera de interrupcion
	if(U1TX_byte_counter < USART_TX_BUFFER_SIZE)
	{
		LATBbits.LATB13 = !LATBbits.LATB13;		// Toggle LED (D6) 
		U1TXREG = U1TX_buffer[U1TX_byte_counter];
		U1TX_byte_counter++;
	}
	else
	{
		IEC0bits.U1TXIE = 0; // Disable UART TX interrupt until new buffer is
		                     // available for transmit
		// Despues de enviar el buffer (y haber ejecutado el comando)
		// se reinicia el comando para indicar que fue ejecutado
		UART_status.data = 0;
		UART_status.status = 0; // Update to idle status
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
   _U1RXIF = 0;					// Clear UART RX Interrupt Flag
	LATBbits.LATB12 = !LATBbits.LATB12;		// Toggle LED (D7)
   UART_status.data = U1RXREG;
   sys_cmd = UART_status.data;
}

void __attribute__((__interrupt__, no_auto_psv)) _SPI1ErrInterrupt (void)
{
   // To be filled with appropiate code
}


void __attribute__((__interrupt__, auto_psv)) _SPI1Interrupt (void)
{
   IFS0bits.SPI1IF = 0x00; // Clear interrupt flag
   // SPI1 transfer completed interrupt 
   switch(SPI_status.status)
   {
      case 1: // Transmiting
         // The first received byte is the STATUS register of nRF24L01+
         if(SPI_byte_counter == 0){
            SPI_status.data = ReadSPI1();
         }
         ReadSPI1(); // Discard the received byte while transmiting
         WriteSPI1(SPI_buffer_ptr[SPI_byte_counter] & 0x00ff);
         SPI_byte_counter++;
         if(SPI_byte_counter == SPI_max_bytes){
            SPI_status.status = 0;  // Transmision complete, return to idle
            CSN_PIN = 0x01;  // SS pin, release slave
         }
         break;
      case 2: // Receiving
         // The first received byte is the STATUS register of nRF24L01+
         if(SPI_byte_counter == 0){
            SPI_status.data = ReadSPI1();
         }else{
            SPI_buffer_ptr[SPI_byte_counter-1] = ReadSPI1();
         }
         
         SPI_byte_counter++;
         if(SPI_byte_counter > SPI_max_bytes){
            SPI_status.status = 0;  // Transmision complete, return to idle
            CSN_PIN = 0x01;  // SS pin, release slave
         }else{
            WriteSPI1(0x00); // Dummy write for clocking the incoming byte
         }
         break;
   }
}
