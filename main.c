

#include <p33FJ12GP202.h>
#include <i2c.h>
#include <libq.h>
#include <dsp.h>
#include <string.h>
#include <spi.h>
// the above include path may be different for each user.  If a compile
// time error appears then check the path for the file above and edit
// the include statement above.

//#define XTFREQ          7370000         //FRC frequency 
#define FCY             39613000        //Instruction Cycle Frequency

#define BAUDRATE         9600		      
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 


// Registros del nRF24L01+




#define N_SAMPLE_BYTES		14
#define USART_TX_BUFFER_SIZE	10
#define SPI_BUFFER_SIZE	10



// Constantes de la implementacion
#define DEG2RAD				17.4533e-003




// Variables globales
unsigned char SPI_buffer[SPI_BUFFER_SIZE], U1TX_buffer[USART_TX_BUFFER_SIZE];
unsigned char cmd, U1TX_byte_counter;
unsigned char SPI_byte_counter, SPI_max_bytes, *SPI_buffer_ptr;
struct status
{
   char status;
   unsigned char data;
}; // Se define el tipo "struct status"
typedef struct status STATUS; // Se abrevia STATUS
STATUS UART_status, SPI_status; // Se definen variables de tipo STATUS


// Prototipos de funciones
char UARTbufferSend(char* c_ptr);
char SPI_WriteCmd(char reg, char * d_ptr, unsigned char n_data);
char SPI_ReadCmd(char reg, char * d_ptr, unsigned char n_data);

int main(void)
{
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
	RPINR18 = 9;			// Make Pin RP9 U1RX
	RPOR4bits.RP8R = 3;		// Make Pin RP8 U1TX
	RPIN20 = 0x01;          // SDI assigned to RP1
	RPOR1bits.RP2R = 0X07;  // SDO assigned to RP2
	RPOR1bits.RP3R = 0X08;  // SCK assigned to RP3            
	AD1PCFGL = 0x03C0;		// Make analog pins digital
	// Config LED's as output and U1RX as Input; RB10 is SS (slave select);
	// RB11 is nRF24L01+ CE (Chip enable, activates RX or TX mode)
	TRISB = 0x029F;
	LATB = 0xF000;
	
// Configurar UART
	U1BRG  = BRGVAL;
	U1MODE = 0x8000; /* Reset UART to 8-n-1, alt pins, and enable */
	U1STA  = 0x0440; /* Reset status register and enable TX & RX*/
	U1STAbits.UTXISEL1 = 0;
	U1STAbits.UTXISEL0 = 1; // Interrupt after one TX character is transmitted
	U1MODEbits.UARTEN = 1; // Enable UART
	U1STAbits.UTXEN = 1; // Enable UART TX
	IEC0bits.U1TXIE = 0; // Disable UART TX interrupt

// Configurar interrupcion externa INT0 (RB7, PIN16)
	INTCON2bits.INT0EP = 1;		// Interrupcion en 0:flanco ascendente 1: flanco descendente
	IFS0bits.INT0IF = 0;		// Clear flag
//	IEC0bits.INT0IE = 1;		// Habilitar interrupcion
	CNPU2bits.CN23PUE = 0;	// pull-up on RB7: 0 = Disable; 1 = Enable

// Configuar Timer1. Habilitar interrupcion
	TMR1 = 0;				// clear timer 1
	PR1 = 0x78E4;//0x3C72;			// interrupt every 100ms
	T1CON = 0x8030;			// 1:256 prescale, start TMR1
	IFS0bits.T1IF = 0;		// clr interrupt flag
//	IEC0bits.T1IE = 1;		// set interrupt enable bit

// Configurar SPI
// PRI_PRESCAL_1_1 & SEC_PRESCAL_6_1 for ~6 MHz
config1 = 0x0000;
config1 = ENABLE_SCK_PIN & ENABLE_SDO_PIN & SPI_MODE16_OFF & 
SPI_SMP_OFF & SPI_CKE_ON & SLAVE_ENABLE_OFF & 
CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON & SEC_PRESCAL_6_1 & PRI_PRESCAL_1_1;

config2 = 0x0000;
config2 = FRAME_ENABLE_OFF & FRAME_SYNC_OUTPUT & FRAME_POL_ACTIVE_LOW &
FRAME_SYNC_EDGE_COINCIDE;

config3 = 0x0000;
config3 = SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR;

OpenSPI1(config1, config2, config3);
ConfigIntSPI1(SPI_INT_PRI_3 & SPI_INT_EN);   // Enable interrupt
LATBbits.LATB2 = 0x01;  // SS idle


// Activar LED que indica entrada en el lazo principal del programa
	LATBbits.LATB13 = 0x00;		// On LED (D6)
	while(1)
	{
		while (_U1RXIF == 0);			// Wait and Receive One Character
		cmd = U1RXREG;
		_U1RXIF = 0;					// Clear UART RX Interrupt Flag
		
		while(!U1STAbits.TRMT);	// Esperar a que se transmita el byte antes de leer el otro de la IMU
		U1TXREG = cmd;

	}//while(1)

	return 0;
}

// Funciones adicionales
char UARTbufferSend(char* c_ptr)
{
   // Check if UART is not already transmiting
   if(UART_status.status == 0){
      UART_status.status = 1; // Update to transmiting status
	   memcpy(U1TX_buffer, c_ptr, USART_TX_BUFFER_SIZE-1);
	   U1TX_buffer[USART_TX_BUFFER_SIZE-1] = 0xFF;

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

char SPI_WriteCmd(char reg, char * d_ptr, unsigned char n_data)
{
   // status: 0: Idle; 1: Sending; 2: Receiving
   if(SPI_status.status == 0){
      SPI_max_bytes = n_data;
      SPI_buffer_ptr = d_ptr;
      SPI_byte_counter = 0;
      SPI_status.status = 1; // Set the sending status
      // Assert Chip select  pin
      LATBbits.LATB10 = 0x00;  //
      WriteSPI(reg); // The first byte is the command byte
   }else{
      return 0; // Transaction in process
   }
}
char SPI_ReadCmd(char reg, char * d_ptr, unsigned char n_data)
{
   // status: 0: Idle; 1: Sending; 2: Receiving
   if(SPI_status.status == 0){
      SPI_max_bytes = n_data;
      SPI_buffer_ptr = d_ptr;
      SPI_byte_counter = 0;
      SPI_status.status = 2; // Set the sending status
      // Assert Chip select  pin
      LATBbits.LATB10 = 0x00;  //
      WriteSPI(reg); // The first byte is the command byte
   }else{
      return 0; // Transaction in process
   }
}

void startRF_TXRX(void)
{
   LATBbits.LATB11 = 0x01;  // Assert the CE pin on nRF24L01+
   IEC0bits.INT0IE = 1;		// Habilitar interrupcion externa
}

// Vectores de interrupcion
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{

	IFS0bits.T1IF = 0;	// clear interrupt flag	
	LATBbits.LATB15 = !LATBbits.LATB15;		//Toggle LED (D4)
	

}

void __attribute__((__interrupt__, no_auto_psv)) _INT0Interrupt(void)
{
	// Manejo de la interrupcion externa: RB7, PIN16
	
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
		LATBbits.LATB12 = !LATBbits.LATB12;		// On LED (D7) (inverted)
		U1TXREG = U1TX_buffer[U1TX_byte_counter];
		U1TX_byte_counter++;
	}
	else
	{
		IEC0bits.U1TXIE = 0; // Disable UART TX interrupt until new transmision time
		// Cambiar el estado del buffer del UART
		UART_status.status = 0; // Update to idle status
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
}


void __attribute__((__interrupt__, no_auto_psv)) _SPI1Interrupt (void)
{
   IFS0bits.SPI1IF = 0x00; // Clear interrupt flag
   // SPI1 transfer completed interrupt 
   switch(SPI_ststus.status)
   {
      case 1: // Transmiting
         // The first received byte is the STATUS register of nRF24L01+
         if(SPI_byte_counter == 0){
            SPI_status.data = ReadSPI1();
         }
         WriteSPI1(SPI_buffer_ptr[SPI_byte_counter] & 0x00ff);
         SPI_byte_counter++;
         if(SPI_byte_counter == SPI_max_bytes)){
            SPI_status.status = 0;  // Transmision complete, return to idle
            LATBbits.LATB10 = 0x01;  // SS pin
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
         if(SPI_byte_counter > SPI_max_bytes)){
            SPI_status.status = 0;  // Transmision complete, return to idle
            LATBbits.LATB10 = 0x01;  // SS pin
         }else{
            WriteSPI1(0x00); // Dummy write for clocking the incoming byte
         }
         break;
   }
}
