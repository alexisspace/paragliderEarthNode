

#include <p33FJ12GP202.h>
#include <i2c.h>
#include <libq.h>
#include <dsp.h>
#include <string.h>
// the above include path may be different for each user.  If a compile
// time error appears then check the path for the file above and edit
// the include statement above.

//#define XTFREQ          7370000         //FRC frequency 
#define FCY             39613000        //Instruction Cycle Frequency

#define BAUDRATE         9600		      
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 


// Registros del nRF24L01+




#define N_SAMPLE_BYTES		14
#define USART_TX_BUFFER_SIZE	17
#define N_CFG_REG			11



// Constantes de la implementacion
#define DEG2RAD				17.4533e-003


// Prototipos de funciones


// Variables globales
unsigned char buffer[N_SAMPLE_BYTES], U1TX_buffer[USART_TX_BUFFER_SIZE];
unsigned char data, U1TX_byte_counter;



int main(void)
{
	

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
	AD1PCFGL = 0x03C0;		// Make analog pins digital
	TRISB = 0x0FFF;			// Config LED's as output and U1RX as Input
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
	INTCON2bits.INT0EP = 0;		// Interrupcion en flanco ascendente
	IFS0bits.INT0IF = 0;		// Clear flag
//	IEC0bits.INT0IE = 1;		// Habilitar interrupcion
	CNPU2bits.CN23PUE = 0;	// pull-up on RB7: 0 = Disable; 1 = Enable



// Configuar Timer1. Habilitar interrupcion
	TMR1 = 0;				// clear timer 1
	PR1 = 0x78E4;//0x3C72;			// interrupt every 100ms
	T1CON = 0x8030;			// 1:256 prescale, start TMR1
	IFS0bits.T1IF = 0;		// clr interrupt flag
//	IEC0bits.T1IE = 1;		// set interrupt enable bit

// Activar LED que indica entrada en el lazo principal del programa
	LATBbits.LATB13 = 0x00;		// On LED (D6)
	while(1)
	{
		while (_U1RXIF == 0);			// Wait and Receive One Character
		data = U1RXREG;
		_U1RXIF = 0;					// Clear UART RX Interrupt Flag
		
		while(!U1STAbits.TRMT);	// Esperar a que se transmita el byte antes de leer el otro de la IMU
		U1TXREG = data;

	}//while(1)

	return 0;
}


// Vectores de interrupcion
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{

	IFS0bits.T1IF = 0;	// clear interrupt flag	
	LATBbits.LATB15 = !LATBbits.LATB15;		//Toggle LED (D4)
	
	// Construir mensaje
	// Copiar estimaciones actuales al buffer de salida del UART
	//memcpy(U1TX_buffer, gyro_data, USART_TX_BUFFER_SIZE-1);
	//U1TX_buffer[USART_TX_BUFFER_SIZE-1] = 0xFF;
	
	//memcpy(U1TX_buffer, q, USART_TX_BUFFER_SIZE-1);
	U1TX_buffer[USART_TX_BUFFER_SIZE-1] = 0xFF;

	// Iniciar transmision de mensaje por el UART
	U1TX_byte_counter = 1;		// Inicializar valor de contador
	U1TXREG = U1TX_buffer[0];
	IEC0bits.U1TXIE = 1; // Enable UART TX interrupt

}

void __attribute__((__interrupt__, no_auto_psv)) _INT0Interrupt(void)
{
	// Manejo de la interrupcion externa: RB7, PIN16



	LATBbits.LATB14 = !LATBbits.LATB14;		// On LED5 (D5) (inverted)
	IFS0bits.INT0IF = 0;		// Clear flag
}

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
	// Aqui se transmite los datos de orientación por el puerto serial

	
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
	}
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
}

