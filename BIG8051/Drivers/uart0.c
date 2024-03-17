#include <C8051F040.h>                 // declaratii SFR
#include <uart0.h>
#include <osc.h>
#include <timer.h>

//******************************************************************************************
int UART0_Init(unsigned char mode, long int baudrate){	// initializare UART0 (mod si baudrate)

	unsigned char SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
		
	SFRPAGE = LEGACY_PAGE;
	
	if(mode == EIGHT_BIT){
		SCON0 = 0x52;     				// 1001 0010
	}
	else if(mode == NINE_BIT){	
		SCON0 =  0xD2;          		// 1101 0010
	}
	else return 0;							// mod eronat
	
	SSTA0 = 0x0F;   // 0000 1111 Timer 4 pentru Tx si Rx

 	SFRPAGE = TMR4_PAGE;

	switch (baudrate){
#if (OSC==XOSC)
		case 9600:	TMR4=RCAP4=0xFF5D;
					break;
		case 19200:	TMR4=RCAP4=0xFFAF;
					break;
		case 38400:	TMR4=RCAP4=0xFFD7;
					break;
		case 115200:TMR4=RCAP4=0xFFF2;
					break;

#elif (OSC==IOSC_MAX)

		case 9600:	TMR4=RCAP4=0xFF60;
					break;
		case 19200:	TMR4=RCAP4=0xFFB0;
					break;
		case 38400:	TMR4=RCAP4=0xFFD8;
					break;
		case 115200:TMR4=RCAP4=0xFFF3;
					break;

#elif (OSC==IOSC_MIN)
		case 9600:	TMR4=RCAP4=0xFFEC;
					break;
		case 19200:	TMR4=RCAP4=0xFFF6;
					break;
		case 38400:	TMR4=RCAP4=0xFFFB;
					break;
		case 115200: return 0;

#endif

		default: 	return 0; 		// wrong baudrate
	}
										//Stop Timer4; Clear TF4;
	TMR4CN = 0x00;                     	// TF4 EXF4 - - EXEN4 TR4 C/T4 CP/RL4 
										// use SYSCLK as timebase
										// Stop Timer4; Clear TF4;
	TMR4CF = 0x08;						// - - - T4M1 T4M0 TOG4 T3O4 DCEN
	TR4 = 1;                           	// start Timer4

	SFRPAGE = SFRPAGE_save;
	return 1;
}

//******************************************************************************************
void UART0_Putch(unsigned char ch){		// transmisie caracter
  unsigned char SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	SFRPAGE = LEGACY_PAGE;	

	while(!TI0);
	TI0 = 0;
	SBUF0 = ch;

	SFRPAGE = SFRPAGE_save;
}

//******************************************************************************************
void UART0_Putstr(unsigned char *ptr){		// transmisie sir de caractere
 	while(*ptr) UART0_Putch(*ptr++);
}

//******************************************************************************************
unsigned char UART0_Getch(unsigned char echo){	// receptie caracter (cu sau fara retransmisie in ecou)
  unsigned char ch, SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	
	SFRPAGE = LEGACY_PAGE;	

	while(!RI0);
	RI0=0;
	ch = SBUF0;
	
	if(echo){
		while (!TI0);
		TI0=0;
		SBUF0 = ch;
	}
	return ch;
}

//******************************
// Verificare receptie caracter
//******************************
unsigned char UART0_Check(void){
  unsigned char check, SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	SFRPAGE = LEGACY_PAGE;	

	if(RI0) check = 1;					// testeaza daca s-a primit un nou caracter pe linia seriala
	else check = 0;
	
	SFRPAGE = SFRPAGE_save;	// reface pagina initiala

	return check;							// returneaza rezultatul testului
}


