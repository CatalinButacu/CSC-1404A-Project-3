#include <C8051F040.h>                 // declaratii SFR
#include <uart1.h>
#include <osc.h>
#include <timer.h>

//******************************************************************************************
int UART1_Init(unsigned char mode, long int baudrate){	// initializare UART1 (mod si baudrate)

// initializeaza UART1 in mod asincron, cu 8 biti/caracter, cu viteza de 9600 Baud
	unsigned char SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
		
	SFRPAGE = UART1_PAGE;
	
	if(mode == EIGHT_BIT){
		SCON1 = 0x40;     				// 0100 0000
	}
	else if(mode == NINE_BIT){	
		SCON1 =  0xC0;          		// 1100 0000
	}
	else return 0;							// mod eronat
	
	SFRPAGE = LEGACY_PAGE;

	switch (baudrate){
#if (OSC==XOSC)
		case 9600:	TL1=TH1=0x93;
								CKCON |= 0x00;	// Timer 1 source = SYSCLK/12
								break;
		case 19200:	TL1=TH1=0x5D;
								CKCON |= 0x01;	// Timer 1 source = SYSCLK/4
								break;
		case 38400:	TL1=TH1=0xAF;
								CKCON |= 0x01;	// Timer 1 source = SYSCLK/4
								break;
		case 115200:TL1=TH1=0x93;
								CKCON |= 0x10;	// Timer 1 source = SYSCLK
								break;

#elif (OSC==IOSC_MAX)

		case 9600:	TL1=TH1=0x96;
								CKCON |= 0x00;	// Timer 1 source = SYSCLK/12
								break;
		case 19200:	TL1=TH1=0x60;
								CKCON |= 0x01;	// Timer 1 source = SYSCLK/4
								break;
		case 38400:	TL1=TH1=0xB0;
								CKCON |= 0x01;	// Timer 1 source = SYSCLK/4
								break;
		case 115200:TL1=TH1=0x96;
								CKCON |= 0x10;	// Timer 1 source = SYSCLK
								break;

#elif (OSC==IOSC_MIN)
		case 9600:	TL1=TH1=0x60;
								CKCON |= 0x10;	// Timer 1 source = SYSCLK
								break;
		case 19200:	TL1=TH1=0xB0;
								CKCON |= 0x10;	// Timer 1 source = SYSCLK
								break;
		case 38400:	TL1=TH1=0xD8;
								CKCON |= 0x10;	// Timer 1 source = SYSCLK
								break;
		case 115200:TL1=TH1=0xF3; 
								CKCON |= 0x10;	// Timer 1 source = SYSCLK
								break;

#endif

		default: 	return 0; 		// wrong baudrate
	}
	
	TMOD &= 0x0F;
	TMOD |= 0x20;	// 0010 0000 mod 2 auto-reload
	TR1 = 1;			// porneste T1	
	SFRPAGE = SFRPAGE_save;
	return 1;
}	
	
//******************************************************************************************
void UART1_Putch(unsigned char ch){		// transmisie caracter
 unsigned char SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	SFRPAGE = UART1_PAGE;	

	while(!TI1);
	TI1 = 0;
	SBUF1 = ch;

	SFRPAGE = SFRPAGE_save;
}

//******************************************************************************************
void UART1_PutchPE(unsigned char ch){		// transmisie caracter
 unsigned char SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	SFRPAGE = UART1_PAGE;	

	while(!TI1);
	TI1 = 0;
	ACC = ch;
	TB81 = P;
	SBUF1 = ch;

	SFRPAGE = SFRPAGE_save;
}


//******************************************************************************************
void UART1_Putstr(unsigned char *ptr){		// transmisie sir de caractere
 	while(*ptr) UART1_Putch(*ptr++);
}

//******************************************************************************************
void UART1_Flush(void){							// goleste bufferul de receptie UART1
 unsigned char SFRPAGE_save = SFRPAGE; 	// Save the current SFRPAGE
	SFRPAGE = UART1_PAGE;	
	
	do RI1 = 0;
	while(RI1);
	
	SFRPAGE = SFRPAGE_save;
}

//******************************************************************************************
void UART1_PutstrPE(unsigned char *ptr){		// transmisie sir de caractere
 	while(*ptr) UART1_PutchPE(*ptr++);
}

//******************************************************************************************
unsigned char UART1_Getch(unsigned char echo){	// receptie caracter (cu sau fara retransmisie in ecou)
  unsigned char ch, SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	
	SFRPAGE = UART1_PAGE;	

	while(!RI1);
	RI1 = 0;
	ch = SBUF1;
	
	if(echo){
		while (!TI1);
		TI1=0;
		SBUF1 = ch;
	}
	SFRPAGE = SFRPAGE_save;
	return ch;
}

//******************************************************************************************
extern int timeval;
unsigned char timeout;

unsigned char UART1_Getch_TMO(int ms){		// receptie caracter cu timeout
  unsigned char ch, SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	TR0 = 1;											// porneste timerul
		
	SFRPAGE = UART1_PAGE;

	timeout = 0;									// pune initial timeout = 0
 	timeval = ms;									// initializeaza timeval
	while(timeval){
		if(RI1){										// asteapta pana cand timeval = 0 sa vina un caracter
			timeval = 0;							// atunci cand vine, pune timeval=0
			
			ch = SBUF1;
			RI1 = 0;
			SFRPAGE = SFRPAGE_save;	

			return ch;				// returneaza caracterul sosit
		}
	}
	timeout = 1;									// daca nu a venit caracterul, pune timeout = 1

	SFRPAGE = SFRPAGE_save;	

	return 0;
} 

//******************************************************************************************
void UART1_MultiprocMode(unsigned char mod){	// configureaza modul multiprocesor adresa/data
  unsigned SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE
	
	SFRPAGE = UART1_PAGE;
	
	if(mod == MULTIPROC_ADRESA){
		TB81 = 1; 		// transmite caractere cu al 9-lea bit 1
		MCE1 = 1;		// primeste doar caractere cu al 9-lea bit 1
	}
	if(mod == MULTIPROC_DATA){
		TB81 = 0;			// transmite caractere cu al 9-lea bit 0
		MCE1 = 0;			// primeste si caractere cu al 9-lea bit 0
	}
	SFRPAGE = SFRPAGE_save;	
}

//******************************************************************************************
void UART1_TxRxEN(unsigned char TxEN, unsigned char RxEN){	// valideaza/inhiba transmitatorul si receptorul UART1
 unsigned char SFRPAGE_save = SFRPAGE; 	// Save the current SFRPAGE
	SFRPAGE = UART1_PAGE;	
	
	if(TxEN) TI1 = 1;
	else TI1 = 0;
	if(RxEN) REN1 = 1;
	else REN1 =0;
	
	SFRPAGE = SFRPAGE_save;
}


//******************************************************************************************
void UART1_RS485_XCVR(unsigned char TxEN, unsigned char RxEN){	// Temele 3 si 4: validare emitator si receptor linie RS485
// valideaza sau dezactiveaza receptorul de linie RS485
	
	if(TxEN) P12 = 1;
	else P12 = 0;
	if(RxEN) P13 = 0;
	else P13 = 1;
}

