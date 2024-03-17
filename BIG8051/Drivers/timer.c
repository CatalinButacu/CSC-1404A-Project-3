#include <C8051F040.h>                 // declaratii SFR
#include <timer.h>
#include <osc.h>

int timeval;						// contor numar de ms decrenentat in functia de intrerupere Timer 0

//***********************************************************************************************************
void Timer0_Init(void){	// initializare timer 0
	TMOD |= 0x01;					// Timer 0 modul 1
	TL0 = 0xDD;
	TH0 = 0xF7;						// Intarziere 1 ms (TH0:TL0 = 65536-2083 = 0xF7DD la 25 MHz, cu factor de divizare 12)
	TR0 = 1;							// Pornire Timer 0
	ET0 = 1;							// Validare intreruperi Timer 0
	EA = 1;							// validare globala intreruperi
}

//***********************************************************************************************************
void Timer0IntHandler (void) interrupt 1 {		// rutina intrerupere timer 0
	TL0 = 0xDD;
	TH0 = 0xF7;							// reincarca numaratorul pentru 1 ms
	if(timeval) timeval--;			// decrementeaza timeval daca este diferit de 0
	else TR0 = 0;						// daca timeval=0, opreste timerul 0
}
 
//***********************************************************************************************************
void Delay(int ms){				  // intarziere programabila
	timeval = ms;					   	// initializeaza timeval
	TR0 = 1;						   		// porneste timerul 0
	while(timeval);				   	// asteapta pana timeval=0 (decrementat in functia de intrerupere)
}

unsigned char T2flag, T3flag, T4flag;

//-----------------------------------------------------------------------------
// Timer2_Init ()
//-----------------------------------------------------------------------------
//
// Valoare returnata : niciuna
// Parametri   : nu
//
// Configurea Timer2 in modul 16-bit auto-reload si genereaza o intrerupere la depasire
// dupa o intarziere programabila.
//
//-----------------------------------------------------------------------------

void Timer2_Init (unsigned int counts)
{
   unsigned char SFRPAGE_save = SFRPAGE; 	// Salveaza SFRPAGE

 	SFRPAGE = TMR2_PAGE;							// Selecteaza TMR2_PAGE

   	TMR2CN = 0x00;                      // Stop Timer2; Sterge TF2;
                                       	// mod auto-reload

   	TMR2CF = 0x00;                      // foloseste SYSCLK/12, cu incrementare

   	RCAP2 = 65536-counts;               // valoarea de reincarcare
   	TMR2 = 0xffff;                      // reincarca imediat
  	
   	ET2 = 1;                            // validare intrerruperi Timer2

   	TR2 = 1;                            // Start Timer2
		
		T2flag = 0;									// flag folosit pentru contorizare intreruperi

	SFRPAGE = SFRPAGE_save;             	// Reface SFRPAGE	
}

//------------------------------------------------------------------------------------
// Timer3_Init
//------------------------------------------------------------------------------------
//
// Configurea Timer3 in modul 16-bit auto-reload si genereaza o intrerupere la depasire
// dupa o intarziere programabila.
//
//-----------------------------------------------------------------------------

void Timer3_Init (unsigned int counts)
{
   unsigned char SFRPAGE_save = SFRPAGE; // Salveaza SFRPAGE
 	SFRPAGE = TMR3_PAGE;						// Selecteaza TMR3_PAGE
	
	TMR3CN = 0x00;             	// TF3 EXF3 - - EXEN3 TR3 C/T3 CP/RL3 
												// Stop Timer3; Sterge TF3;
												
	TMR3CF = 0x00;						// - - - T3M1 T3M0 TOG3 T3OE DCEN
												// foloseste SYSCLK/12

   RCAP3   = 65536-counts;             // valoarea de reincarcare
   TMR3    = 0xffff;                  	// reincarca imediat
	
   EIE2   |= 0x01;                    	// validare intreruperi Timer3
   TR3 = 1;                           	// start Timer3

	T3flag = 0;									// flag folosit pentru contorizare intreruperi

	SFRPAGE = SFRPAGE_save;            	// Reface SFRPAGE	
}

//------------------------------------------------------------------------------------
// Timer4_Init
//------------------------------------------------------------------------------------
//
// Configurea Timer4 in modul 16-bit auto-reload si genereaza o intrerupere la depasire
// dupa o intarziere programabila.
//
//-----------------------------------------------------------------------------

void Timer4_Init (unsigned int counts)
{
   unsigned char SFRPAGE_save = SFRPAGE; // Salveaza SFRPAGE
 	SFRPAGE = TMR4_PAGE;						// Selecteaza TMR4_PAGE
	
	TMR4CN = 0x00;             	// TF4 EXF4 - - EXEN4 TR4 C/T4 CP/RL4 
												// Stop Timer4; Sterge TF4;
												
	TMR4CF = 0x00;						// - - - T4M1 T4M0 TOG4 T4OE DCEN
												// foloseste SYSCLK/12

   RCAP4   = 65536-counts;             // valoarea de reincarcare
   TMR4    = 0xffff;                  	// reincarca imediat
	
   EIE2   |= 0x04;                    	// validare intreruperi Timer4
   TR4 = 1;                           	// start Timer4

	T4flag = 0;									// flag folosit pentru contorizare intreruperi

	SFRPAGE = SFRPAGE_save;            	// Reface SFRPAGE	
}

