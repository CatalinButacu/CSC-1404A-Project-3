#include <c8051F040.h>	// declaratii SFR
#include <wdt.h>
#include <osc.h>
#include <port.h>
#include <uart0.h>
#include <uart1.h>
#include <lcd.h>
#include <keyb.h>
#include <Protocol.h>
#include <UserIO.h>

nod retea[NR_NODURI];					// retea cu 5 noduri

unsigned char STARE_COM = 0;		// starea initiala a FSA COM
unsigned char TIP_NOD	= 0;		  // tip nod initial: Slave sau No JET
unsigned char STARE_IO 	= 0;		// stare interfata IO - asteptare comenzi
unsigned char ADR_MASTER;				// adresa nod master - numai MS

extern unsigned char AFISARE;		// flag permitere afisare

//***********************************************************************************************************
void TxMesaj(unsigned char i);	// transmisie mesaj destinat nodului i
unsigned char RxMesaj(unsigned char i);		// primire mesaj de la nodul i

//***********************************************************************************************************
void main (void) {
	// variabile locale
	unsigned char i, found;	
	
	// dezactiveaza WDT
	WDT_Disable();	
	// initializeaza si selecteaza oscilatorul ales in osc.h
	SYSCLK_Init();	
	// initilizare UART1 - port comunicatie (TxD la P1.0 si RxD la P1.1)
	UART1_Init(NINE_BIT, BAUDRATE_COM);	
	// validare Tx si Rx UART1
	UART1_TxRxEN (1,1);	
	// conecteaza perifericele la pini (UART0, UART1) si stabileste tipul pinilor
 	PORT_Init ();	

	// initilizeaza LCD-ul
	LCD_Init();    							
	// initializare driver tastatura matriciala locala					
	KEYB_Init();										
	// initializare UART0  - conectata la USB-UART (P0.0 si P0.1)			
	UART0_Init(EIGHT_BIT, BAUDRATE_IO);		

	// initializare Timer 0
	Timer0_Init();  								

	// valideaza intreruperile
 	EA = 1;                         	
	// selecteaza pagina 0 SFR		
 	SFRPAGE = LEGACY_PAGE;          			
	
	// initializare structuri de date
	for(i = 0; i < NR_NODURI; i++)
	{		
		// initializeaza buffer gol pentru toate nodurile
		retea[i].full = 0;		
		// pune primul caracter in buffer-ele ASCII ":"				
		retea[i].bufasc[0] = ':';				
	}

	// Afiseaza meniul de comenzi
	Afisare_meniu();			   				
	
 	while(1) // bucla infinita
	{										
		switch(STARE_COM)
		{
			// nodul este slave, asteapta mesaj complet si corect de la master	
			case 0:		
				// asteapta un mesaj de la master
				switch(RxMesaj(ADR_NOD))
				{							
					case TMO:	
						// anunta ca nodul curent devine master
						Error("\n\rNod SLAVE -> MASTER!");  
						// nodul curent devine master
						TIP_NOD = MASTER;				
						// Afiseaza meniul de comenzi						
						Afisare_meniu(); 
						// trece in starea 2										
						STARE_NOD = 2;	
						// primul slave va fi cel care urmeaza dupa noul master										
						i = ADR_NOD;												
						break;

					case ROK:		// a primit un mesaj de la master, il afiseaza si trebuie sa raspunda
						STARE_NOD = 1; 
						Afisare_mesaj(); 
						break;	
					
					case POK:	
						STARE_COM = 1; 	
						break;		
					
					case CAN:	
						Error("\n\rMesaj incomplet!"); // afiseaza eroare Mesaj incomplet
						break;
					
					case TIP:	
						Error("\n\rTip mesaj necunoscut!"); // afiseaza eroare Tip mesaj necunoscut
						break;	
					
					case ESC:									
						Error("\n\rEroare SC!"); // afiseaza Eroare SC
						break;	
					
					default:	
						Error("\n\rEroare necunoscuta!"); // afiseaza cod eroare necunoscut, asteapta 1 sec
						break;	
				}
				break;

			// nodul este slave, transmite mesaj catre master	
			case 1:				
				// cauta sa gaseasca un mesaj util de transmis
				found = 0;  
				for(i = 0; i < NR_NODURI; i++){
					if(retea[i].full == 1){
							found = 1;
							break;
					}
				}																
				
				if(found) // daca gaseste un nod i
				{  
					// pune adresa HW dest este ADR_MASTER
					retea[i].bufbin.adresa_hw_dest = ADR_MASTER; 
					// transmite mesajul catre nodul i
					TxMesaj(i); 
				}
				else // daca nu gaseste, construieste un mesaj in bufferul ADR_MASTER
				{  
					// adresa HW dest este ADR_MASTER
					retea[ADR_MASTER].bufbin.adresa_hw_dest = ADR_MASTER;
					// adresa HW src este ADR_NOD 
					retea[ADR_MASTER].bufbin.adresa_hw_src = ADR_NOD;	
					// tip mesaj = POLL_MES
					retea[ADR_MASTER].bufbin.tipmes = POLL_MES;		
					// transmite mesajul din bufferul ADR_MASTER
					TxMesaj(ADR_MASTER);  
				}

				// trece in starea 0, sa astepte un nou mesaj de la master
				STARE_NOD = 0;				
				break;

			// tratare stare 2 si comutare stare	
			case 2:		
				// nodul este master, tratare stare 2 si comutare stare
				do {
					// selecteaza urmatorul slave (incrementeaza i)
					i = (++i) % NR_NODURI;	
				} while(i == ADR_NOD);

				// adresa HW dest este i	
				retea[i].bufbin.adresa_hw_dest = i;	

				// daca in bufferul i se afla un mesaj util, il transmite										
				if(retea[i].full == 1)  
				{
					TxMesaj(i);									
				}
				else  // altfel, construieste un mesaj de interogare in bufferul i
				{	
					// adresa HW src este ADR_NOD
					retea[i].bufbin.adresa_hw_src = ADR_NOD; 
					// tip mesaj = POLL_MES
					retea[i].bufbin.tipmes = POLL_MES;      
					// transmite mesajul din bufferul i
					TxMesaj(i);
				}				

				// trece in starea 3, sa astepte raspunsul de la slave-ul i						
				STARE_NOD = 3; 
				break;

			// nodul este master, asteapta mesaj de la slave
			case 3:			
				// nodul este slave, asteapta mesaj de la master	
				switch(RxMesaj(i)) // asteapta un raspuns de la slave i
				{						
					case TMO:	
						// afiseaza eroare Timeout nod i
						Error("\n\rTimeout nod ");
						UART0_Putch(i+'0');
						break;

					case ROK:	break;	// a primit un mesaj, il afiseaza
					case POK:	break;							

					case ERI:	
						// afiseaza Eroare incadrare
						Error("\n\rEroare incadrare!");
						break;	

					case ERA: 
						// afiseaza Eroare adresa
						Error("\n\rEroare adresa!");
						break;	

					case TIP:
						// afiseaza Tip mesaj necunoscut
						Error("\n\rTip mesaj necunoscut!");
						break;	

					case OVR:	
						// afiseaza Eroare suprapunere
						Error("\n\rEroare suprapunere!");
						break;	

					case ESC:
						// afiseaza Eroare SC
						Error("\n\rEroare SC!")
						break;	

					case CAN:
						// afiseaza mesaj incomplet
						Error("\n\rMesaj incomplet!")
						break;	

					case TEST:
						// afiseaza Eroare TEST
						Error("\n\rMesaj incomplet!")	
						break;				

					default:
						// afiseaza Eroare necunoscuta, apoi asteapta 1000ms
						Error("\n\rEroare necunoscuta!");
						break;	
				}
				// revine in starea 2 (a primit sau nu un raspuns de la slave, trece la urmatorul slave)
				STARE_NOD = 2;
				break;			
		}
		
		// apel functie interfata
		UserIO();							
	}
}
