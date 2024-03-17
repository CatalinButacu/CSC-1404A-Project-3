#include <c8051F040.h>	// declaratii SFR
#include <uart1.h>
#include <Protocol.h>
#include <UserIO.h>

extern unsigned char TIP_NOD;			// tip nod initial: Nu Master, Nu Jeton

extern nod retea[];

extern unsigned char timeout;		// variabila globala care indica expirare timp de asteptare eveniment
//***********************************************************************************************************
void TxMesaj(unsigned char i);	// transmisie mesaj destinat nodului i

//***********************************************************************************************************
void TxMesaj(unsigned char i){					// transmite mesajul din buffer-ul i
	unsigned char sc, j;
																			
																				// daca este un mesaj de interogare (POLL_MES) sau JET_MES (au aceeasi valoare)
																					// calculeaza direct sc
																				
																				// altfel...
																					// initializeaza SC	cu adresa HW a nodului destinatie
																					// ia in calcul adresa_hw_src
																					// ia in calcul tipul mesajului
																					// ia in calcul adresa nodului sursa al mesajului
																					// ia in calcul adresa nodului destinatie al mesajului
																					// ia in calcul lungimea datelor
																					
																					// ia in calcul datele
																					// stocheaza suma de control
 																			
																				// transmite adresa HW a nodului destinatie
																				// transmite adresa HW a nodului sursa
																				// transmite tipul mesajului
	
																				// Daca mesajul este USER_MES ...
																					// transmite adresa nodului sursa mesaj
																					// transmite adresa nodului destinatie mesaj
																					// transmite lungimea mesajului
    																				 	
																						// transmite octetii de date
																				
																				// transmite suma de control
	
																				// masterul nu goleste buffer-ul

}

//***********************************************************************************************************
