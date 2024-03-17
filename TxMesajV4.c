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

																				// daca este un mesaj de interogare POLL_MES sau JET_MES (au aceeasi valoare)
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
 																			
	
																				// urmeaza transmisia octetului de adresa (mod MULTIPROC_ADRESA)
																				// validare Tx si Rx UART1
																				// validare Tx si Rx RS485

																				// transmite adresa HW a nodului dest
																				
																				// asteapta sa receptioneze caracterul transmis
																				// daca caracterul primit e diferit de cel transmis sau a aparut timeout ...
																						// dezactivare Tx UART1
																						// dezactivare Tx RS485
																						// afiseaza Eroare coliziune
																						// asteapta WAIT msec
																						// termina transmisia (revine)
																				

																				// urmeaza transmisia octetilor de date (mod MULTIPROC_DATA)
																				// dezactivare Rx UART1

																				// transmite adresa HW a nodului sursa
																				// transmite tipul mesajului
	
																				// Daca mesajul este de date ...
																					// transmite adresa nodului sursa mesaj
																					// transmite adresa nodului dest mesaj
																					// transmite lungimea mesajului
    																				
																						// transmite octetii de date
																			
																				// transmite suma de control

																				// activare Rx UART1
																				
																				// masterul nu goleste bufferul
	
																				// asteapta transmisia/receptia ultimului caracter

																				// dezactivare Tx si RX UART1
																				// dezactivare Tx si Rx RS485
}

//***********************************************************************************************************
