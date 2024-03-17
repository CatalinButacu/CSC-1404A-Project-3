#include <c8051F040.h>	// declaratii SFR
#include <uart1.h>
#include <Protocol.h>
#include <UserIO.h>

extern unsigned char TIP_NOD;			// tip nod initial: Nu Master, Nu Jeton

extern nod retea[];

extern unsigned char timeout;		// variabila globala care indica expirare timp de asteptare eveniment
//***********************************************************************************************************
void TxMesaj(unsigned char i);	// transmisie mesaj destinat nodului i
void bin2ascii(unsigned char ch, unsigned char *ptr);	// functie de conversie octet din binar in ASCII HEX

//***********************************************************************************************************
void TxMesaj(unsigned char i){					// transmite mesajul din buffer-ul i
	unsigned char sc, *ptr, ch, j;

																				// daca este un mesaj POLL_MES sau JET_MES (au aceeasi valoare)
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
 																				
																				// initializare pointer pe bufferul ASCII
																				// pune in bufasc adresa HW dest + '0'
																				// pune in bufasc adresa HW src in ASCII HEX
																				
																				// pune in bufasc tipul mesajului
																				
 																				// daca este un mesaj de date (USER_MES)
																					// pune in bufasc src
																					
																					// pune in bufasc dest
																					
																					// pune in bufasc lng date
																					
																					
																						// pune in bufasc datele
																					
																				
																			
																				// pune in bufasc SC
																				
																				// pune in bufasc CR
																				// pune in bufasc LF
	
																				// urmeaza transmisia octetului de adresa (mod MULTIPROC_ADRESA)
																				// valideaza Tx si Rx UART1
																				// validare Tx si Rx RS485
																				
																				// reinitializare pointer
																				// transmite adresa HW dest
	
																				// daca caracterul primit e diferit de cel transmis ...
																						// dezactivare Tx si Rx UART1
																						// dezactivare Tx si Rx RS485
																						// afiseaza Eroare coliziune
																						// asteapta WAIT msec
																						// termina transmisia (revine)
																				

																					// urmeaza transmisia octetilor de date (mod MULTIPROC_DATA)
																					// dezactivare Rx UART1
																					
																					// transmite restul caracterelor din bufferul ASCII, mai putin ultimul
																					
																					// transmite si ultimul caracter
																					// activare Rx UART1
	
																					//nodul master nu goleste bufferul
																					
																					// asteapta terminarea transmisiei/receptiei ultimului caracter
																					
																					// dezactivare Tx si Rx UART1
																					// dezactivare Tx si Rx RS485
}

//***********************************************************************************************************
void bin2ascii(unsigned char ch, unsigned char *ptr){	// converteste octetul ch in doua caractere ASCII HEX puse la adresa ptr
	unsigned char first, second;
	first = (ch & 0xF0)>>4;						// extrage din ch primul digit
	second = ch & 0x0F;								// extrage din ch al doilea digit
	if(first > 9) *ptr++ = first - 10 + 'A';	// converteste primul digit daca este litera
	else *ptr++ = first + '0';				// converteste primul digit daca este cifra
 	if(second > 9) *ptr++ = second - 10 + 'A';	// converteste al doilea digit daca este litera
	else *ptr++ = second + '0';				// converteste al doilea digit daca este cifra
}
