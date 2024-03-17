#include <c8051F040.h>	// declaratii SFR
#include <uart1.h>
#include <Protocol.h>
#include <UserIO.h>

extern nod retea[];						// reteaua Master-Slave, cu 5 noduri

extern unsigned char TIP_NOD;			// tip nod
extern unsigned char ADR_MASTER;	// adresa nodului master

extern unsigned char timeout;		// variabila globala care indica expirare timp de asteptare eveniment
//***********************************************************************************************************
unsigned char RxMesaj(unsigned char i);				// primire mesaj de la nodul i

//***********************************************************************************************************
unsigned char RxMesaj(unsigned char i){					// receptie mesaj															   
	unsigned char j, ch, sc, adresa_hw_src, screc, src, dest, lng, tipmes;
	
																				// Daca nodul este master sau detine jetonul ...
																					// M: asteapta cu timeout raspunsul de la slave
																					// M: timeout, terminare receptie, devine master

																					// M: raspunsul de la slave a venit, considera ca mesajul anterior a fost transmis cu succes	
																					
																						// M: ignora restul mesajului
																						
																						// M: adresa HW gresita, terminare receptie
																			
	
																			
																			
																						// S: ignora mesajul in curs de transmisie
																						
																						// S: inceput mesaj nou, dupa o pauza in transmisie
			
																						// S: masterul e oprit, nodul va deveni master sau se va regenera jetonul
																					
																					// S: iese doar atunci cand mesajul ii este adresat acestui nod slave
 																				
																				// M+S: initializeaza screc cu adresa HW dest (ADR_NOD)
	
																				// M+S: Asteapta cu timeout receptia adresei HW sursa
																				// M+S: mesaj incomplet
	
																				// M+S: ia in calcul in screc  adresa hw src

																				// nodul slave actualizeaza adresa master
	
																				// M+S: Asteapta cu timeout receptia codului functiei
																				// M+S: mesaj incomplet

																				// M+S: tip mesaj necunoscut?
																					// M+S: ignora restul mesajului
																					
																					// M+S: tip mesaj eronat, terminare receptie
																				
																				// M+S: ia in calcul in screc tipul mesajului
	
																				// M+S: Daca mesajul este USER_MES
																					// M+S: asteapta cu timeout adresa nodului sursa
																					
																					// M+S: ia in calcul in screc adresa src
																					// M+S: asteapta cu timeout adresa nodului destinatie
																					
																					// M+S: ia in calcul in screc adresa dest
		
																					// Daca nodul este master...
																						
																							// M: ignora restul mesajului
																							
																														// M: bufferul destinatie este deja plin, terminare receptie
																						
																				
																					// M+S: asteapta cu timeout receptia lng
																					
																												// M+S: ia in calcul in screc lungimea datelor
		
																					// Daca nodul este master...
																						// M: stocheaza adresa HW sursa	= ADR_NOD
																						// M: stocheaza tipul mesajului	
																						// M: stocheaza la src codul nodului sursa al mesajului	
																						// M: stocheaza la dest codul nodului destinatie a mesajului	
																						// M: stocheaza lng
																							
																							// M: asteapta cu timeout un octet de date
																							
																							// M: ia in calcul in screc datele
																							
																						
																						// M: Asteapta cu timeout receptia sumei de control
																						

																						
																							// M: mesaj corect, marcare buffer plin
																							
																					
																						// M: eroare SC, terminare receptie
																															
																					// nodul este slave
																						// S: stocheaza la src codul nodului sursa al mesajului	
																						// S: stocheaza lng
																						
																						// S: asteapta cu timeout un octet de date
																							
																							// S: ia in calcul in screc datele
																							
																					
																						// S: Asteapta cu timeout receptia sumei de control
																						

																						
																			// S: mesaj corect, marcare buffer plin
																							
																						
																						// S: eroare SC, terminare receptie
																					
																			
																				
																					// memoreaza de la cine a primit jetonul
																					// M+S: Asteapta cu timeout receptia sumei de control
																					
																					
																					// M+S: eroare SC, terminare receptie
																					
			return TMO;			// simuleaza lipsa unui raspuns																	
}


//***********************************************************************************************************


