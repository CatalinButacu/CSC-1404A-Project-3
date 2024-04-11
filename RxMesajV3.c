#include <Protocol.h>
#include <UserIO.h>
#include <c8051F040.h>  					// declaratii SFR
#include <uart1.h>

extern nod retea[];  							// reteaua Master-Slave, cu 5 noduri

extern unsigned char TIP_NOD;     // tip nod
extern unsigned char ADR_MASTER;  // adresa nodului master

extern unsigned char timeout;  		// variabila globala care indica expirare timp de asteptare eveniment
//***********************************************************************************************************
unsigned char RxMesaj(unsigned char i);  			// primire mesaj de la nodul i
unsigned char ascii2bin(unsigned char *ptr);  // functie de conversie 2 caractere ASCII HEX in binar

//***********************************************************************************************************
unsigned char RxMesaj(unsigned char i) {  		// receptie mesaj
  unsigned char j, sc, ch, adresa_hw_src, screc, src, dest, lng, tipmes, *ptr;

  UART1_TxRxEN(0, 1);      										// dezactivare Tx, validare RX UART1
  UART1_RS485_XCVR(0, 1);  										// dezactivare Tx, validare RX RS485
  UART1_MultiprocMode(MULTIPROC_ADRESA);  		// receptie doar octeti de adresa (mod MULTIPROC_ADRESA)

  if (TIP_NOD == JETON)  											// Daca nodul este master sau detine jetonul ...
  {
    ch = UART1_Getch_TMO(WAIT);  										// M: asteapta cu timeout primul caracter al raspunsului de la slave
    if (timeout == 1)  															// M: timeout, terminare receptie
      return TMO;
    retea[i].full = 0;  														// M: raspunsul de la slave vine, considera ca mesajul anterior a fost transmis cu succes

    if (ch != ADR_NOD + '0')  											// M: adresa HW ASCII gresita, terminare receptie
      return ERA;
  } else  																		// Daca nodul este slave...
  {
    do {
      ch = UART1_Getch_TMO(2 * WAIT + ADR_NOD * WAIT);
      if (timeout == 1)         										// S: asteapta cu timeout primirea primului caracter al unui mesaj
        return TMO;  																// S: timeout, terminare receptie
    } while (ch != ADR_NOD + '0');  
  }
																										// S: iese doar cand mesajul era adresat acestui nod

  UART1_MultiprocMode(MULTIPROC_DATA);  			// receptie octeti de date
  ptr = retea[ADR_NOD].bufasc;          			// M+S: pune in bufasc restul mesajului ASCII HEX
  *ptr = ADR_NOD + '0';

  do {
    *(++ptr) = UART1_Getch_TMO(5);
    if (timeout == 1)  															// M+S: timeout, terminare receptie
      return CAN;
  } while (*ptr == 0x0A);

  ptr = retea[ADR_NOD].bufasc;  							// M+S: reinitializare pointer in bufferul ASCII
  screc = *ptr++ - '0';         							// M+S: initializeaza screc cu adresa HW dest

  adresa_hw_src = ascii2bin(ptr);  						// M+S: determina adresa HW src

  ptr += 2;  																	// M+S: aduna adresa HW src
  screc += adresa_hw_src;

  if (TIP_NOD == NOJET)  											// Slave actualizeaza adresa Master
    ADR_MASTER = adresa_hw_src;
  tipmes = ascii2bin(ptr);  									// M+S: determina tipul mesajului
  ptr += 2;
  if (tipmes > 1)  														// M+S: tip mesaj eronat, terminare receptie
    return TIP;    																	// M+S: ia in calcul in screc tipul mesajului
  screc += tipmes;

  if (tipmes != USER_MES)  										// M+S: Daca mesajul este unul de date (USERMES)
  {
    retea[ADR_NOD].bufbin.adresa_hw_src = adresa_hw_src;
    sc = ascii2bin(ptr);
    if (sc == screc)
      return POK;
    else
      return ESC;
  } else {
    src = ascii2bin(ptr);  										// M+S: determina sursa mesajului
    ptr += 2;
    screc += src;  														// M+S: ia in calcul in screc adresa src

    dest = ascii2bin(ptr);  									// M+S: determina destinatia mesajului
    ptr += 2;
    screc += dest;  													// M+S: ia in calcul in screc adresa dest

    if (TIP_NOD == JETON) 										// Daca nodul este master...
      if (retea[dest].full == 1) 									 	// M: bufferul destinatie este deja plin, terminare receptie
        return OVR;

    lng = ascii2bin(ptr);  													// M+S: determina lng
    ptr += 2;
    screc += lng;																		// M+S: ia in calcul in screc lungimea datelor

    if (TIP_NOD == JETON)  													// Daca nodul este master...
    {
      retea[dest].bufbin.adresa_hw_src = ADR_NOD;  				// M: stocheaza in bufbin adresa HW src
      retea[dest].bufbin.tipmes = tipmes;  								// M: stocheaza in bufbin tipul mesajului
      retea[dest].bufbin.src = src;  											// M: stocheaza in bufbin adresa nodului sursa al mesajului
      retea[dest].bufbin.dest = dest;  										// M: stocheaza in bufbin adresa nodului destinatie al mesajului
      retea[dest].bufbin.lng = lng;  											// M: stocheaza lng

      for (j = 0; j < retea[ADR_NOD].bufbin.lng; j++)  					// M: determina un octet de date
      {
        retea[ADR_NOD].bufbin.date[j] = ascii2bin(ptr);  				// M: ia in calcul in screc octetul de date
        ptr += 2;
        screc += retea[ADR_NOD].bufbin.date[j];  								// M: determina suma de control
      }
      sc = ascii2bin(ptr);  															// M: pune sc in bufbin

      if (sc == screc)  																	// M: eroare SC, terminare receptie
      {
        retea[dest].full = 1;  																	// M: mesaj corect, marcare buffer plin
        return ROK;
      } else
        return ESC;
    } else 
		{  																										// Daca nodul este slave ...
      retea[ADR_NOD].bufbin.src = src;  												// S: stocheaza la destsrc codul nodului sursa al mesajului
      retea[ADR_NOD].bufbin.lng = lng;  												// S: stocheaza lng

      for (j = 0; j < retea[ADR_NOD].bufbin.lng; j++)  					// S: determina un octet de date
      {
        retea[ADR_NOD].bufbin.date[j] = ascii2bin(ptr);  							// S: ia in calcul in screc octetul de date
        ptr += 2;
        screc += retea[ADR_NOD].bufbin.date[j];  											// S: determina suma de control
      }
    }

    sc = ascii2bin(ptr);  														// S: mesaj corect, marcare buffer plin

    if (sc == screc)  																// M: eroare SC, terminare receptie
    {
      retea[dest].full = 1;
      return ROK;
    } else
      return ESC;
  }
}

//***********************************************************************************************************
unsigned char ascii2bin(
    unsigned char *ptr) {  // converteste doua caractere ASCII HEX de la adresa ptr
  unsigned char bin;

  if (*ptr > '9') bin = (*ptr++ - 'A' + 10) << 4;  // contributia primului caracter daca este litera
  else bin = (*ptr++ - '0') << 4;  // contributia primului caracter daca este cifra
  if (*ptr > '9') bin += (*ptr++ - 'A' + 10);  // contributia celui de-al doilea caracter daca este litera
  else bin += (*ptr++ - '0');  // contributia celui de-al doilea caracter daca este cifra
  return bin;
}


