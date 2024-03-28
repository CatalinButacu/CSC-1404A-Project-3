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
void TxMesaj(unsigned char i)
	{					// transmite mesajul din buffer-ul i
	unsigned char sc, *ptr, ch, j;

	if (retea[i].bufbin.tipmes=='0')    // daca este un mesaj POLL_MES sau JET_MES (au aceeasi valoare)
		{						
			sc = retea[i].bufbin.adresa_hw_dest;						// calculeaza direct sc
			sc = sc+ retea[i].bufbin.adresa_hw_src;		//aduna adresa_hw_src retea[i].bufbin.sc=sc;	
			retea[i].bufbin.sc = sc;
		}														
		else	 // altfel...
		{					      
			sc = retea[i].bufbin.adresa_hw_dest;		// initializeaza SC	cu adresa HW a nodului destinatie
			sc = sc + retea[i].bufbin.adresa_hw_src;					// ia in calcul adresa_hw_src
			sc = sc + retea[i].bufbin.tipmes;					// ia in calcul tipul mesajului
			sc = sc + retea[i].bufbin.src;						// ia in calcul adresa nodului sursa al mesajului
			sc = sc + retea[i].bufbin.dest;						// ia in calcul adresa nodului destinatie al mesajului
			sc = sc + retea[i].bufbin.lng;						// ia in calcul lungimea datelor
			sc = sc + retea[i].bufbin.date[retea[i].bufbin.lng-1];					// ia in calcul datele
			retea[i].bufbin.sc = sc;				// stocheaza suma de control
		}					
	ptr = retea[i].bufasc			// initializare pointer pe bufferul ASCII
	*ptr++ = retea[i].bufbin.adresa_hw_dest + '0';// pune in bufasc adresa HW dest + '0'
	bin2ascii(retea[i].bufbin.adresa_hw_src, ptr);
	ptr += 2;					// pune in bufasc adresa HW src in ASCII HEX
	bin2ascii(retea[i].bufbin.tipmes, ptr);
	ptr += 2;					// pune in bufasc tipul mesajului
	if(retea[i].bufbin.tipmes=='1')      // daca este un mesaj de date (USER_MES)
	{					
		retea[i].bufasc.src=retea[i].bufin.src;				// pune in bufasc src
		retea[i].bufasc.dest=retea[i].bufin.dest;		// pune in bufasc dest
		retea[i].bufasc.lng=retea[i].bufin.lng;		// pune in bufasc lng date
		for (j = 0; j<lng; j++)
		{
			retea[i].bufasc.date[j] = retea[i].bufin.date[j];		// pune in bufasc datele
		}				
	}
	retea[i].bufasc.sc = retea[i].bufin.sc;					// pune in bufasc SC
	retea[i].bufasc.CR = 0x0d;					// pune in bufasc CR
	retea[i].bufasc.LF = 0x0a;					// pune in bufasc LF
	
	UART1_MultiprocMode(MULTIPROC_ADRESA);	// urmeaza transmisia octetului de adresa (mod MULTIPROC_ADRESA)
	UART1_TxRxEN(1, 1);			// valideaza Tx si Rx UART1
	UART1_RS485_XCVR(1, 1);			// validare Tx si Rx RS485
																				
	ptr = retea[i].bufasc;			// reinitializare pointer
	UART1_Putch(*ptr);			// transmite adresa HW dest
	
	if(UART1_Getch_TMO(2) != *ptr || Timeout)	// daca caracterul primit e diferit de cel transmis ...
	{		
		UART1_TxRxEN(0, 0);					// dezactivare Tx si Rx UART1
		UART1_RS485_XCVR(0, 0);					// dezactivare Tx si Rx RS485
		Error("Detectie coliziune!");				// afiseaza Eroare coliziune
		Delay(WAIT);						// asteapta WAIT msec
		return;				// termina transmisia (revine)
	}														
					  
	UART1_MultiprocMode(MULTIPROC_DATA);	// urmeaza transmisia octetilor de date (mod MULTIPROC_DATA)
	UART1_TxRxEN(1, 0);			// dezactivare Rx UART1
	do
	{
		UART1_TxRxEN(1, 0)
	}						
	while (*ptr!=0x0d)				// transmite restul caracterelor din bufferul ASCII, mai putin ultimul
																					
	UART1_Putch(*(++ptr));				// transmite si ultimul caracter
	UART1_TxRxEN(1, 1);				// activare Rx UART1
	
	if(TIP_NOD == 0)
	{				//nodul master nu goleste bufferul
		retea[i].full=0;												
	}								
	UART1_Getch(0);					// asteapta terminarea transmisiei/receptiei ultimului caracter
	
	UART1_TxRxEN(0, 0);				// dezactivare Tx si Rx UART1
	UART1_RS485_XCVR(0, 0);				// dezactivare Tx si Rx RS485
	return;	
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
