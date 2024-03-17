#include <c8051F040.h>						// declaratii SFR
#include <osc.h>
#include <Protocol.h>
#include <uart0.h>
#include <UserIO.h>

void Afisare_meniu(void);					// afisare meniu initial
void Afisare_mesaj(void);					// afisare mesaj receptionat
void Citire_mesaj(unsigned char i);			// preluare mesaj pentru a fi transmis catre nodul i
void Delay(int ms);								// intarziere programabila
void Error(char *ptr);						// afisare mesaj de eroare

unsigned char TERM_Input(void);

//***********************************************************************************************************
extern unsigned char TIP_NOD;
extern unsigned char STARE_IO;
extern nod retea[];

//***********************************************************************************************************
void UserIO(void){					// interfata cu utilizatorul
	unsigned char flag,tasta,dest;	// variabile locale
	
	tasta = TERM_Input();		    // periodic verifica daca s-a apasat o tasta
		
	switch(tasta){
		case '1':										// Tratare comanda transmisie mesaj
			UART0_Putstr("\n\rTx Msg:> Nod = ");	
			do{
				dest = TERM_Input();
				if(dest<'5' && dest>='0' && dest != ADR_NOD+'0'){	   // adresa trebuie sa fie 0-4, mai putin adresa proprie
					if(retea[dest-'0'].full){						   // Daca este deja un mesaj in buffer ...
						UART0_Putstr("\n\r!Buffer plin!");
						Delay(1000);													// Asteapta 1 secunda
						flag = 0;							  							// flag=0 indica acum ca s-a terminat tratarea comenzii
					}									
					else flag = 1;						// flag=1 indica acum ca nu s-a terminat tratarea comenzii
					break;
				}
			}
			while(1);								  
			if(flag == 1){							 	// Daca s-a terminat tratarea comenzii...
				retea[dest-'0'].bufbin.tipmes = USER_MES;			// pune in bufbin codul functiei
				retea[dest-'0'].bufbin.src = ADR_NOD;				// pune in bufbin adresa nodului sursa
				retea[dest - '0'].bufbin.dest = dest-'0';			// S: pune in bufbin adresa nodului destinatie
				UART0_Putstr("\n\rMesajul:> ");		// cere introducerea mesajului
				Citire_mesaj(dest - '0');                 // preia mesajul de la tastatura in bufferul nodului destinatie
				retea[dest-'0'].full = 1;						// marcheaza buffer plin
			}
			Afisare_meniu();
			break;																// terminare tratare comanda
	
		case '2':									   			// Tratare comanda citire stare buffer
			UART0_Putstr("\n\r>Numar nod = ");	
			do{											
				dest = TERM_Input();
				if(dest < '5' && dest >= '0' && dest != ADR_NOD + '0'){		   	// preia adresa nodului
					if(retea[dest - '0'].full) UART0_Putstr("\n\r!Buffer plin! ");  // afiseaza starea bufferului
					else  UART0_Putstr("\n\r!Buffer gol! ");
					Delay(1000);								// asteapta 1 secunda
					break;										// terminare tratare comanda
				}
			}
			while(1);
			Afisare_meniu();

	}
}

//***********************************************************************************************************
void Afisare_meniu(void){				  			// afisare meniu initial
	UART0_Putstr("\nTema ");
	UART0_Putch(TEMA + '0');
	
#if(PROTOCOL == MS)
	if(TIP_NOD == MASTER) UART0_Putstr(" Master ");	// daca programul se executa pe nodul master
	else UART0_Putstr(" Slave ");						// daca programul se executa pe un nod slave
#elif(PROTOCOL == JETON)
	if(TIP_NOD == JETON) UART0_Putstr(" Jeton ");
	else UART0_Putstr(" NoJet ");
#endif
	UART0_Putch(ADR_NOD + '0');						// afiseaza adresa nodului
#if(TEMA == 1 || TEMA == 3)
	UART0_Putstr(":ASC" );								// afiseaza parametrii specifici temei
#elif(TEMA == 2 || TEMA == 4)
	UART0_Putstr(":BIN" );
#endif
	UART0_Putstr("\n> 1-TxM 2-Stare :>");	// meniul de comenzi
}

//***********************************************************************************************************
void Citire_mesaj(unsigned char i){  // citire mesaj de la tastatura (max. 10 caractere, terminat cu \r)
	unsigned char tasta;
	unsigned char *ptr;
	ptr=&retea[i].bufbin.date;			  		// pointer in bufferul destinatie
	retea[i].bufbin.lng=0;				  			// contor lungime: initial 0
	while(1){
		tasta = TERM_Input();				  			// daca s-a apasat o tasta
		if(tasta){
			if(tasta == '\r') break;		  			// daca tasta a fost 'F', sfarsit mesaj
			else {						  			// altfel...
				if(retea[i].bufbin.lng == 10) break;	// daca s-a depasit lungimea maxima, terminare mesaj
				else{								// altfel...
					retea[i].bufbin.lng++;	  		// incrementeaza contor lungime
					*ptr++ = tasta;								// pune codul ASCII al tastei apasate im bufferul de date 
				}
			}
		}
	}
}

//***********************************************************************************************************
void Afisare_mesaj(void){          		// afisare mesaj din bufferul de receptie i
	unsigned char j, lng, *ptr;
	if(retea[ADR_NOD].full){						// exista mesaj in bufferul de receptie?
		lng = retea[ADR_NOD].bufbin.lng;
		UART0_Putstr("\n\r>Rx: De la nodul ");		
		UART0_Putch(retea[ADR_NOD].bufbin.src + '0');			// afiseaza adresa nodului sursa al mesajului
		UART0_Putstr(": ");
		for(j = 0, ptr = retea[ADR_NOD].bufbin.date; j < lng; j++)
			UART0_Putch(*ptr++);							// afiseaza mesajul, caracter cu caracter
		retea[ADR_NOD].full = 0;					// mesajul a fost afisat, marcheaza buffer gol
	}
}

//***********************************************************************************************************
void Error(char *ptr){
 	UART0_Putstr(ptr);
}

unsigned char TERM_Input(void){
	if(RI0) return UART0_Getch(1);
	else return 0;
}

