#include <c8051F040.h>						// declaratii SFR
#include <osc.h>
#include <Protocol.h>
#include <uart0.h>
#include <lcd.h>
#include <keyb.h>
#include <UserIO.h>

void Afisare_meniu(void);					// afisare meniu initial
void Afisare_mesaj(void);					// afisare mesaj receptionat
void Error(char *ptr);						// afisare mesaj de eroare

unsigned char TERM_Input(void);
unsigned char AFISARE = 1;

extern unsigned char LCD_line,LCD_col;

//***********************************************************************************************************
extern unsigned char ADR_MASTER;
extern unsigned char TIP_NOD;
extern unsigned char STARE_IO;
extern nod retea[];

//***********************************************************************************************************
void UserIO(void){					// interfata cu utilizatorul
	static unsigned char tasta, cmd, dest, lng;	// variabile locale statice
	
	if(0 == (tasta = TERM_Input())){
		tasta = KEYB_Input();
		if(tasta) LCD_Putch(tasta);
	}
	if(tasta){
		
		switch(STARE_IO){
			
			case 0:	switch(tasta){									
				
							case '1': 											// s-a dat comanda de transmisie mesaj								
																							// afiseaza Tx Msg:> Nod = 
																							// blocheaza afisarea (AFISARE = 0)
																							// trece in starea 1
																							// comanda este '1'
									break;
							
							case '2': 											// s-a dat comanda de afisare Stare Nod:
								
																							// blocheaza afisarea (AFISARE = 0)
																							// trece in starea 1
																							// comanda este '2'
							default: break;
						}
						break;
									
			case 1:													// s-a selectat nodul								
																			
																			// daca comanda e '1' si adresa e intre '0' - '4', mai putin adresa proprie
																				// extrage dest din tasta
																				// Daca este deja un mesaj in buffer ...
																				// afiseaza Buffer plin
			
																				// trece in starea 0, s-a terminat tratarea comenzii '1'
																				// afisare meniu
																					
																			// altfel ...
																				// daca nodul e master
																					// pune in bufferul dest adresa hw dest egala cu dest
																				// altfel ...
																					// pune in bufferul dest adresa hw dest egala cu ADR_MASTER
								
																				// pune in bufferul dest adresa hw sursa  egala cu ADR_NOD
																				// pune in bufferul dest adresa nodului sursa ADR_NOD
																				// pune in bufferul dest adresa nodului destinatie (dest)
																				// cere introducerea mesajului
			
																				// initializeaza lng = 0 
																				// trece in starea 2, sa astepte caracterele mesajului
																	
																
																		// daca comanda e '2' si adresa e intre '0'-'4'
																			// extrage dest din tasta
																			// Daca este deja un mesaj in buffer ...
																				// Afiseaza Buffer plin
																				
																			// altfel
																				// Afiseaza Buffer gol
																				
																			// trece in starea 0, s-a terminat tratarea comenzii
																			// afisare meniu
						break;
		
			case 2:													// daca tasta e diferita de CR ('\r'), de NL ('\n') si de '*' si nu s-a ajuns la limita maxima a bufferului de caractere
																			// stocheaza codul tastei apasate in bufferul de date si incrementeaza lng
																		// daca s-a atins nr maxim de caractere sau s-a apasat Enter ('\r') sau ('\n') sau '*'
																			// stocheaza lng
																			// pune in bufbin tipul mesajului (USER_MES)
																			// marcheaza buffer plin
																			// trece in starea 0, s-a terminat tratarea comenzii
																			// afisare meniu
	
						break;	
	
		}
	}
}

//***********************************************************************************************************
void Afisare_meniu(void){				  			// afisare meniu initial
	AFISARE = 1;
	UART0_Putstr("\n\rTema ");
	LCD_PutStr(0,0,"T");
	UART0_Putch(TEMA + '0');
	LCD_Putch(TEMA + '0');
	
#if(PROTOCOL == MS)
	if(TIP_NOD == MASTER){
		UART0_Putstr(" Master ");	// daca programul se executa pe nodul master
		LCD_PutStr(LCD_line, LCD_col, " Master:");
	}
	else{ 
		UART0_Putstr(" Slave ");						// daca programul se executa pe un nod slave
		LCD_PutStr(LCD_line, LCD_col, " Slave:");
	}
#elif(PROTOCOL == JT)
	if(TIP_NOD == JETON){
		UART0_Putstr(" Jeton ");
		LCD_PutStr(LCD_line, LCD_col, " Jeton:");
	}
	else{
		UART0_Putstr(" NoJet ");
		LCD_PutStr(LCD_line, LCD_col, "NoJet:");
	}
#endif
	
	UART0_Putch(ADR_NOD + '0');						// afiseaza adresa nodului
	LCD_Putch(ADR_NOD + '0');
#if(TEMA == 1 || TEMA == 3)
	UART0_Putstr(":ASC" );								// afiseaza parametrii specifici temei
	LCD_PutStr(LCD_line, LCD_col, " ASC");
#elif(TEMA == 2 || TEMA == 4)
	UART0_Putstr(":BIN" );
	LCD_PutStr(LCD_line, LCD_col, " BIN");
#endif
	UART0_Putstr("\n\r> 1-TxM 2-Stare :>");	// meniul de comenzi
	LCD_PutStr(1,0, "1-TxM 2-Stare :>");
}


//***********************************************************************************************************
void Afisare_mesaj(void){          		// afisare mesaj din bufferul de receptie i
	unsigned char j, lng, *ptr;
	if(retea[ADR_NOD].full){						// exista mesaj in bufferul de receptie?
		lng = retea[ADR_NOD].bufbin.lng;
		UART0_Putstr("\n\r>Rx: De la nodul ");
		LCD_DelLine(1);
		LCD_PutStr(1,0, "Rx: ");		
		UART0_Putch(retea[ADR_NOD].bufbin.src + '0');			// afiseaza adresa nodului sursa al mesajului
		LCD_Putch(retea[ADR_NOD].bufbin.src + '0');
		
		UART0_Putstr(": ");
		LCD_PutStr(LCD_line, LCD_col, ">: ");	
		
		for(j = 0, ptr = retea[ADR_NOD].bufbin.date; j < lng; j++) UART0_Putch(*ptr++);	// afiseaza mesajul, caracter cu caracter
		for(j = 0, ptr = retea[ADR_NOD].bufbin.date; j < lng; j++) LCD_Putch(*ptr++);		// afiseaza mesajul, caracter cu caracter

		retea[ADR_NOD].full = 0;					// mesajul a fost afisat, marcheaza buffer gol
	}
}

//***********************************************************************************************************
void Error(char *ptr){
	if(AFISARE){
		UART0_Putstr(ptr);
		LCD_DelLine(1);
		LCD_PutStr(1,0, ptr+2);
	}
}

unsigned char TERM_Input(void){

	unsigned char ch, SFRPAGE_save = SFRPAGE;
	
	SFRPAGE = LEGACY_PAGE;
	
	ch = 0;
	if(RI0) ch = UART0_Getch(1);
	
	SFRPAGE = SFRPAGE_save;
	
	return ch;
}