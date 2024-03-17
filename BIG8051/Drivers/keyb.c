#include <C8051F040.h>                 // SFR declarations
#include <keyb.h>
#include <timer.h>
#include <osc.h>

unsigned char key, last_key, keypress_detected, key_released; // variabile globale

#define T3INT_10MS 10 * SYSCLK/12/1000

void KEYB_Init(void){
  	unsigned char SFRPAGE_save = SFRPAGE; // Save the current SFRPAGE

 	SFRPAGE = CONFIG_PAGE;

	key = keypress_detected = 0;
	key_released = 1;
	
	P4 &= 0x0F;			// Pune 0 pe coloane (P4.7 - P4.4)
	P4 |= 0x80;			// comandã pe 1 prima coloanã (P4.7)

	Timer3_Init(T3INT_10MS);	// initializeaza Timer 3
	
	SFRPAGE = SFRPAGE_save;	
}

char KEYB_Input(void){
	unsigned char val;
	EA 	= 	0;			// dezactivare intreruperi
	val = key;
	if (key) last_key = key;
	key	=	0;
	EA 	= 1;			// reactivare intreruperi
	return val;
}

void Timer3_ISR (void) interrupt 14	   // handler intrerupere Timer3 
{
unsigned char val;

	if(TF3){						   // intrerupere la depasire Timer 3?
		TF3 = 0;					   	// Daca da, sterge flagul de intrerupere la depasire
		SFRPAGE = CONFIG_PAGE;		// permite accesul la P4
		val = P4;					   // citeste P4
		if(!keypress_detected){		// daca nu era detectata anterior nicio apasare
			if(val & 0x0F){					// daca s-a apasat intre timp o tasta
				keypress_detected = val;			// memoreaza codul tastei apaste
				return;									// revine fara sa se treaca la urmatoarea coloana
			}
		}
		else {							// daca exista o apasare detectat anterior
			if(val == keypress_detected) {	// daca aceeasi tasta a ramas apasata
				if(key_released){						// daca inainte de detectare nu era nicio tasta apasata			
					key_released = 0;						// acum este apasata o tasta
					switch(val){							// decodifica tasta apasata si pune codul ASCII in key
						case 0x88: key = 'D'; break;		// codul tastei apasate ferm corespunde tastei "D"
						case 0x84: key = 'C'; break;		// ...................
						case 0x82: key = 'B'; break;
						case 0x81: key = 'A'; break;
						case 0x48: key = '#'; break;
						case 0x44: key = '9'; break;
						case 0x42: key = '6'; break;
						case 0x41: key = '3'; break;
						case 0x28: key = '0'; break;
						case 0x24: key = '8'; break;
						case 0x22: key = '5'; break;
						case 0x21: key = '2'; break;
						case 0x18: key = '*'; break;
						case 0x14: key = '7'; break;
						case 0x12: key = '4'; break;
						case 0x11: key = '1'; break;
					}
				}
			}
			else {									// daca codul tastei apasate acum e diferit de cel memorat
				keypress_detected = 0;				// nu mai este detectata nicio apasare
				key_released = 1;						// nu mai este apasata nicio tasta
			}
		}
		if(key_released){											// daca nu mai este apasata nicio tasta
			P4 &= 0x0F;		  											// dezactiveaza si coloana curenta 
			if ((val & 0xF0) != 0x10)	P4 |= val >> 1;		// daca nu a fost ultima coloana, trece la urmatoarea 
			else P4 |= 0x80;											// altfel, reia de la prima coloana
		}
	}
}

