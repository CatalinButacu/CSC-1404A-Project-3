#include <C8051F040.h>                 // Declaratii SFR
#include <wdt.h>

void WDT_Disable(void){		// dezactiveaza ceasul de garda
   unsigned char SFRPAGE_save = SFRPAGE; // Salveaza SFRPAGE

   SFRPAGE = CONFIG_PAGE;          	// Selecteaza CONFIG_PAGE

	if(EA == 1){
		EA = 0;
	   WDTCN = 0xDE;
	   WDTCN = 0xAD;
		EA = 1;
	}
	else{
	   WDTCN = 0xDE;
	   WDTCN = 0xAD;
	}
	SFRPAGE = SFRPAGE_save;             // Reface SFRPAGE
}

void WDT_Restart(void){		// rearmeaza ceasul de garda
   unsigned char SFRPAGE_save = SFRPAGE; 	// Salveaza SFRPAGE

   SFRPAGE = CONFIG_PAGE;              	// Selecteaza CONFIG_PAGE

   WDTCN = 0xA5; 

	SFRPAGE = SFRPAGE_save;             		// Reface SFRPAGE	
}

void WDT_Lock(void){		// blocheaza accesul la ceasul de garda
   unsigned char SFRPAGE_save = SFRPAGE; 	// Salveaza SFRPAGE

   SFRPAGE = CONFIG_PAGE;              	// Selecteaza CONFIG_PAGE

   WDTCN = 0xFF; 	

	SFRPAGE = SFRPAGE_save;             		// Reface SFRPAGE	
}

int WDT_Status(void){		// citeste starea ceasului de garda: 0x40-ON; 0-OFF; 
   unsigned char ch, SFRPAGE_save = SFRPAGE; // Salveaza SFRPAGE

   SFRPAGE = CONFIG_PAGE;              		// Selecteaza CONFIG_PAGE

   ch=WDTCN & 0x10; 

	SFRPAGE = SFRPAGE_save;             			// Reface SFRPAGE	

	return ch;	
}