#include <C8051F040.h>                 // declaratii SFR
#include <osc.h>

//-----------------------------------------------------------------------------
void SYSCLK_Init (void){
	unsigned char SFRPAGE_save = SFRPAGE; 	// salveaza SFRPAGE
	int i;
	SFRPAGE = CONFIG_PAGE;              // comuta SFRPAGE
	switch(OSC){
		case IOSC_MAX:	OSCICN = 0x83;                // IOSC_CLK = IOSC_CLK_MAX 24.5 MHz)
					   	CLKSEL = 0x00;					   // selecteazã oscilatorul intern ca SYSCLK
							break;

		case XOSC:		OSCXCN = 0x60; 					// cristal de cuartz, fara divizare cu 2
					   	OSCXCN |= 7;						// 10 MHz < f_crystal <= 30 MHz
					   	for (i = 3000; i > 0; i--);        	// asteapta 1 ms: la 24.5 MHz, 1 ms = 24500 SYSCLKs
					   	while ((OSCXCN & 0x80) != 0x80);		// asteapta stabilizarea oscilatiilor: XTLVLD => ‘1’.
					   	CLKSEL = 0x01;					    		// selecteazã oscilatorul extern ca SYSCLK
							break;
		
		case IOSC_MIN:	OSCICN = 0x80;                // IOSC_CLK = IOSC_CLK_MAX 24.5 MHz)
					   	CLKSEL = 0x00;					   // selecteazã oscilatorul intern ca SYSCLK
	}
   RSTSRC = 0x04; 						   // valideaza Missing Clock Detector
	SFRPAGE = SFRPAGE_save;             // reface SFRPAGE	           
}
