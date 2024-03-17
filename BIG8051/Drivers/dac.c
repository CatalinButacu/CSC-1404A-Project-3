#include <c8051f040.h>
#include <dac.h>

unsigned char DAC0_Init(unsigned char DAC0_Mode){
	unsigned char SFRPAGE_save = SFRPAGE;	// salveaza codul paginii SFR curente
	
	SFRPAGE = DAC0_PAGE;		// comuta in pagina SFR aferenta DAC0
	switch(DAC0_Mode){
		case DAC0_SW:		DAC0CN = 0x80; break;	// Validare DAC0, generare semnal la scrierea in DAC0H, aliniere la dreapta
		case DAC0_T3OV:	DAC0CN = 0x88; break;	// Validare DAC0, generare semnal la depasire Timer 3, aliniere la dreapta
		case DAC0_T4OV:	DAC0CN = 0x90; break;	// Validare DAC0, generare semnal la depasire Timer 4, aliniere la dreapta
		case DAC0_T2OV:	DAC0CN = 0x98; break;	// Validare DAC0, generare semnal la depasire Timer 2, aliniere la dreapta
		default:	
			SFRPAGE = SFRPAGE_save;
			return 0;
	}
	REF0CN |= 0x03;				//	REFBE = 1, BIASE = 1; // Genereaza VREF = 2,4 V
	return 1;
}
					
void DAC0_Write(unsigned int DAC0_val){
	unsigned char SFRPAGE_save = SFRPAGE;	// salveaza codul paginii SFR curente

	SFRPAGE = DAC0_PAGE;		// comuta in pagina SFR aferenta DAC0
	DAC0 = DAC0_val;
	SFRPAGE = SFRPAGE_save;
}