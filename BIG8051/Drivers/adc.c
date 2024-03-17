#include <c8051f040.h>			// include fisierul cu definitiile SFR pentru C8051F040
#include <adc.h>

unsigned char ADC0_Init(unsigned char ADC0_Input, unsigned char ADC0_Start_SRC, float GT_value, float LT_value, unsigned char ADC0_IE, unsigned char ADC0W_IE){
	unsigned char SFRPAGE_save = SFRPAGE;	// salveaza codul paginii SFR curente
	
	SFRPAGE = ADC0_PAGE;		// comuta in pagina SFR aferenta ADC0

	ADC0_Select_Input(ADC0_Input);
	ADC0_Start_Mode(ADC0_Start_SRC);
	ADC0_Select_REF(EXT_VREF);
	ADC0W_Limits(GT_value, LT_value, 2.4);
	if(ADC0_IE) EIE2 |= 0x02; //EADC0 = 1;		// validare intrerupere la terminarea conversiei
	if(ADC0W_IE) EIE1 |= 0x04; //EWADC0 = 1;	// validare intrerupere la incadrare in fereastra
	if(ADC0_IE || ADC0W_IE) EA = 1;
	
	AD0EN = 1;							// Valideaza ADC0
	SFRPAGE = SFRPAGE_save;			// revenire in pagina de la intrare
	return 1;
}

unsigned char ADC0_Select_Input(unsigned char ADC0_Input){
	unsigned char SFRPAGE_save = SFRPAGE;	// salveaza codul paginii SFR curente
	
	SFRPAGE = ADC0_PAGE;		// comuta in pagina SFR aferenta ADC0
	
	switch (ADC0_Input){
		case AIN0: AMX0SL = 0x00;	AMX0CF = 0x00;	break; // Selectie semnal single ended AIN0.0
		case AIN01: AMX0SL = 0x00; AMX0CF = 0x01;	break; // Selectie semnal diferential +AIN0.0 -AIN0.1
		case AIN1: AMX0SL = 0x01;	AMX0CF = 0x00;	break; // Selectie canal AIN0.1
		case AIN2: AMX0SL = 0x02;	AMX0CF = 0x00;	break; // Selectie semnal single ended AIN0.2
		case AIN23: AMX0SL = 0x02;	AMX0CF = 0x02;	break; // Selectie semnal diferential +AIN0.2 -AIN0.3
		case AIN3: AMX0SL = 0x03;  AMX0CF = 0x00;	break; // Selectie canal AIN0.3
		case HVDA: AMX0SL = 0x04;	AMX0CF = 0x00;	break; // Selectie canal HVDA
		case HVDA_HVREF: AMX0SL = 0x04; AMX0CF = 0x04; break;	// Selectie +HVDA -HVREF
		case AGND: AMX0SL = 0x05; AMX0CF = 0x00;	break; // Selectie canal AGND
		case P3EVEN: AMX0SL = 0x06; AMX0CF = 0x00;break; // Selectie canal P3EVEN
		case P3EVEN_ODD : AMX0SL = 0x06;	AMX0CF = 0x08;	break; // Selectie canal +P3EVEN -P3ODD
		case P3ODD: AMX0SL = 0x06;	AMX0CF = 0x00;	break; // Selectie canal P3ODD
		case TEMP_SENSOR: AMX0SL = 0x08; AMX0CF = 0x00;	break; // Selectie canal TEMP_SENSOR
		default: 
			SFRPAGE = SFRPAGE_save;	// revenire in pagina de la intrare
			return 0;					// numar canal eronat
	}
	SFRPAGE = SFRPAGE_save;				// revenire in pagina de la intrare
	return 1;
}

unsigned char ADC0_Start_Mode (unsigned char ADC0_Start_SRC){
	unsigned char SFRPAGE_save = SFRPAGE;	// salveaza codul paginii SFR curente
	
	SFRPAGE = ADC0_PAGE;		// comuta in pagina SFR aferenta ADC0
	
	switch(ADC0_Start_SRC){
		case	ADC0_Start_SW:	AD0CM1 = AD0CM0 = 0; break;	  		// declansare conversie la scrierea in AD0BUSY
		case	ADC0_Start_T3OV: AD0CM1 = 0; AD0CM0 = 1; break;		// declansare conversie la depasire Timer 3
		case	ADC0_Start_CNVSTR0: AD0CM1 = 1; AD0CM0 = 0; break; // declansare conversie pe front crescator al CNVSTR0
		case	ADC0_Start_T2OV: AD0CM1 = AD0CM0 = 1; break;			// declansare conversie la depasire Timer 2
		default: 
				SFRPAGE = SFRPAGE_save;	// revenire in pagina de la intrare
				return 0;					// numar canal eronat
	}
	SFRPAGE = SFRPAGE_save;				// revenire in pagina de la intrare
	return 1;
}

unsigned char ADC0_Select_REF(unsigned char ADC0_REF){
	unsigned char SFRPAGE_save = SFRPAGE;	// salveaza codul paginii SFR curente
	
	SFRPAGE = ADC0_PAGE;		// comuta in pagina SFR aferenta ADC0
	switch(ADC0_REF){
		case	EXT_VREF: REF0CN &= 0xEF; REF0CN |= 0x03; break;	// Foloseste REF = VREF0 = VREF = 2,4V
		case	DAC0_VREF: REF0CN |= 0x10; break; // Foloseste REF = DAC0
		default:
				SFRPAGE = SFRPAGE_save;	// revenire in pagina de la intrare
				return 0;					// numar canal eronat
	}
	SFRPAGE = SFRPAGE_save;				// revenire in pagina de la intrare
	return 1;
}

unsigned char ADC0W_Limits(float ADC0_GT, float ADC0_LT, float ADC0_Ref){
	unsigned char SFRPAGE_save = SFRPAGE;	// salveaza codul paginii SFR curente
	
	SFRPAGE = ADC0_PAGE;		// comuta in pagina SFR aferenta ADC0

//	ADC0LTH = (int)(4096 * ADC0_Max/ADC0_Ref) / 256;
//	ADC0LTL = (int)(4096 * ADC0_Max/ADC0_Ref) % 256;
	ADC0LT = 4096 * ADC0_LT/ADC0_Ref;		// seteaza limita superioara a ferestrei

//	ADC0GTH = (int)(4096 * ADC0_Min/ADC0_Ref) / 256;
//	ADC0GTL = (int)(4096 * ADC0_Min/ADC0_Ref) % 256;
	ADC0GT = 4096 * ADC0_GT/ADC0_Ref;		// seteaza limita inferioara a ferestrei
	
	SFRPAGE = SFRPAGE_save;				// revenire in pagina de la intrare
	return 1;
}