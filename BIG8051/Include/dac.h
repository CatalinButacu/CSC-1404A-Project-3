unsigned char DAC0_Init(unsigned char DAC0_Mode);
void DAC0_Write(unsigned int DAC0_val);

sfr16	DAC0 = 0xD2;				// adresa perechii DAC0H:DAC0L ca registru SFR virtual pe 16 biti

#define DAC0_SW	0
#define DAC0_T3OV	1
#define DAC0_T4OV	2
#define DAC0_T2OV	3

	
