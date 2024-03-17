unsigned char ADC0_Init(unsigned char ADC0_Input, unsigned char ADC0_Start_Mode, float GT_value, float LT_value, unsigned char ADC0_int, unsigned char ADC0W_int);
unsigned char ADC0_Select_Input(unsigned char ADC0_Input);
unsigned char ADC0_Start_Mode (unsigned char ADC0_Start_SRC);
unsigned char ADC0_Select_REF(unsigned char ADC0_REF);
unsigned char ADC0W_Limits(float ADC0_GT, float ADC0_LT, float ADC0_Ref);

sfr16	ADC0GT = 0xC4;				// adresa perechii ADC0GTH:ADC0GTL ca registru SFR virtual pe 16 biti
sfr16	ADC0LT = 0xC6;				// adresa perechii ADC0LTH:ADC0LTL ca registru SFR virtual pe 16 biti
sfr16	ADC0 = 0xBE;				// adresa perechii ADC0H:ADC0L ca registru SFR virtual pe 16 biti

#define AIN0 			0
#define AIN01 			1
#define AIN1 			2
#define AIN2 			3
#define AIN23 			4
#define AIN3 			5
#define HVDA 			6
#define HVDA_HVREF 	7
#define AGND 			8
#define P3EVEN			9
#define P3EVEN_ODD 	10
#define P3ODD			11
#define TEMP_SENSOR	12

#define ADC0_Start_SW		0
#define ADC0_Start_T3OV		1
#define ADC0_Start_CNVSTR0	2
#define ADC0_Start_T2OV		3

#define EXT_VREF	0
#define DAC0_VREF	1