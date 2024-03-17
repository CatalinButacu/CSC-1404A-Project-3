sfr16 RCAP2 = 0xCA;                    	// Timer2 reload value
sfr16 TMR2 = 0xCC;                     	// Timer2 counter

sfr16 RCAP3    = 0xCA;                 	// Timer3 reload value
sfr16 TMR3     = 0xCC;                 	// Timer3 counter

sfr16 RCAP4    = 0xCA;                 	// Timer4 reload value
sfr16 TMR4     = 0xCC;                 	// Timer4 counter

void Timer0_Init(void);
void Delay(int ms);

void Timer2_Init (unsigned int counts);
void Timer3_Init (unsigned int counts);
void Timer4_Init (unsigned int counts);