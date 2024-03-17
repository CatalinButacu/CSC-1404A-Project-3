//***************************************************************************
void PORT_Init (void);		// functia de initializare configuratie la pini

//***************************************************************************
// Definitii pentru valorile bitilor registrelor SFR XBR0-3 (Crossbar Switch)

//XBR0
#define CP0_ON 	0x80   	// COMP0 la pin
#define ECI0_ON	0x40	// PCA0 counter input la pin
#define PCA0ME_ON	0x30	// CEX0-CEX5 la pini
#define UART0_ON 	0x04	// UART0 la pini
#define SPI0_ON 	0x02	// SPI0 la pini
#define SMB0_ON 	0x01	// SMB/I2C la pini

#define CP0_OFF 		0		// COMP neconectat
#define ECI0_OFF		0		// PCA0 counter input neconectat
#define PCA0ME_OFF	0		// CEX0-5 neconectate
#define UART0_OFF 	0		// UART0 neconectat
#define SPI0_OFF 		0		// SPI0 neconectat
#define SMB0_OFF 		0		// SMB0 neconectat

//**********************************************************
//XBR1
#define SYSCLK_ON 0x80	// SYSCLK la pin
#define T2EX_ON	0x40	// T2EX pin
#define T2_ON 		0x20	// T2 pin
#define INT1_ON 	0x10	// INT1 pin
#define T1_ON 		0x08	// T1 pin
#define INT0_ON 	0x04	// INT0	pin
#define T0_ON 		0x02	// T0 pin
#define CP1_ON 	0x01	// CP1 pin

#define SYSCLK_OFF 	0		// SYSCLK neconectat
#define T2EX_OFF		0		// T2EX neconectat
#define T2_OFF 		0		// T2 neconectat
#define INT1_OFF 		0		// INT1 neconectat
#define T1_OFF 		0		// T1 neconectat
#define INT0_OFF 		0		// INT0 neconectat
#define T0_OFF 		0		// T0 neconectat
#define CP1_OFF 		0		// CP1 neconectat

//**********************************************************
//XBR2
#define WEAKPU_ON 0x00	// Weak pullups conectate
#define XBARE_ON 	0x40	// conectare pini
#define T4EX_ON 	0x10	// T4EX pin
#define T4_ON 		0x08	// T4 pin
#define UART1_ON 	0x04	// UART1 pini
#define EMIFL_ON 	0x02	// EMIF conectat la P0-3
#define CNVST0_ON 0x01	// CNVST0 pin

#define WEAKPU_OFF 	0x80	// Weak pullup neconectate
#define XBARE_OFF 	0		// conectare pini selectati
#define T4EX_OFF 		0		// T4EX neconectat
#define T4_OFF 		0		// T4 neconectat
#define UART1_OFF 	0		// UART1 neconectat
#define EMIFL_OFF 	0		// EMIF conectat la P4-7
#define CNVST0_OFF 	0		// CNVST0 neconectat

//**********************************************************
//XBR3
#define CTXOUT_ON 0x80	// CAN transmit pin iesire push-pull
#define CP2_ON 	0x08	// CP2 la pin
#define CNVST2_ON 0x04	// CNVST2 la pin
#define T3EX_ON 	0x02	// T3EX la pin
#define T3_ON 		0x01	// T3 la pin

#define CTXOUT_OFF 	0		// CAN transmit pin open-drain
#define CP2_OFF		0		// CP2 neconectat
#define CNVST2_OFF 	0		// CNVST2 neconectat
#define T3EX_OFF 		0		// T3EX neconectat
#define T3_OFF 		0		// T3 neconectat	

//************************************************************************************************************
// Configuratii posibile
#define BIG8051_ALL_GPIO 				0		// toti pinii GPIO (niciun periferic intern accesibil la pini)
#define BIG8051_UART0					1		// P0.0,1 = UART0
#define BIG8051_UART0_SPI				2		// P0.0,1 = UART0; P0.2,3,4,5 = SPI0
#define BIG8051_UART0_SPI_UART1		3		// P0.0,1 = UART0; P0.2,3,4,5 = SPI0; P0.6,7 = UART1
#define BIG8051_UART0_SPI_SMB_UART1	4		// P0.0,1 = UART0; P0.2,3,4,5 = SPI0; P0.6,7 = SMB; P1.0,1 = UART1
#define BIG8051_SPI						5		// P0.1,2,3,4 = SPI0
#define BIG8051_SPI_UART1				6		// P0.1,2,3,4 = SPI0; P0.4,5 = UART1
#define BIG8051_SPI_UART1_SMB			7		// P0.1,2,3,4 = SPI0; P0.4,5 = UART1; P0.6,7 = SMB
#define BIG8051_SMB						8		// P0.1,2 = SMB

//*************************************************************************************************************
#define BIG8051_CONFIG 	BIG8051_UART0_SPI_SMB_UART1	// ALEGERE CONFIGURATIE: TREBUIE SA CORESPUNDA CU CEA DE PE BIG8051
//*************************************************************************************************************

#ifndef BIG8051_CONFIG
	#error "Nu a fost selectata o configuratie pentru crossbar switch!!!"
#endif

//*************************************************************************************************************
#if	(BIG8051_CONFIG == BIG8051_ALL_GPIO)

#define XBR0_CONFIG	CP0_OFF    | ECI0_OFF | PCA0ME_OFF | UART0_OFF | SPI0_OFF  | SMB0_OFF 		 	

#define XBR1_CONFIG	SYSCLK_OFF | T2EX_OFF | T2_OFF     | INT1_OFF  | T1_OFF    | INT0_OFF  | T0_OFF | CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON  | XBARE_ON |	T4EX_OFF   | T4_OFF    | UART1_OFF | EMIFL_OFF | CNVST0_OFF 
							// enable pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON  | CP2_OFF  |	CNVST2_OFF | T3EX_OFF  | T3_OFF   
							// CAN TX pin push-pull mode
#endif

//*************************************************************************************************************							
#if BIG8051_CONFIG == BIG8051_UART0

#define XBR0_CONFIG	CP0_OFF    | ECI0_OFF | PCA0ME_OFF | UART0_ON  | SPI0_OFF  | SMB0_OFF 		 	
																		//enable UART0

#define XBR1_CONFIG	SYSCLK_OFF | T2EX_OFF | T2_OFF     | INT1_OFF  | T1_OFF    | INT0_OFF  | T0_OFF | CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON  | XBARE_ON |	T4EX_OFF   | T4_OFF    | UART1_OFF | EMIFL_OFF | CNVST0_OFF 
							// enable weak pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON  | CP2_OFF  |	CNVST2_OFF | T3EX_OFF  | T3_OFF   
							// CAN TX pin push-pull mode

#endif

//************************************************************************************************************							
#if BIG8051_CONFIG == BIG8051_UART0_SPI

#define XBR0_CONFIG	CP0_OFF    | ECI0_OFF | PCA0ME_OFF | UART0_ON  | SPI0_ON   | SMB0_OFF 		 	
																			//enable UART0, SPI0

#define XBR1_CONFIG	SYSCLK_OFF | T2EX_OFF | T2_OFF     | INT1_OFF  | T1_OFF    | INT0_OFF   | T0_OFF | CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON  | XBARE_ON |	T4EX_OFF   | T4_OFF    | UART1_OFF | EMIFL_OFF | CNVST0_OFF 
							// enable weak pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON  | CP2_OFF  |	CNVST2_OFF | T3EX_OFF  | T3_OFF   
							// CAN TX pin push-pull mode
					
#endif

//**********************************************************
#if BIG8051_CONFIG == BIG8051_UART0_SPI_UART1

#define XBR0_CONFIG	CP0_OFF 	| 	\	
					ECI0_OFF 		|	\	
					PCA0ME_OFF		|	\	
					UART0_ON 		|	\	
					SPI0_ON 			|	\	
					SMB0_OFF 		 	 //enable UART0, SPI0

#define XBR1_CONFIG	SYSCLK_OFF 	| 	\	
					T2EX_OFF 			|	\	
					T2_OFF				|	\	
					INT1_OFF 			|	\	
					T1_OFF 				|	\	
					INT0_ON 				|	\	
					T0_OFF				|	\	
					CP1_OFF				//enable INT0 - conectat la INT ZigBee

#define XBR2_CONFIG	WEAKPU_ON 	| 	\	   
					XBARE_ON 			|	\	   
					T4EX_OFF				|	\	
					T4_OFF 				|	\	
					UART1_ON 			|	\	
					EMIFL_OFF 			|	\	
					CNVST0_OFF			 // enable UART1, pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON 	| 	\	 
					CP2_OFF 				|	\	
					CNVST2_OFF			|	\	
					T3EX_OFF 			|	\	
					T3_OFF 				 // CAN TX pin push-pull mode
							
#endif
//**********************************************************
#if BIG8051_CONFIG == BIG8051_UART0_SPI_SMB_UART1

#define XBR0_CONFIG	CP0_OFF 	| 	\	
					ECI0_OFF 		|	\	
					PCA0ME_OFF		|	\	
					UART0_ON 		|	\	
					SPI0_ON 			|	\	
					SMB0_ON 		 	 //enable UART0, SPI0, SMB0

#define XBR1_CONFIG	SYSCLK_OFF 	| 	\	
					T2EX_OFF 			|	\	
					T2_OFF				|	\	
					INT1_OFF 			|	\	
					T1_OFF 				|	\	
					INT0_OFF 			|	\	
					T0_OFF				|	\	
					CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON 	| 	\	   
					XBARE_ON 			|	\	   
					T4EX_OFF				|	\	
					T4_OFF 				|	\	
					UART1_ON 			|	\	
					EMIFL_OFF 			|	\	
					CNVST0_OFF			 // enable UART1, pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON 	| 	\	 
					CP2_OFF 				|	\	
					CNVST2_OFF			|	\	
					T3EX_OFF 			|	\	
					T3_OFF 				 // CAN TX pin push-pull mode

#endif
//**********************************************************					
#if (BIG8051_CONFIG == BIG8051_SPI)

#define XBR0_CONFIG	CP0_OFF 	| 	\	
					ECI0_OFF 		|	\	
					PCA0ME_OFF		|	\	
					UART0_OFF 		|	\	
					SPI0_ON 			|	\	
					SMB0_OFF 		 	 //enable SPI0

#define XBR1_CONFIG	SYSCLK_OFF 	| 	\	
					T2EX_OFF 			|	\	
					T2_OFF				|	\	
					INT1_OFF 			|	\	
					T1_OFF 				|	\	
					INT0_OFF 			|	\	
					T0_OFF				|	\	
					CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON 	| 	\	   
					XBARE_ON 			|	\	   
					T4EX_OFF				|	\	
					T4_OFF 				|	\	
					UART1_OFF 			|	\	
					EMIFL_OFF 			|	\	
					CNVST0_OFF			 // enable pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON 	| 	\	 
					CP2_OFF 				|	\	
					CNVST2_OFF			|	\	
					T3EX_OFF 			|	\	
					T3_OFF 				 // CAN TX pin push-pull mode
												
#endif
//**********************************************************																	
#if (BIG8051_CONFIG == BIG8051_SPI_UART1)

#define XBR0_CONFIG	CP0_OFF 	| 	\	
					ECI0_OFF 		|	\	
					PCA0ME_OFF		|	\	
					UART0_OFF 		|	\	
					SPI0_ON 			|	\	
					SMB0_OFF 		 	 //enable SPI0

#define XBR1_CONFIG	SYSCLK_OFF 	| 	\	
					T2EX_OFF 		|	\	
					T2_OFF			|	\	
					INT1_OFF 		|	\	
					T1_OFF 			|	\	
					INT0_OFF 		|	\	
					T0_OFF			|	\	
					CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON 	| 	\	   
					XBARE_ON 			|	\	   
					T4EX_OFF				|	\	
					T4_OFF 				|	\	
					UART1_ON 			|	\	
					EMIFL_OFF 			|	\	
					CNVST0_OFF			 // enable UART1, pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON 	| 	\	 
					CP2_OFF 				|	\	
					CNVST2_OFF			|	\	
					T3EX_OFF 			|	\	
					T3_OFF 				 // CAN TX pin push-pull mode

#endif
//**********************************************************
#if (BIG8051_CONFIG == BIG8051_SPI_UART1_SMB)

#define XBR0_CONFIG	CP0_OFF 	| 	\	
					ECI0_OFF 		|	\	
					PCA0ME_OFF		|	\	
					UART0_OFF 		|	\	
					SPI0_ON 			|	\	
					SMB0_ON 		 	 //enable SPI0, SMB0

#define XBR1_CONFIG	SYSCLK_OFF 	| 	\	
					T2EX_OFF 		|	\	
					T2_OFF			|	\	
					INT1_OFF 		|	\	
					T1_OFF 			|	\	
					INT0_OFF 		|	\	
					T0_OFF			|	\	
					CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON 	| 	\	   
					XBARE_ON 			|	\	   
					T4EX_OFF				|	\	
					T4_OFF 				|	\	
					UART1_ON 			|	\	
					EMIFL_OFF 			|	\	
					CNVST0_OFF			// enable UART1, pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON 	| 	\	 
					CP2_OFF 				|	\	
					CNVST2_OFF			|	\	
					T3EX_OFF 			|	\	
					T3_OFF 				// CAN TX pin push-pull mode					

#endif
//**********************************************************
#if (BIG8051_CONFIG == BIG8051_SMB)

#define XBR0_CONFIG	CP0_OFF 	| 	\	
					ECI0_OFF 		|	\	
					PCA0ME_OFF		|	\	
					UART0_OFF 		|	\	
					SPI0_OFF 		|	\	
					SMB0_ON 		 	 //enable SMB0

#define XBR1_CONFIG	SYSCLK_OFF 	| 	\	
					T2EX_OFF 			|	\	
					T2_OFF				|	\	
					INT1_OFF 			|	\	
					T1_OFF 				|	\	
					INT0_OFF 			|	\	
					T0_OFF				|	\	
					CP1_OFF				

#define XBR2_CONFIG	WEAKPU_ON 	| 	\	   
					XBARE_ON 			|	\	   
					T4EX_OFF				|	\	
					T4_OFF 				|	\	
					UART1_OFF 			|	\	
					EMIFL_OFF 			|	\	
					CNVST0_OFF			 // enable pull-ups, crossbar

#define XBR3_CONFIG	CTXOUT_ON 	| 	\	 
					CP2_OFF 				|	\	
					CNVST2_OFF			|	\	
					T3EX_OFF 			|	\	
					T3_OFF 				 // CAN TX pin push-pull mode			
										 
#endif

//**********************************************************