#define EIGHT_BIT 0		// mod de lucru cu 8 biti pe caracter
#define NINE_BIT 1		// mod de lucru cu 9 biti pe caracter (cu bit de paritate sau în regim multiprocesor)

#define MULTIPROC_ADRESA	1		// seteaza al 9-lea bit pe 1 la transmisie si receptioneaza doar caractere cu bitul 9 setat
#define MULTIPROC_DATA		0		// forteaza pe 0 al 9-lea bit la transmisie si receptioneaza toate caracterele,
											// indiferent de  bitul 9 


int UART1_Init(unsigned char mode, long int baudrate); 	// initializare UART1
void UART1_Putch(unsigned char ch);								// transmisie caracter cu UART1
void UART1_Putstr(unsigned char *ptr);							// transmisie sir de caractere cu UART1
unsigned char UART1_Getch(unsigned char echo);				// receptie caracter - cu blocare
void UART1_PutchPE(unsigned char ch);							// transmisie caracter cu paritate para
void UART1_PutstrPE(unsigned char *ptr);						// transmisie sir de caractere cu paritate para
void UART1_Flush(void);
void UART1_TxRxEN(unsigned char TxEN, unsigned char RxEN);// valideaza/inhiba transmitatorul si receptorul UART1

unsigned char UART1_Getch_TMO(int ms);							// receptie caracter cu timeout
void UART1_Flush(void);
void UART1_TxRxEN(unsigned char TxEN, unsigned char RxEN);// valideaza/inhiba transmitatorul si receptorul UART1

void UART1_MultiprocMode(unsigned char mod);					// selectare regim multiprocesor adresa/date
void UART1_RS485_XCVR(unsigned char TxEN, unsigned char RxEN);	// velidare/dezactivare emitator/receptor de linie RS485

sbit P12 = P1^2;		// validare emitator de linie (TxEN = 1)
sbit P13 = P1^3;		// vaidare receptor de linie (RxEN = 0)