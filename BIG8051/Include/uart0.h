#define EIGHT_BIT 0	// mod de lucru cu 8 biti pe caracter
#define NINE_BIT 1	// mod de lucru cu 9 biti pe caracter (cu bit de paritate sau în regim multiprocesor)

int UART0_Init(unsigned char mode, long int baudrate);	// initializare UART0
void UART0_Putch(unsigned char ch);								// transmisie caracter cu UART1
void UART0_Putstr(unsigned char *ptr);							// transmisie sir de caractere cu UART1
unsigned char UART0_Getch(unsigned char echo);				// receptie caracter - cu blocare
unsigned char UART0_Check(void);									// testeaza receptia unui caracter - fara blocare