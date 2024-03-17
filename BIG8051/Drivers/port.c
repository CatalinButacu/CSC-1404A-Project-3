#include <C8051F040.h>           // declaratii SFR
#include <port.h>

//====================================================================================
// PORT_Init - Configurare Crossbar Switch si Porturi de I/E de uz general (GPIO)
//====================================================================================

void PORT_Init (void)
{
   unsigned char SFRPAGE_save = SFRPAGE; 	// Salveaza SFRPAGE

   SFRPAGE = CONFIG_PAGE;   // Selecteaza CONFIG_PAGE
	
	P0MDOUT |= 0x01;        // valideaza Tx0 ca iesire push-pull
	P1MDOUT |= 0x0D;        // valideaza Tx1, TxEN si RxEN ca iesiri push-pull
	P1 &= 0xFB;					// pune TxEN = 0

	P3MDOUT |= 0x3F;			// valideaza P3.5 - P3.0 ca iesiri push-pull (linii de comanda si date LCD)
	P4MDOUT |= 0xF0;			// valideaza P4.7 - P4.4 ca iesiri push-pull  (comanda coloane Keyboard)

	XBR0 = XBR0_CONFIG;		// valideaza perifericele interne utilizate			
	XBR1 = XBR1_CONFIG;					
	XBR3 = XBR3_CONFIG;					
	XBR2 = XBR2_CONFIG;		// valideaza crossbar switch si weak pull-ups interne

	SFRPAGE = SFRPAGE_save; // Reface SFRPAGE	
}