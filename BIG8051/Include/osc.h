#define IOSC_CLK_MAX 24500000L			// frecventa clock oscilator intern calibrat la 24,5 MHz
#define IOSC_CLK_MIN IOSC_CLK_MAX / 8	// frecventa clock oscilator intern dupã resetare
#define XOSC_CLK 25000000L        		// frecventa clock oscilator extern = 25 MHz

#define	IOSC_MIN	0					// oscilator intern la frecventa minima	(3,0625 MHz)
#define	IOSC_MAX	1					// oscilator intern la frecventa maxima (24,5 HHz)
#define  XOSC		2					// oscilator extern (25 MHz)

//*********************************************************************************************
#define OSC IOSC_MAX						// alege oscilatorul si frecventa
//*********************************************************************************************

#if(OSC==IOSC_MIN)
#define SYSCLK IOSC_CLK_MIN

#elif(OSC==IOSC_MAX)
#define SYSCLK IOSC_CLK_MAX

#elif (OSC==XOSC)
#define SYSCLK	XOSC_CLK			// selecteazã semnal de tact sistem de la oscilatorul extern
#elif
	#error "Nu este selectat un oscilator pentru SYSCLK în osc.h!"
#endif

void SYSCLK_Init (void);
