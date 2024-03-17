#include <C8051F040.h>                 // SFR declarations
#include <lcd.h>
#include <osc.h>

unsigned char LCD_line,LCD_col;

// initializare LCD
void LCD_Init(void)
{
	LCD_PORT = 0xFF;
	Delay_ms (20);

	LCD_PORT = 0xF3<<2;
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (10);

	LCD_PORT = 0xF3<<2;
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (1);

	LCD_PORT = 0xF3<<2;
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (1);
	
	LCD_PORT = 0xF2<<2;
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (1);

	LCD_Cmd(0x28);       // 4-bit mode - 2 line - 5x7 font. 
	LCD_Cmd(0x0F);       // Display on - cursor - blink.
//	LCD_Cmd(0x0C);       // Display on - no cursor - no blink.
	LCD_Cmd(0x06);       // Automatic Increment - No Display shift.
	LCD_Cmd(0x80);       // Address DDRAM with 0 offset 80h.
	LCD_Cmd(HOME);
	Delay_ms (1);
	LCD_Cmd(CLEAR);
	Delay_ms (1);
	LCD_line = LCD_col = 0;
 }

// transmite o comanda catre LCD
void LCD_Cmd(unsigned char cmd){
	LCD_PORT = 0xC0 | (cmd >> 2) & 0xFC;
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (1);

	LCD_PORT = 0xC0 | (cmd << 2) & 0xFC;
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (1);
}

// afiseaza un caracter in pozitia curenta
void LCD_Data(unsigned char ch){
	LCD_PORT = 0xC1 | ((ch >> 2) & 0xFC);
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (1);

	LCD_PORT = 0xC1 | ((ch << 2) & 0xFC);
	LCD_EN = 1;
	LCD_EN = 0;
	Delay_ms (1);
}

// seteaza pozitie cursor
int LCD_SetPos(unsigned char line, unsigned char column){
	if(line>1 || column>15) return 0;
	if(line) LCD_Cmd(0xC0+column);
	else LCD_Cmd(0x80+column);
	LCD_line = line;
	LCD_col = column;
	return 1;
}

// afiseaza un caracter intr-o anumita positie
int LCD_PutchPos(unsigned char line, unsigned char column, char ch){
   	if(LCD_SetPos(line, column)){
		LCD_Putch(ch);
		return 1;
	}
	else return 0;
}

// afiseaza un caracter in pozitia curenta a cursorului
void LCD_Putch(char ch) {
		LCD_Data(ch);
		if(++LCD_col>15){
			LCD_col=0;
			if(++LCD_line>1) LCD_line=0;
			LCD_SetPos (LCD_line, LCD_col);
		}
}

// afiseaza un sir de caractere
int LCD_PutStr(unsigned char line, unsigned char column, char *ptr){
	if(LCD_SetPos(line, column)){
		while(*ptr)
			LCD_Putch(*ptr++);
		return 1;
	}
	else return 0;
}

// sterge LCD, cursor (0,0)
void LCD_Home(void){
	LCD_Cmd(CLEAR);
	Delay_ms (1);
	LCD_Cmd(HOME);
	Delay_ms (1);
	LCD_line = LCD_col =0;
}

//sterge LCD
void LCD_Clear(void){
	LCD_Cmd(CLEAR);
	Delay_ms (1);
}

// permite/blocheaza afisarea pe LCD
void LCD_Display(unsigned char OnOff){
	if(OnOff == DISPLAY_ON) LCD_Cmd(DISPLAY_ON);
	if(OnOff == DISPLAY_OFF) LCD_Cmd(DISPLAY_OFF);
	Delay_ms (1);
}


//sterge LCD
void LCD_Shift(unsigned char val){
	switch(val){
		case SHIFT_DISPLAY_RIGHT:
		case SHIFT_DISPLAY_LEFT:
			LCD_Cmd(val);
			Delay_ms (1);
			break;
		
		case SHIFT_CURSOR_RIGHT:
			if(LCD_col<15) LCD_SetPos(LCD_line,LCD_col+1);
			break;
			
		case SHIFT_CURSOR_LEFT:
			if(LCD_col) LCD_SetPos(LCD_line,LCD_col-1);
			break;
	}
}

// sterge linia (0 sau 1)
int LCD_DelLine(unsigned int line){
	if(line>1) return 0;
	else LCD_PutStr(line,0, "                ");
	LCD_SetPos(line,0);
	return 1;
}


// afiseaza un octet in zecimal
int LCD_PutByte(unsigned char line, unsigned char column, unsigned char byte)
{
	char i,str[3];
	if(LCD_SetPos(line, column)){
		for(i=3; i>0; i--){
			str[i-1] = byte % 10 + '0';
			byte = byte/10;
		}
		for (i=0; (i<2) && (str[i]=='0') ; i++) LCD_Putch(' ');
		for(; i<3; i++) LCD_Putch(str[i]);
		return 1; 
	}
	else return 0;
}

// afiseaza un cuvant de 16 biti in zecimal
int LCD_PutWord(unsigned char line, unsigned char column, unsigned int word)
{
	char i,str[5];
	if(LCD_SetPos(line, column)){
		for(i=5; i>0; i--){
			str[i-1] = word % 10+ '0';
			word = word/10;
		}
		for (i=0; (i<4) && (str[i]=='0'); i++) LCD_Putch(' ');
		for(; i<5; i++) LCD_Putch(str[i]);
		return 1; 
	}
	else return 0;
}

// afiseaza un octet in hexazecimal
int LCD_PutByteHex(unsigned char line, unsigned char column, unsigned char input)
{
    char temp,digit,str[2];
    if(LCD_SetPos(line, column)){	             //go to position (x,y)
		for (digit=2; digit > 0; digit--) {
	    	temp = input % 0x10;
	        if(temp <= 0x09) str[digit-1] = temp + '0';
	        else str[digit-1] = temp - 0x0a + 'A';
	        input = input / 0x10;

	    }
		for (digit=0; digit<2; digit++) LCD_Putch(str[digit]);
		return 1;
	}
	else return 0;
}

// afiseaza un cuvant de 16 biti in hexazecimal
int LCD_PutWordHex(unsigned char line, unsigned char column, unsigned int input)
{
    char temp,digit,str[4];
    if(LCD_SetPos(line, column)){	             //go to position (x,y)
		for (digit=4; digit > 0; digit--) {
	    	temp = input % 0x10;
	        if(temp <= 0x09) str[digit-1] = temp + '0';
	        else str[digit-1] = temp - 0x0a + 'A';
	        input = input / 0x10;

	    }
		for (digit=0; digit<4; digit++) LCD_Putch(str[digit]);
		return 1;
	}
	else return 0;
}

#if (OSC==XOSC)
#define COUNT_1MS 3200

#elif (OSC==IOSC_MAX)
#define COUNT_1MS 3000

#elif (OSC==IOSC_MIN)
#define COUNT_1MS 400
#endif

void Delay_ms(int ms){
	int i,j;
	for(i = ms; i > 0; i--)
   		for (j = COUNT_1MS; j > 0; j--);    // la 25 MHz, 1 ms = 25000 / SYSCLK
}
