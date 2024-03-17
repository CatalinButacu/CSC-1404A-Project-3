// Conexiuni modul LCD
sfr LCD_PORT = 0xB0;	  // P3 (P3.2,3,4,5)
sbit LCD_EN = P3^1;

#define CURSOR_ON 0x0A
#define CURSOR_OFF 0x08
#define DISPLAY_ON 0x0C
#define DISPLAY_OFF 0x08
#define CURSOR_BLINK 0x09

#define SHIFT_CURSOR_LEFT 0x10
#define SHIFT_CURSOR_RIGHT 0x14
#define SHIFT_DISPLAY_LEFT 0x18
#define SHIFT_DISPLAY_RIGHT 0x1C
#define CLEAR 0x01
#define HOME 0x02


void LCD_Init(void);
void LCD_Cmd(unsigned char cmd);
void LCD_Data(unsigned char ch);
void Delay_ms(int ms);
void LCD_Home(void);
void LCD_Clear(void);

void LCD_Display(unsigned char OnOff); // permite/blocheaza afisarea pe LCD
void LCD_Shift(unsigned char val);
void LCD_Putch(char ch);
int LCD_PutchPos(unsigned char line, unsigned char column, char ch);
int LCD_SetPos(unsigned char line, unsigned char column);
int LCD_PutStr(unsigned char line, unsigned char column, char *ptr);
int LCD_DelLine(unsigned int line);
int LCD_PutByte(unsigned char line, unsigned char column, unsigned char byte);
int LCD_PutByteHex(unsigned char line, unsigned char column, unsigned char byte);
int LCD_PutWordHex(unsigned char line, unsigned char column, unsigned int input);
int LCD_PutWord(unsigned char line, unsigned char column, unsigned int word);