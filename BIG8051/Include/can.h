
sfr16 CAN0DAT = 0xD8;
//******************************************************************************
// CAN Protocol Register Index for CAN0ADR
//******************************************************************************
#define CANCTRL            0x00                 //Control Register
#define CANSTAT            0x01                 //Status register
#define ERRCNT             0x02                 //Error Counter Register
#define BITREG             0x03                 //Bit Timing Register
#define INTREG             0x04                 //Interrupt Low Byte Register
#define CANTSTR            0x05                 //Test register
#define BRPEXT             0x06                 //BRP Extension         Register

//******************************************************************************
//IF1 Interface Registers
//******************************************************************************
#define IF1CMDRQST         0x08                 //IF1 Command Request      Register
#define IF1CMDMSK          0x09                 //IF1 Command Mask      Register
#define IF1MSK1            0x0A                 //IF1 Mask1             Register
#define IF1MSK2            0x0B                 //IF1 Mask2             Register
#define IF1ARB1            0x0C                 //IF1 Arbitration 1     Register
#define IF1ARB2            0x0D                 //IF1 Arbitration 2     Register
#define IF1MSGC            0x0E                 //IF1 Message Control   Register
#define IF1DATA1           0x0F                 //IF1 Data A1           Register
#define IF1DATA2           0x10                 //IF1 Data A2           Register
#define IF1DATB1           0x11                 //IF1 Data B1           Register
#define IF1DATB2           0x12                 //IF1 Data B2           Register

//******************************************************************************
//IF2 Interface Registers
//******************************************************************************
#define IF2CMDRQST         0x20                 //IF2 Command Rest      Register
#define IF2CMDMSK          0x21                 //IF2 Command Mask      Register
#define IF2MSK1            0x22                 //IF2 Mask1             Register
#define IF2MSK2            0x23                 //IF2 Mask2             Register
#define IF2ARB1            0x24                 //IF2 Arbitration 1     Register
#define IF2ARB2            0x25                 //IF2 Arbitration 2     Register
#define IF2MSGC            0x26                 //IF2 Message Control   Register
#define IF2DATA1           0x27                 //IF2 Data A1           Register
#define IF2DATA2           0x28                 //IF2 Data A2           Register
#define IF2DATB1           0x29                 //IF2 Data B1           Register
#define IF2DATB2           0x2A                 //IF2 Data B2           Register

//******************************************************************************
//Message Handler Registers
//******************************************************************************
#define TRANSREQ1          0x40                 //Transmission Rest1 Register
#define TRANSREQ2          0x41                 //Transmission Rest2 Register

#define NEWDAT1            0x48                 //New Data 1            Register
#define NEWDAT2            0x49                 //New Data 2            Register

#define INTPEND1           0x50                 //Interrupt Pending 1   Register
#define INTPEND2           0x51                 //Interrupt Pending 2   Register

#define MSGVAL1            0x58                 //Message Valid 1       Register
#define MSGVAL2            0x59                 //Message Valid 2       Register

////////////////////////////////////////////////////////////////////////////////

#define CANID_XTD 			1
#define CANID_STD 			0
#define MSGOBJ_TX 			1
#define MSGOBJ_RX 			0

typedef union{
	unsigned char Byte[8];
	unsigned int Word[4];
} CAN_Data;

typedef struct {
	unsigned int ID;
	unsigned char DLC;
	CAN_Data Date;
	unsigned char NewDat;
}CAN_Msg;

////////////////////////////////////////////////////////////////////////////////
// Prototipuri functii CAN
////////////////////////////////////////////////////////////////////////////////

// Initialize Message Object
void CAN_Init (void);
int CAN_Init_Object(char MsgObjNum, char CANID_Type, unsigned long CANID, char ObjDLC, char MsgObjDir);
void CAN_Transmit (char MsgObjNum, unsigned char NewID, CAN_Msg *ptr);
void CAN_Receive (char MsgObjNum, CAN_Msg *ptr);


