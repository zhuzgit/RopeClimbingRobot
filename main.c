// #pragma float64 // 启用双精度浮点数支持
#include <REG251.H> //包含STC32G的头文件
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "intrins.h" //使用_nop_()函数所必须要包含的头文件,
                     //否则延时函数中调用的_nop_()函数没有被头文件引用过来，
                     //会导致编译器找不到这个而函数而报错。

typedef unsigned char u8;
typedef unsigned int u16;
typedef unsigned long u32;

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;






//==========================================================================

/*************  本地常量声明    **************/
#define UART2_BUF_LENGTH    32


/*************  IO口定义    **************/
//COM2 RXD read SBUS

#define MOVE P02
#define UD P03
#define RC P32
#define RE P33



/*************  本地变量声明    **************/


u8  RX2_Cnt;    //接收计数

u8  RX2_Buffer[UART2_BUF_LENGTH]; //接收缓冲
u8 sbus[25],rcd=0;

u16 CAN_ID,CAN_ID1;
u8 RX_BUF[8];
u8 TX_BUF[8];
bit B_CanRead;      //CAN 收到数据标志

u8 menable[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
u8 mdisable[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd};
u8 mclear[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfb};
u8 mspd[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//========================================================================
// 函数: u8 CanReadReg(u8 addr)
// 描述: CAN功能寄存器读取函数。
// 参数: CAN功能寄存器地址.
// 返回: CAN功能寄存器数据.
// 版本: VER1.0
// 日期: 2020-11-16
// 备注: 
//========================================================================
u8 CanReadReg(u8 addr)
{
	u8 dat;
	CANAR = addr;
	dat = CANDR;
	return dat;
}

//========================================================================
// 函数: void CanWriteReg(u8 addr, u8 dat)
// 描述: CAN功能寄存器配置函数。
// 参数: CAN功能寄存器地址, CAN功能寄存器数据.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-11-16
// 备注: 
//========================================================================
void CanWriteReg(u8 addr, u8 dat)
{
	CANAR = addr;
	CANDR = dat;
}

//========================================================================
// 函数: void CanReadFifo(u8 *pdat)
// 描述: 读取CAN缓冲区数据函数。
// 参数: *pdat: 存放CAN缓冲区数据.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-11-16
// 备注: 
//========================================================================
/*
void CanReadFifo(u8 *pdat)
{
	pdat[0]  = CanReadReg(RX_BUF0);
	pdat[1]  = CanReadReg(RX_BUF1);
	pdat[2]  = CanReadReg(RX_BUF2);
	pdat[3]  = CanReadReg(RX_BUF3);

	pdat[4]  = CanReadReg(RX_BUF0);
	pdat[5]  = CanReadReg(RX_BUF1);
	pdat[6]  = CanReadReg(RX_BUF2);
	pdat[7]  = CanReadReg(RX_BUF3);

	pdat[8]  = CanReadReg(RX_BUF0);
	pdat[9]  = CanReadReg(RX_BUF1);
	pdat[10] = CanReadReg(RX_BUF2);
	pdat[11] = CanReadReg(RX_BUF3);

	pdat[12]  = CanReadReg(RX_BUF0);
	pdat[13]  = CanReadReg(RX_BUF1);
	pdat[14]  = CanReadReg(RX_BUF2);
	pdat[15]  = CanReadReg(RX_BUF3);
}
*/
//========================================================================
// 函数: u16 CanReadMsg(u8 *pdat)
// 描述: CAN接收数据函数。
// 参数: *pdat: 接收数据缓冲区.
// 返回: CAN ID.
// 版本: VER1.0
// 日期: 2020-11-19
// 备注: 
//========================================================================
/*
u16 CanReadMsg(u8 *pdat)
{
	u8 i;
	u16 CanID;
	u8 buffer[16];

	CanReadFifo(buffer);
	CanID = ((buffer[1] << 8) + buffer[2]) >> 5;
	for(i=0;i<8;i++)
	{
		pdat[i] = buffer[i+3];
	}
	return CanID;
}
*/
//========================================================================
// 函数: void CanSendMsg(u16 canid, u8 *pdat)
// 描述: CAN发送数据函数。
// 参数: canid: CAN ID; *pdat: 发送数据缓冲区.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-11-19
// 备注: 
//========================================================================
void CanSendMsg(u16 canid, u8 *pdat)
{
	u16 CanID;

	CanID = canid << 5;
	CanWriteReg(TX_BUF0,0x08);	//bit7: 标准帧(0)/扩展帧(1), bit6: 数据帧(0)/远程帧(1), bit3~bit0: 数据长度(DLC)
	CanWriteReg(TX_BUF1,(u8)(CanID>>8));
	CanWriteReg(TX_BUF2,(u8)CanID);
	CanWriteReg(TX_BUF3,pdat[0]);

	CanWriteReg(TX_BUF0,pdat[1]);
	CanWriteReg(TX_BUF1,pdat[2]);
	CanWriteReg(TX_BUF2,pdat[3]);
	CanWriteReg(TX_BUF3,pdat[4]);

	CanWriteReg(TX_BUF0,pdat[5]);
	CanWriteReg(TX_BUF1,pdat[6]);
	CanWriteReg(TX_BUF2,pdat[7]);
	
	CanWriteReg(TX_BUF3,0x00);
	CanWriteReg(CMR ,0x04);		//发起一次帧传输
}

//========================================================================
// 函数: void CANSetBaudrate()
// 描述: CAN总线波特率设置函数。
// 参数: none.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-11-19
// 备注: 
//========================================================================
void CANSetBaudrate()////1000Kbps@24MHz

{
	CanWriteReg(MR, 0x04);		//使能Reset模式
	CanWriteReg(BTR0, 0x00);	//SJW(0), BRP(0)
	CanWriteReg(BTR1, 0x18);	//SAM(0), TSG2(1), TSG1(8)
	CanWriteReg(MR, 0x00);		//退出Reset模式
}

//========================================================================
// 函数: void CANInit()
// 描述: CAN初始化函数。
// 参数: none.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-11-19
// 备注: 
//========================================================================
void CANInit()
{
	CANEN = 1;          //CAN1模块使能
	CANSEL = 0;         //选择CAN1模块
	P_SW1 = (P_SW1 & ~(3<<4)) | (0<<4); //端口切换(CAN_Rx,CAN_Tx) 0x00:P0.0,P0.1  0x10:P5.0,P5.1  0x20:P4.2,P4.5  0x30:P7.0,P7.1

//	CAN2EN = 1;         //CAN2模块使能
//	CANSEL = 1;         //选择CAN2模块
//	P_SW3 = (P_SW3 & ~(3)) | (0);       //端口切换(CAN_Rx,CAN_Tx) 0x00:P0.2,P0.3  0x01:P5.2,P5.3  0x02:P4.6,P4.7  0x03:P7.2,P7.3

	CanWriteReg(MR  ,0x04);		//使能 Reset Mode
	CANSetBaudrate();	//设置波特率
	
	CanWriteReg(ACR0,0x00);		//总线验收代码寄存器
	CanWriteReg(ACR1,0x00);
	CanWriteReg(ACR2,0x00);
	CanWriteReg(ACR3,0x00);
	CanWriteReg(AMR0,0xFF);		//总线验收屏蔽寄存器
	CanWriteReg(AMR1,0xFF);
	CanWriteReg(AMR2,0xFF);
	CanWriteReg(AMR3,0xFF);

	CanWriteReg(IMR ,0xff);		//中断寄存器
	CanWriteReg(ISR ,0xff);		//清中断标志
	CanWriteReg(MR  ,0x00);		//退出 Reset Mode

	CANICR = 0x02;		//CAN中断使能
}

//========================================================================
// 函数: void CANBUS_Interrupt(void) interrupt CAN_VECTOR
// 描述: CAN总线中断函数。
// 参数: none.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-11-19
// 备注: 
//========================================================================
void CANBUS_Interrupt(void) interrupt CAN1_VECTOR
{
	u8 isr;
	u8 arTemp;
	arTemp = CANAR;     //CANAR现场保存，避免主循环里写完 CANAR 后产生中断，在中断里修改了 CANAR 内容
	
	isr = CanReadReg(ISR);
	if((isr & 0x04) == 0x04)  //TI
	{
		CANAR = ISR;
		CANDR |= 0x04;    //CLR FLAG
	}
	if((isr & 0x08) == 0x08)  //RI
	{
		CANAR = ISR;
		CANDR |= 0x08;    //CLR FLAG
	
		B_CanRead = 1;
	}

	if((isr & 0x40) == 0x40)  //ALI
	{
		CANAR = ISR;
		CANDR |= 0x40;    //CLR FLAG
	}

	if((isr & 0x20) == 0x20)  //EWI
	{
		CANAR = ISR;
		CANDR |= 0x20;    //CLR FLAG
	}

	if((isr & 0x10) == 0x10)  //EPI
	{
		CANAR = ISR;
		CANDR |= 0x10;    //CLR FLAG
	}

	if((isr & 0x02) == 0x02)  //BEI
	{
		CANAR = ISR;
		CANDR |= 0x02;    //CLR FLAG
	}

	if((isr & 0x01) == 0x01)  //DOI
	{
		CANAR = ISR;
		CANDR |= 0x01;    //CLR FLAG
	}

	CANAR = arTemp;    //CANAR现场恢复
}




/*************  COM Init    **************/
#define MAIN_Fosc        24000000L   //定义主时钟（精确计算115200波特率）
#define Baudrate2   (65536 - MAIN_Fosc / 100000 / 4)
#define Baudrate (65536 - MAIN_Fosc / 460800 / 4)


void Uart1_Init(void)	//460800bps@24.000MHz
{
  TR1 = 0;
  S1BRT = 0;     // S1 BRT Use Timer1;
  T1_CT = 0;     // Timer1 set As Timer
  T1x12 = 1;     // Timer1 set as 1T mode
  TMOD &= ~0x30; // Timer1_16bitAutoReload;
  TH1 = (u8)(Baudrate / 256);
  TL1 = (u8)(Baudrate % 256);
  ET1 = 0; // 禁止定时器中断
  TR1 = 1;
  SCON = (SCON & 0x3f) | 0x40; // UART1模式, 0x00: 同步移位输出, 0x40: 8位数据,可变波特率, 0x80: 9位数据,固定波特率, 0xc0: 9位数据,可变波特率
                               //   PS  = 1;    //高优先级中断
  ES = 1;  // 允许串口中断
  REN = 1; // 允许接收
  P_SW1 &= 0x3f;
}

void Uart2_Init(void)	//100000bps@24.000MHz
{
	S2CON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x04;		//定时器时钟1T模式
    T2H = (u8)(Baudrate2 / 256);
    T2L = (u8)(Baudrate2 % 256);
	AUXR |= 0x10;		//定时器2开始计时
	IE2 |= 0x01; // ES2 = 1，允许 UART2 中断
}

	u8 udt;

void UART2_int (void) interrupt 8
{
u8 i;
	
    if(S2RI)
    {
        S2RI = 0;    //Clear Rx flag
	//SBUF=S2BUF;  while (!TI);TI = 0;
			
			
RX2_Buffer[RX2_Cnt]=S2BUF;
if(RX2_Buffer[0]!=0x0f)RX2_Cnt=0;else RX2_Cnt++;
if(RX2_Cnt==25)
{
for (i = 0; i < 25; i++) { sbus[i]=RX2_Buffer[i];}
rcd=1;
RX2_Cnt=0;
}
		
    }

    if(S2TI)
    {
        S2TI = 0;    //Clear Tx flag
//        B_TX2_Busy = 0;
    }
}
char putchar(char ch)
{
  // 等待发送缓冲区准备好（例如，检查发送中断标志位 TI）
  //  while (!(SCON & 0x02)); // 假设使用 SCON 寄存器和 TI 标志位 (具体寄存器名称和位定义取决于你的 251 芯片型号)
  SBUF = ch; // 将字符发送到串口数据寄存器
  while (!TI)
    ;
  TI = 0;
  // 在某些 Keil C51/C251 配置中，你可能需要手动清除 TI 标志，具体取决于库的实现和硬件配置
  // TI = 0;
  return ch; // 返回发送的字符
}

void PortSwitch(void)
{
	P_SW1 &= ~0xc0;						//UART1/USART1: RxD(P3.0), TxD(P3.1)
	P_SW2 &= ~0x01;						//UART2/USART2: RxD2(P1.0), TxD2(P1.1)
	P_SW1 &= ~0x30;						//CAN1: CANRX(P0.0), CANTX(P0.1)
	    P0M0 &= ~0x0c; P0M1 &= ~0x0c; 
    P3M0 &= ~0x0c; P3M1 &= ~0x0c; 

	
}

union
{
  float f;
  unsigned char c[4];
} u;

/******************** 主函数 **************************/
void main(void)
{
	u16 ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10;
	u16 c3,c4;//,c5,c6;
	u8 rcloop=0,mof=0;
	float speed;
EAXFR = 1;
//使能访问 XFR
CKCON = 0x00;
//设置外部数据总线速度为最快
WTST = 0x00;
//设置程序代码等待参数，
//赋值为 0 可将 CPU 执行程序的速度设置为最快
P0M0 = 0x00;
P0M1 = 0x00;
P1M0 = 0x00;
P1M1 = 0x00;
P2M0 = 0x00;
P2M1 = 0x00;
P3M0 = 0x00;
P3M1 = 0x00;
P4M0 = 0x00;
P4M1 = 0x00;
P5M0 = 0x00;
P5M1 = 0x00;

XOSCCR = 0xc0;
//启动外部晶振
while (!(XOSCCR & 1));
//等待时钟稳定
CLKDIV = 0x00;
//时钟不分频
CLKSEL = 0x01;
//选择外部晶振

PortSwitch();	
Uart1_Init();
Uart2_Init();
	CANInit();
	
    EA = 1;             //允许全局中断
		
	CAN_ID = 0x01;
	CAN_ID1 = 0x201;
	TX_BUF[0] = 0x11;
	TX_BUF[1] = 0x22;
	TX_BUF[2] = 0x33;
	TX_BUF[3] = 0x44;
	TX_BUF[4] = 0x55;
	TX_BUF[5] = 0x66;
	TX_BUF[6] = 0x77;
	TX_BUF[7] = 0x88;		
		
RC=0;
MOVE=0;
UD=0;
mof=0;
    printf("STC32G12K128 Starting!\r\n");  //UART2发送一个字符串

//CanSendMsg(CAN_ID,menable);

    while (1)
    {

			
			if(rcd==1)
			{
			rcd=0;
				rcloop++;
				rcloop=rcloop%11;
//				if(rcloop==5)	CanSendMsg(CAN_ID,TX_BUF);
				if(rcloop>5)RC=0;else RC=1;
		if(sbus[23]==0x00){
		RE=0;
			//192 992 1792
		ch1=(sbus[1]|sbus[2]<<8)&0x07FF;
		ch2= (sbus[2] >> 3 | sbus[3] << 5 ) & 0x07FF;
ch3= (sbus[ 3] >> 6 | sbus[4] << 2 | sbus[ 5] << 10 ) & 0x07FF;
ch4= (sbus[ 5] >> 1 | sbus[6] << 7 ) & 0x07FF;
ch5= (sbus[ 6] >> 4 | sbus[7] << 4 ) & 0x07FF;
ch6= (sbus[ 7] >> 7 | sbus[ 8] << 1| sbus[9] <<9 ) & 0x07FF;
ch7= (sbus[9]>>2 |sbus[10]<<6) & 0x07FF;
ch8= (sbus[10]>>5|sbus[11]<<3) & 0x07FF;
ch9 = (sbus[12]|sbus[13]<<8) & 0x07FF;
ch10= (sbus[13]>>3|sbus[14]<<5) & 0x07FF;			
			//printf("%d,%d\r\n",ch1,ch2);

if(ch3!=c3)	{
c3=ch3;
	if(c3<992){CanSendMsg(CAN_ID,menable);mof=1;}else {CanSendMsg(CAN_ID,mdisable);mof=0;}
}		
if(ch4!=c4)	{
c4=ch4;
	CanSendMsg(CAN_ID,mclear);
}		

			speed=(ch2-992.0)*100.0/800.0;
			u.f=speed;
			printf("%f\r\n",speed);	
			mspd[3] = u.c[0];
      mspd[2] = u.c[1];
      mspd[1] = u.c[2];
      mspd[0] = u.c[3];
if(mof==1)CanSendMsg(CAN_ID1,mspd);	

			if(ch2!=992){MOVE=1;
			if(ch2>992){UD=1;}else {UD=0;}
			}else MOVE=0;
		}else{ RE=1;	
CanSendMsg(CAN_ID,mdisable);mof=0;
		}


			}
			

    }
}