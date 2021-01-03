/*******************************************************************************
* 文件名称：USART2-数据接收、发送实验-用查询方式
* 实验内容：1.打开Tera Term软件，将波特率设置为9600
*           2.download该程序，Tera Term上会出现欢迎界面
			3.键盘输入1，返回作者的学号；输入2，返回作者的姓名；其他输入会被视为非法，返回错误提示。
* 日期版本：2020-12-06
* 程序作者：18计科A1 牛群 20181111707
*******************************************************************************/

#include "stm32f10x.h"

void USART2_Init(void);					//USART2发送初始化
uint16_t Receive_Char(void);			//字符接受函数
void Send_Char(uint16_t ch);			//字符发送函数
void Send_String(char s[]);				//字符串发送函数

int main(void) {
	char s[] = "This is my USART test program.";
	uint16_t number;	  
	USART2_Init();
	Send_String(s);
	while (1) {
		number = Receive_Char();
		if (number == '1') {
			Send_String("\n\r20181111707");	//20181111707 是我的学号
		}
		else if (number == '2') {
			Send_String("\n\rfluent Nu");	//fluent Nu 是我的英文名		
		}
		else {								//错误提示
			Send_String("\n\rDATA ERROR! Please enter character '1' or '2'!");
		}
	}
}


/**
  * @说明     USART2相关GPIO和工作模式配置
  * @参数     None
  * @返回值   None
  */
void USART2_Init(void) {	//USART2发送初始化
	//使能USART2对应的GPIOA时钟
	//等效于库函数：RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC->APB2ENR |= (1 << 2);
	//等效于库函数：RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	
	RCC->APB1ENR |= (1 << 17);	//使能USART2外设对应的时钟
	//USART2发送引脚PA2(TX2)	USART2接收引脚PA3(RX2)
	//设置发送引脚工作模式：复用推挽50MHz(B:1011)
	//设置接收引脚工作模式：复用浮空输入(4:0100)
	//参考STM32数据手册，这里USART2的TX和RX分别对应PA2和PA3
	GPIOA->CRL &= 0xFFFF00FF;
	GPIOA->CRL |= 0x00004B00;

	RCC->APB1RSTR |= (1 << 17);		//复位USART2
	RCC->APB1RSTR &= ~(1 << 17);	//停止复位

	//串口2工作模式配置: APB1时钟36MHz 波特率9600bit/s
	USART2->BRR = 0xEA6;	//DIV=234.375
	//发送使能、接受使能
	USART2->CR1 = 0x200C;  	
	//使能USART2(其实在上一步中已经使能)
	USART2->CR1 |= (1<<13);			  	
}

/**
  * @说明     USART2字符接受函数
  * @参数     None
  * @返回值   data：接收到的字符
  */
uint16_t Receive_Char(void) {
	uint16_t data;
	//状态寄存器(USART_SR)位5为RXNE：读数据寄存器非空 (Read data register not empty)
	//0：数据没有收到；1：收到数据，可以读出。
	while(1){
		if((USART2->SR & (1 << 5)) != 0){ //取出RXNE为状态判断是否非0
			break;
		}
	}  
	//把RXNE置零
	USART2->SR &= ~(1 << 5);
	data = (uint16_t)(USART2->DR & 0x01FF);
	return data;
}

/**
  * @说明     USART2字符发送函数
  * @参数     ch: 待发送的字符
  * @返回值   None
  */
void Send_Char(uint16_t ch) {	//字符发送函数
	//状态寄存器(USART_SR)位6为TC：发送完成 (Transmission complete)。
	//0：发送还未完成；1：发送完成。
	while ((USART2->SR & (1 << 6)) == 0);		//等待上一个字符发送完毕	
	USART2->DR = (ch & (uint16_t)0x01FF);		//发送本次的char
}

/**
  * @说明     USART2字符串发送函数
  * @参数     str: 指向字符串的指针
  * @返回值   None
  */
void Send_String(char str[]) {				//字符串发送函数
	int index = 0;
	while (str[index] != '\0') {			//遍历该字符串
		Send_Char((uint16_t)str[index]);	//调用字符发送函数
		index++;
	}
}
