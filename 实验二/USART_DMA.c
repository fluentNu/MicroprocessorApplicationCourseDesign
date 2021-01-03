/*******************************************************************************
* 文件名称：USART2-数据接收、发送实验-用DMA方式实现
* 实验内容：1.打开Tera Term软件，将波特率设置为9600
*           2.download该程序，Tera Term上会出现欢迎界面
			3.键盘输入1，返回作者的学号；输入2，返回作者的姓名；其他输入会被视为非法，返回错误提示。
* 日期版本：2020-12-11
* 程序作者：18计科A1 牛群 20181111707
* 配置基本采用库函数实现
*******************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "misc.h"
char Hello[] = "This is my USART test program through DMA.";	//欢迎页面
char Number[] = "\n\r20181111707";								//学号
char Name[] = "\n\rfluent Nu";									//姓名	
char Tip[] = "\n\rDATA ERROR! Please enter character '1' or '2'!";//非法输入时的提示
char ReceData[1] = "";	//接收字符串，其实这里用char来实现会更好
void NVIC_Configuration(void);	  	//NVIC配置初始化
void USART2_Init(void);				//USART2初始化
//DMA配置初始化
void DMA_Config(DMA_Channel_TypeDef* DMA_Channel, char* StringBuffer, uint32_t size, uint32_t DMA_Dir);
//DMA开始发送
void DMA_Start_Send(DMA_Channel_TypeDef* DMA_Channel, FunctionalState status);
//DMA开始接收
void DMA_Start_Recieve(DMA_Channel_TypeDef* DMA_Channel);


int main(void)
{

	DMA_Channel_TypeDef* Send_DMA_Ch = DMA1_Channel7;
	DMA_Channel_TypeDef* Recieve_DMA_Ch = DMA1_Channel6;

	NVIC_Configuration();

	USART2_Init();

	//先发送一个欢迎页面的字符串
	DMA_Config(Send_DMA_Ch, Hello, sizeof(Hello), DMA_DIR_PeripheralDST);
	DMA_Start_Send(DMA1_Channel7, ENABLE);
	//配置接收通道
	DMA_Config(Recieve_DMA_Ch, ReceData, sizeof(ReceData), DMA_DIR_PeripheralSRC);
	//接收通道的中断开启	
	DMA_ITConfig(Recieve_DMA_Ch, DMA_IT_TC, ENABLE);
	Recieve_DMA_Ch->CCR |= DMA_CCR6_CIRC;  	//设置为循环模式
	//开始接收 
	DMA_Start_Recieve(Recieve_DMA_Ch);
	while (1);
}

void DMA_Config(DMA_Channel_TypeDef* DMA_Channel, char* StringBuffer, uint32_t size, uint32_t DMA_Dir)
{
	DMA_InitTypeDef 	 DMA_InitStruct;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 	//使能时钟

	//外设基地址
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
	//定义MDA内存基地址
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)(StringBuffer);
	//定义外设是作为传输目的地还是来源，DST是目的地，SRC是来源
	DMA_InitStruct.DMA_DIR = DMA_Dir;
	//缓冲区大小
	DMA_InitStruct.DMA_BufferSize = size;
	//外设地址寄存器递增与否，这里选择否
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//内存基地址递增
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据宽度为8位(位9:8)
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//内存数据宽度为8位(位11:10)
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//工作在正常缓存模式(位5)
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	//最低优先级(位13:12)
	DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
	//不设置内存到内存传输(位14)
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA_Channel, &DMA_InitStruct);
}

void DMA_Start_Send(DMA_Channel_TypeDef* DMA_Channel, FunctionalState status)
{
	DMA_Cmd(DMA_Channel, status);

	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}
void DMA_Start_Recieve(DMA_Channel_TypeDef* DMA_Channel)
{
	DMA_Cmd(DMA_Channel, ENABLE);	//开启DMA通道

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
}

void DMA1_Channel6_IRQHandler(void)
{
	//清除中断标志位
	DMA_ClearITPendingBit(DMA1_IT_GL6);
	DMA_Start_Send(DMA1_Channel7, DISABLE);

	if (ReceData[0] == '1')
		DMA_Config(DMA1_Channel7, Number, sizeof(Number), DMA_DIR_PeripheralDST);
	else  if (ReceData[0] == '2')
		DMA_Config(DMA1_Channel7, Name, sizeof(Name), DMA_DIR_PeripheralDST);
	else
		DMA_Config(DMA1_Channel7, Tip, sizeof(Tip), DMA_DIR_PeripheralDST);

	DMA_Start_Send(DMA1_Channel7, ENABLE);
}

void USART2_Init(void) {	//USART2发送初始化
	//使能USART2对应的GPIOA时钟	
	RCC->APB2ENR |= (1 << 2);
	//使能USART2外设对应的时钟
	RCC->APB1ENR |= (1 << 17);
	//USART2发送引脚PA2(TX2)	USART2接收引脚PA3(RX2)
	//设置发送引脚工作模式：复用推挽50MHz(B:1011)
	//设置接收引脚工作模式：复用浮空输入(4:0100)
	//参考STM32数据手册，这里USART2的TX和RX分别对应PA2和PA3
	GPIOA->CRL &= 0xFFFF00FF;
	GPIOA->CRL |= 0x00004B00;

	//串口2工作模式配置: APB1时钟36MHz 波特率9600bit/s
	USART2->BRR = 0xEA6;	//DIV=234.375
	//USART2->BRR = 0x753;  //Baud=19200
	//字长设置为8位
	USART2->CR1 &= ~(1 << 12);
	//停止位设置为1个停止位
	USART2->CR2 &= ~((1 << 12) | (1 << 13));
	//发送使能、接收使能
	USART2->CR1 = 0x200C;
}

void NVIC_Configuration(void)	//中断设置
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//设置优先级分组	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//DMA通道6中断
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	//先占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//NVIC使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC初始化
	NVIC_Init(&NVIC_InitStructure);
}
