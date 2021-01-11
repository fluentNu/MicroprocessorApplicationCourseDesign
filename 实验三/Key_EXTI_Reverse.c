/*******************************************************************************
* 文件名称：按键控制点灯实验(用中断实现)
* 实验内容：1.初始LED灯全灭
*           2.按下B1-B4按钮，对应的LED0-LED3点亮
			3.再次按下按钮，可以实现LED灯的翻转
* 日期版本：2020-12-14
* 程序作者：18计科A1 牛群 20181111707
*******************************************************************************/
#define LED0    GPIO_Pin_8
#define LED1    GPIO_Pin_9
#define LED2    GPIO_Pin_10
#define LED3    GPIO_Pin_11
#define LED4    GPIO_Pin_12
#define LED5    GPIO_Pin_13
#define LED6    GPIO_Pin_14
#define LED7    GPIO_Pin_15
#define LEDALL	GPIO_Pin_All
#include "stm32f10x.h"
uint32_t TimingDelay = 0;
uint8_t EXTI_Status = 0;
int flag1 = 0, flag2 = 0, flag3 = 0, flag4 = 0;
void EXTI_Config(void);
void LED_Init(void);
void LED_Control(uint16_t LED, uint8_t LED_Status);
void LED_Reverse(int* flag, uint16_t LED);
void Delay_Ms(uint32_t nTime);

int main(void)
{
	SysTick_Config(SystemCoreClock / 1000);

	EXTI_Config();
	LED_Init();
	LED_Control(LEDALL, 0);

	while (1);
}

void EXTI_Config(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	//时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//Key1对应PA0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	//外部中断线0
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	//设置EXTI线路为中断请求
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//设置输入线路下降沿为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	//使能 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//设置优先级并使能
	//外部中断线0中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//Key2对应PA8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);

	//外部中断线8
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	//设置EXTI线路为中断请求
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//设置输入线路上升沿为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	//使能
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//设置优先级并使能
	//外部中断线 9-5 中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//Key3对应PB1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	//外部中断线1
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	//设置EXTI线路为中断请求
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//设置输入线路下降沿为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	//使能 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//设置优先级并使能
	//外部中断线1中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//**********Key4对应PB2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);

	//外部中断线2
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	//设置EXTI线路为中断请求
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//设置输入线路下降沿为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	//使能 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//设置优先级并使能
	//外部中断线2中断
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = LED0 | LED1 | LED2 | LED3 | LED4 | LED5 | LED6 | LED7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void LED_Control(uint16_t LED, uint8_t LED_Status)
{
	if (LED_Status == 0) {
		GPIO_SetBits(GPIOC, LED);
		GPIO_SetBits(GPIOD, GPIO_Pin_2);
		GPIO_ResetBits(GPIOD, GPIO_Pin_2);  //状态锁存
	}
	else
	{
		GPIO_ResetBits(GPIOC, LED);
		GPIO_SetBits(GPIOD, GPIO_Pin_2);
		GPIO_ResetBits(GPIOD, GPIO_Pin_2);  //状态锁存    
	}
}

void LED_Reverse(int* flag, uint16_t LED) {
	if (*flag) {
		LED_Control(LED, 0);
		*flag = 0;
	}
	else {
		LED_Control(LED, 1);
		*flag = 1;
	}
}
void Delay_Ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while (TimingDelay != 0);
}

//中断服务函数
void EXTI0_IRQHandler(void)
{
	//检查指定的 EXTI 线路触发请求发生与否
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		LED_Reverse(&flag1, LED0);
		//清除 EXTI 线路挂起位
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		LED_Reverse(&flag2, LED1);
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}
void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		LED_Reverse(&flag3, LED2);
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}
void EXTI2_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		LED_Reverse(&flag4, LED3);
		EXTI_ClearITPendingBit(EXTI_Line2);
	}
}
