/*******************************************************************************
* 文件名称：按键控制点灯实验(用库函数查询实现)
* 实验内容：1.初始LED灯全灭
*           2.按下B1-B4按钮，对应的LED0-LED3点亮
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
#define RB1	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)
#define RB2	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)
#define RB3 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define RB4 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)
#include "stm32f10x.h"
uint32_t TimingDelay = 0;
void Key_Init(void);
void LED_Init(void);
void LED_Control(uint16_t LED, uint8_t LED_Status);
uint8_t Key_Scan(void);
void Delay_Ms(uint32_t nTime);

int main(void)
{
	uint8_t key_temp;

	SysTick_Config(SystemCoreClock / 1000);  //1ms中断一次

	Key_Init();  //按键接口初始化
	LED_Init();
	LED_Control(LEDALL, 0);

	while (1) {
		key_temp = Key_Scan();
		switch (key_temp)
		{
		case '1':
			LED_Control(LEDALL, 0);
			LED_Control(LED0, 1);
			break;
		case '2':
			LED_Control(LEDALL, 0);
			LED_Control(LED1, 1);
			break;
		case '3':
			LED_Control(LEDALL, 0);
			LED_Control(LED2, 1);
			break;
		case '4':
			LED_Control(LEDALL, 0);
			LED_Control(LED3, 1);
			break;
		}
		key_temp = 0;  //清除按键
	}
}

void Key_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//B1、B2按键引脚配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//B3、B4按键引脚配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t Key_Scan(void)
{
	uint8_t key_value = 0xff;

	if (RB1 == 0) {
		key_value = '1';
	}
	if (RB2 == 0) {
		key_value = '2';
	}
	if (RB3 == 0) {
		key_value = '3';
	}
	if (RB4 == 0) {
		key_value = '4';
	}
	return key_value;
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

void Delay_Ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while (TimingDelay != 0);
}
