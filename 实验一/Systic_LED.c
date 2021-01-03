/*******************************************************************************
* 文件名称：LED闪烁控制实验
* 实验内容：1.LED1-LED8依次点亮
*           2.每次点亮时间持续1秒
* 日期版本：2020-11-22
* 程序作者：18计科A1 牛群 20181111707
*******************************************************************************/
#include "stm32f10x.h"
#include "led.h"
#include "delay.h"

void LED_Control(uint16_t LED, uint8_t LED_Status);
void LED_Init(void);

int main(void)
{
	unsigned int i;
	int lightingLED;	//控制哪个LED灯亮

	//初始化
	LED_Init();
	LED_Control(LEDALL, 0);
	SysTick_Config(SystemCoreClock / 1000);

	while (1) {
		lightingLED = 1;				//选中第一个灯
		for (i = 1; i <= 8; i++) {
			GPIOC->ODR |= 0xFF << 8;	//关闭所有LED
			GPIOC->ODR &= ~(lightingLED << 8);	//选中的LED亮
			GPIOD->BSRR = GPIO_Pin_2;	//PD2置位，锁存器输出随输入变化 
			GPIOD->BRR = GPIO_Pin_2;	//PD2清0，状态锁存
			DelayTime();				//延时1秒，具体采用的方法是随机的(见函数内部实现)
			lightingLED = lightingLED << 1;		//左移，选中下一个LED
		}
	}
}

void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	//LED引脚配置，PC08~PC15
	GPIO_InitStructure.GPIO_Pin = LED0 | LED1 | LED2 | LED3 | LED4 | LED5 | LED6\
		| LED7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//74HC573锁存引脚配置，PD2
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
