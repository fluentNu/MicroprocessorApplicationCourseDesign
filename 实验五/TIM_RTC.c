/*******************************************************************************
* 文件名称：定时器应用实验
* 实验内容：1.利用定时器4，实现在液晶屏显示时：分：秒的功能,同时实现每秒流水灯移动一次.
			2.利用定时器2，实现在PA1引脚输出1KHz,占空比为0.3的波形.
			3.显示时间使用RTC实现.
* 日期版本：2021-1-3
* 程序作者：18计科A1 牛群 20181111707
*******************************************************************************/

#include "stm32f10x.h"
#include "lcd.h"
#include "stdio.h"
#include "led.h"
#include "stm32f10x_rtc.h"
uint32_t TimingDelay = 0;
uint16_t Channel2Pulse = 0;
uint8_t Time[16] = __TIME__;
uint8_t Date[16] = __DATE__;
uint8_t __50ms;
extern uint8_t leds;
void Delay_Ms(uint32_t nTime);
void NVIC_Configuration(void);
void TIM_Config(void);
void LED_Init(void);
void TIM_PWM_Config(uint16_t Channel2Pulse);
void PWM_IO_Config(void);
void RTC_Configuration(void);
void Time_Display(u32 TimeVar);

int main(void)
{
	uint8_t  string[20];  //
	SysTick_Config(SystemCoreClock / 1000);  //1ms中断一次

	TIM_Config();
	LED_Init();
	PWM_IO_Config();
	RTC_Configuration();
	TIM_PWM_Config(998 * 3 / 10);
	//LCD工作模式配置
	STM3210B_LCD_Init();
	LCD_Clear(White);
	LCD_SetTextColor(White);
	LCD_SetBackColor(Blue);

	LCD_ClearLine(Line0);
	LCD_ClearLine(Line1);
	LCD_ClearLine(Line2);
	LCD_ClearLine(Line3);
	LCD_ClearLine(Line4);

	LCD_DisplayStringLine(Line1, "  TIM base and PWM  ");
	LCD_DisplayStringLine(Line2, "     fluent Nu      ");
	LCD_DisplayStringLine(Line3, "    20181111707     ");

	LCD_SetTextColor(Blue);
	LCD_SetBackColor(White);

	while (1) {

		GPIO_Write(GPIOC, ~(1 << (leds + 7)));
		GPIO_SetBits(GPIOD, GPIO_Pin_2);
		GPIO_ResetBits(GPIOD, GPIO_Pin_2);

		sprintf((char*)string, "%s%d", "LED ON:LD", leds);
		LCD_DisplayStringLine(Line6, string);
		LCD_DisplayStringLine(Line7, "PA1-PWMVALUE:30%");
		//LCD_DisplayStringLine(Line8, Time);//与另一种实现方式不同，这行被注释掉了
		Time_Display(RTC_GetCounter());		 //现在是这行代码用来显示时间
		LCD_DisplayStringLine(Line9, Date);
	}
}

void TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* TIM4 时钟使能 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//中断向量配置
	NVIC_Configuration();

	/* TIM基础配置 */
	TIM_TimeBaseStructure.TIM_Period = 50000;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	//TIM4预分频设置:1MHZ,APB1分频系数2，TIM4时钟为36MHzx2 = 72MHz  
	TIM_PrescalerConfig(TIM4, 71, TIM_PSCReloadMode_Immediate);

	//通用定时器TIM4中断配置
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	/* TIM4 使能 */
	TIM_Cmd(TIM4, ENABLE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 使能TIM4全局中断 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
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

void TIM_PWM_Config(uint16_t Channel2Pulse)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* TIM2 时钟使能 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* TIM基础配置 */
	TIM_TimeBaseStructure.TIM_Period = 999;  //1KHz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	//TIM2预分频设置:1MHZ,APB1分频系数2  
	TIM_PrescalerConfig(TIM2, 71, TIM_PSCReloadMode_Immediate);

	/* 配置通道2 通道2映射到PA1，可以查看PPT21页或数据手册 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	//使能TIM2定时计数器
	TIM_Cmd(TIM2, ENABLE);
	//使能TIM2 PWM输出模式
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void PWM_IO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
uint8_t text[20];
void Time_Display(u32 TimeVar)
{
	uint32_t Hour = 0, Minute = 0, Second = 0;
	Hour = TimeVar / 3600;
	Minute = (TimeVar % 3600) / 60;
	Second = (TimeVar % 3600) % 60;
	sprintf((char*)text, "Time: %0.2d:%0.2d:%0.2d", Hour, Minute, Second);
	LCD_DisplayStringLine(Line8, text);
}
void RTC_Configuration(void)
{
	int HH = (Time[0] - '0') * 10 + Time[1] - '0';
	int MM = (Time[3] - '0') * 10 + Time[4] - '0';
	int SS = (Time[6] - '0') * 10 + Time[7] - '0';
	/* 使能PWR和BKP时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	PWR_BackupAccessCmd(ENABLE);
	BKP_DeInit();
	RCC_LSICmd(ENABLE);
	/*等待*/
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	RCC_RTCCLKCmd(ENABLE);

	RTC_WaitForSynchro();
	RTC_WaitForLastTask();
	/*使能RTC*/
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	RTC_WaitForLastTask();
	/*设置RTC时间间隔为1s*/
	RTC_SetPrescaler(39999);
	RTC_WaitForLastTask();

	RTC_SetCounter(HH * 3600 + MM * 60 + SS);
	RTC_WaitForLastTask();
}
void Delay_Ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while (TimingDelay != 0);
}

//TIM4中断服务函数
void TIM4_IRQHandler(void)
{
	if (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) == SET) {
		//清除标志位
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		if (++__50ms == 20) {
			__50ms = 0;
			if (++leds == 9) {
				leds = 1;
			}
		}
	}
}

void RTC_IRQHandler(void)
{
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
		RTC_ClearITPendingBit(RTC_IT_SEC);
		RTC_WaitForLastTask();
		/* 23:59:59 */
		if (RTC_GetCounter() == 0x00015180) {
			RTC_SetCounter(0x0);
			RTC_WaitForLastTask();
		}
	}
}
