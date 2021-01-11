/*******************************************************************************
* 文件名称：ADC模拟数字转换实验
* 实验内容：1.LCD屏上同时显示电压和温度
			2.通过串口将检测的电压和温度发送给PC机。串口波特率为9600
* 日期版本：2020-01-11
* 程序作者：18计科A1 牛群 20181111707
*******************************************************************************/
#include "stm32f10x.h"
#include "lcd.h"
#include "stdio.h"
#include "stm32f10x_usart.h"

uint32_t TimingDelay = 0;
uint8_t ADC_Flag;
uint8_t LCD_Flag = 0;
uint8_t USART_Falg = 0;
uint8_t USART_RXBUF;
float tempADC;
void Delay_Ms(uint32_t nTime);
void ADC_Config(void);
void USART_Config(void);
void NVIC_Configuration(void);
float Read_ADC(void);
void USART_SendString(uint8_t* str);

int main(void)
{
	uint8_t  string[20];  //ADC结果

	SysTick_Config(SystemCoreClock / 1000);  //1ms中断一次

	USART_Config();
	ADC_Config();

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
	LCD_ClearLine(Line5);
	LCD_ClearLine(Line6);
	LCD_ClearLine(Line7);
	LCD_ClearLine(Line8);

	LCD_DisplayStringLine(Line1, "   ADC experiment   ");
	LCD_DisplayStringLine(Line2, "     fluent Nu      ");
	LCD_DisplayStringLine(Line3, "    20181111707     ");
	LCD_DisplayStringLine(Line9, "     2020.12.28     ");

	while (1) {
		if (USART_Falg == 1) {
			USART_Falg = 0;
			if (USART_RXBUF != '\0') {
				tempADC = Read_ADC();
			}
			if (LCD_Flag) {
				LCD_Flag = 0;
				sprintf((char*)string, "%s%.3f", "ADC Value:", tempADC);
				LCD_DisplayStringLine(Line7, string);
				USART_SendString(string);
			}
			ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
			USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		}
	}
}

void ADC_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//PB0-ADC channel 8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ADC1 工作模式配置
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  //单次转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_13Cycles5);

	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));

	NVIC_Configuration();
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);		 //开启ADC中断
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

float Read_ADC(void)
{
	float ADC_VALUE;
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	Delay_Ms(5);
	ADC_VALUE = ADC_GetConversionValue(ADC1) * 3.30 / 0xfff;
	return ADC_VALUE;
}

void Delay_Ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while (TimingDelay != 0);
}
void ADC1_2_IRQHandler(void)
{
	if (ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		LCD_Flag = 1;
		ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
	}
}
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}
void USART_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	//配置USART2 TX引脚工作模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置USART2 RX引脚工作模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//串口2工作模式配置
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);

}
void USART_SendString(uint8_t* str)
{
	uint8_t index = 0;
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	do
	{
		USART_SendData(USART2, str[index]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		index++;
	} while (str[index] != 0);  //检查字符串结束标志
	USART_SendData(USART2, '\r');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, '\n');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		USART_RXBUF = USART_ReceiveData(USART2);
		USART_Falg = 1;
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	}
}
/******************************************END OF FILE*************************/
