/*******************************************************************************
* 文件名称：ADC模拟数字转换实验
* 实验内容：1.LCD屏上同时显示电压和温度
			2.通过串口将检测的电压和温度发送给PC机。串口波特率为9600
* 日期版本：2020-12-28
* 程序作者：18计科A1 牛群 20181111707
*******************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stdio.h"
#include "lcd.h"
#include "misc.h"

uint32_t TimingDelay = 0;
uint8_t ADC_Flag;

void Delay_Ms(uint32_t nTime);
void ADC_Config(void);
float Get_Voltage(void);
float Get_Temperature(void);
void USART2_Init(void);
void USART_SendString(uint8_t* str);

int main(void)
{
	int voltagex;
	float voltage_temp;		//保存电压
	float temperature_temp;	//保存温度
	uint8_t  voltage_str[20];	//电压的字符串结果
	uint8_t  temperature_str[20];	//温度的字符串结果

	SysTick_Config(SystemCoreClock / 1000);  //1ms中断一次

	ADC_Config();
	USART2_Init();

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
		//1秒扫描一次ADC
		//将数字量转换成字符串

		if (ADC_Flag) {
			ADC_Flag = 0;
			voltage_temp = Get_Voltage();
			voltagex = ADC_GetConversionValue(ADC1);
			sprintf((char*)voltage_str, "%s%d", "ADC_CH0_VOL:", voltagex);
			LCD_DisplayStringLine(Line5, voltage_str);
			sprintf((char*)voltage_str, "%s%.3fV", "ADC_CH0_VOL:", voltage_temp);
			USART_SendString(voltage_str);

			LCD_DisplayStringLine(Line6, voltage_str);
			temperature_temp = Get_Temperature();
			sprintf((char*)temperature_str, "%s%.2fC", "Temperature:", temperature_temp);
			temperature_str[18] = 0;
			LCD_DisplayStringLine(Line7, temperature_str);
			USART_SendString(temperature_str);

			Delay_Ms(1000);
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
	/* 检查ADC1复位校准寄存器的结束 */
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	/* 检查ADC1校准结束 */
	while (ADC_GetCalibrationStatus(ADC1));
}

void Delay_Ms(uint32_t nTime)
{
	TimingDelay = nTime;
	while (TimingDelay != 0);
}

float Get_Voltage(void)
{
	float ADC_VALUE;
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	Delay_Ms(5);
	ADC_VALUE = ADC_GetConversionValue(ADC1) * 3.30 / 0xfff;

	return ADC_VALUE;
}

float Get_Temperature(void)
{
	uint16_t tempdata;
	float temperature;
	float volt;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);
	ADC_TempSensorVrefintCmd(ENABLE);  //使能温度传感器

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	tempdata = ADC_GetConversionValue(ADC1);
	volt = (tempdata * 3.3) / 0xfff;

	temperature = (1430 - 1000 * volt) / 4.3 + 25;
	return temperature;
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

void USART_SendString(uint8_t* str)
{
	uint8_t index = 0;
	USART_SendData(USART2, '\r');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	do
	{
		USART_SendData(USART2, str[index]);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		index++;
	} while (str[index] != 0);  //检查字符串结束标志
	USART_SendData(USART2, '\n');
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/******************************************END OF FILE*************************/
