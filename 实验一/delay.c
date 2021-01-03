#include "delay.h"
#include "stm32f10x.h"
#include <stdlib.h>
uint32_t TimingDelay = 0;
static uint16_t fac_ms = 0;					//ms延时倍数

//第一种方法，利用循环延时							
void delay1(void) {
	int i;
	for (i = 0x8f8f8f; i > 0; i--);
}

//第二种方法，SysTick查询延时
void delay2(uint32_t delaytime_ms) {	
	uint32_t temp;
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_ms = SystemCoreClock / 8000;  		//SysTick工作频率为CPU工作频率的8分频
	SysTick->LOAD = delaytime_ms * fac_ms;	//设置RELOAD寄存器重装值
	SysTick->VAL = 0x00;					//使用前将VAL寄存器清0 
	SysTick->CTRL = 0x01;					//开始倒数
	do {
		temp = SysTick->CTRL;	
	} while ((temp & 0x01) && !(temp & (1 << 16)));	//等待时间到达
	SysTick->CTRL = 0x00;
	SysTick->VAL = 0x00;					//使用后完毕后清空
	SysTick_Config(SystemCoreClock / 1000);	//这一步骤很重要，目的是为了中断的正常使用
}

//第三种方法，SysTick中断延时
void delay3(uint32_t delaytime_ms) {
	TimingDelay = delaytime_ms;
	while (TimingDelay != 0);
}

//将三种方法封装成一个函数，并随机调用
void DelayTime() {
	int x;
	x = rand();	 	//生成随机数
	//下面根据生成的随机数随机调用具体的延时方法
	if (x % 3 == 0)
		delay1();
	else if (x % 3 == 1)
		delay2(1000);
	else
		delay3(1000);
}
