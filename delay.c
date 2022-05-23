//writer : bonus adityas (bonusonic@gmail.com)
//23 august 2016

#include "delay.h"
#include "REG/TIMER/stm8s_tim4.h"

void delay_init()
{
	TIM4_PSCR = 4; // CLK/16
}

void delay_us(unsigned long delus)
{
	unsigned int du;
	
	for(du=0;du<(delus/10);du++)
	{
		delay_timer(100);
	}
	delay_timer(delus%10);
}

void delay_ms(unsigned long delms)
{
	unsigned long dm;

	for(dm=0;dm<(delms*100);dm++)
	{
		delay_timer(100);
	}
}

void delay_timer(unsigned char deltim)
{
	TIM4_CR1 = (1<<TIM4_CR1_CEN);
	while(TIM4_CNTR<deltim);
	TIM4_CR1 = (0<<TIM4_CR1_CEN);
	TIM4_CNTR = 0; //reset timer	
}
