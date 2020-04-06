//2020.04.02 
//code by Xuejun Liu
//pwm OUTPUT PD2
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#define XTAL       16000000UL
//define busses prescalers
#define AHB_PRE    1
#define APB1_PRE   2
#define APB2_PRE   1
#define SysTickClk 20000 
//calculate peripheral frequencies
#define SYSCLK     73000000
#define AHB        SYSCLK/AHB_PRE
#define APB1       AHB/APB1_PRE
#define APB1_TIM   APB1*2
#define APB2       AHB/APB2_PRE
#define APB2_TIM   APB2*1
#define SysTicks   AHB/SysTickClk
unsigned char LED1=0;
void MY_NVIC_SetVectorTable(uint32_t NVIC_VectTab,uint32_t Offset)	 
{ 	   	  
	SCB->VTOR=NVIC_VectTab|(Offset&(uint32_t)0xFFFFFE00);//Set the vector table offset register of NVIC, 
	                                                     //the lower 9 bits of vtor are reserved, that is, [8:0] is reserved.
}
//Set NVIC group
//NVIC_Group:NVIC group 0-4      5 groups in total 		   
void MY_NVIC_PriorityGroupConfig(uint16_t NVIC_Group)	 
{ 
	uint32_t temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//Take the last three position.
	temp1<<=8;
	temp=SCB->AIRCR;  //Read previous settings
	temp&=0X0000F8FF; //clear previous group
	temp|=0X05FA0000; 
	temp|=temp1;	   
	SCB->AIRCR=temp;  	    	  				   
}
//设置NVIC 
//NVIC_PreemptionPriority:
//NVIC_SubPriority       
//NVIC_Channel           
//NVIC_Group            
void MY_NVIC_Init(uint16_t NVIC_PreemptionPriority,uint16_t NVIC_SubPriority,uint16_t NVIC_Channel,uint16_t NVIC_Group)	 
{ 
	uint32_t temp;	  
	MY_NVIC_PriorityGroupConfig(NVIC_Group);
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;								
	NVIC->ISER[NVIC_Channel/32]|=1<<NVIC_Channel%32;
	NVIC->IP[NVIC_Channel]|=temp<<4;				 	    	  				   
} 

void gpio_init()           //GPIO initialization
{
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN|//Turn on the clock of gpioa and gpiob and gpiod
	            RCC_AHB1ENR_GPIOBEN|
	            RCC_AHB1ENR_GPIODEN;
GPIOB -> MODER = 0x00005555;//GPIOB 0-7  output
GPIOD -> MODER = 0x00000055;//GPIOD 0-3 OUTPUT
}
 void clock_set()
{
FLASH->ACR|=FLASH_ACR_PRFTEN;
FLASH->ACR|=FLASH_ACR_LATENCY_2WS;//FLASH PREFETCH

RCC->CFGR&=~RCC_CFGR_HPRE_Msk;
RCC->CFGR|=RCC_CFGR_HPRE_DIV1;
RCC->CFGR&=~RCC_CFGR_PPRE1_Msk;
RCC->CFGR|=RCC_CFGR_PPRE1_DIV1;
RCC->CFGR&=~RCC_CFGR_PPRE2_Msk;
RCC->CFGR|=RCC_CFGR_PPRE2_DIV1;

RCC->PLLCFGR&=~RCC_PLLCFGR_PLLM_Msk;
RCC->PLLCFGR|=8<<RCC_PLLCFGR_PLLM_Pos;
RCC->PLLCFGR&=~RCC_PLLCFGR_PLLN_Msk;
RCC->PLLCFGR|=73<<RCC_PLLCFGR_PLLN_Pos;
RCC->PLLCFGR&=~RCC_PLLCFGR_PLLP_Msk;
RCC->CR|=RCC_CR_PLLON;
while((RCC->CR&RCC_CR_PLLRDY)==0)
{}
RCC->CFGR&=(uint32_t)((uint32_t)~(RCC_CFGR_SW));
RCC->CFGR|=RCC_CFGR_SW_PLL;
while((RCC->CFGR&RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL)
{}
}
//Timer 3 interrupt service routine 
void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//溢出中断
	{
		if(GPIOD->ODR&GPIO_ODR_OD2)
			{
			GPIOD->BSRR|=GPIO_BSRR_BR2;
			}
			else
			{
			GPIOD->BSRR|=GPIO_BSRR_BS2;   
			}		    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//Universal timer 3 interrupt initialization
//Here, the clock selection is 2 times of APB1, while APB1 is 36.5m
//arr
//psc：
//Calculation method of timer overflow time:Tout=((arr+1)*(psc+1))/Ft us.
void TIM3_Int_Init(uint16_t arr,uint16_t psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr;  	//设定计数器自动重装值 
	TIM3->PSC=psc;  	//预分频器	  
	TIM3->DIER|=1<<0;   //允许更新中断	  
	TIM3->CR1|=0x01;    //使能定时器3
  	MY_NVIC_Init(1,3,TIM3_IRQn,2);	//抢占1，子优先级3，组2									 
}
void SysTick_Handler(void)
{
			if(GPIOD->ODR&GPIO_ODR_OD2)
			{
			GPIOD->BSRR|=GPIO_BSRR_BR2;
			}
			else
			{
			GPIOD->BSRR|=GPIO_BSRR_BS2;   
			}

}
int main()
{ 
//  clock_set();
	clock_set();
  gpio_init();
 // TIM3_Int_Init(73-1,50-1);//20khz
	GPIOD->ODR=0X00000001;
	SysTick_Config(SysTicks);
  while(1)
  {
	
  };
}

