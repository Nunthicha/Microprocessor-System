#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_lcd.h"
#include "stm32l152_glass_lcd.h"
#define LINE1 11
#define LINE2 12

void SystemClock_Config(void);

typedef enum {false, true}bool;

// Variable
uint8_t player1 = 0; //DATA player1
uint8_t player2 = 0; //DATA player2
uint16_t adc_data = 0; //check ADC
bool sound_on;
char text[7];
// POTENTIOMETER SETUP 
void setup_POTEN();
bool is_switch_on();
// BUTTON SETUP
void setup_BUTTON();
// EXTI SETUP
void MS_EXTIConfig(void);
void EXTI0_IRQHandler();
void EXTI1_IRQHandler();
void EXTI2_IRQHandler();
// LCD SETUP
void setup_LCD();
void set_text(char[6]);
// BUZZER SETUP
void setup_Buzzer();
void set_sound(bool);
// LCD BUZZER OUTPUT
void output(bool);
// GAME CHECK
bool check_result(uint8_t,uint8_t);
bool player1_Button_pressed();
bool player2_Button_pressed();
void check_reset();
bool game_run = true;

bool dalay(unsigned int);
void sleep(unsigned int);



int main()
{
	setup_Buzzer();
	setup_POTEN();
	setup_LCD();
	MS_EXTIConfig();
	setup_BUTTON();
	while(1)
	{
		if(is_switch_on())
		{
			if (game_run)
			{
				if(check_result(player1,player2))
				{
					GPIOA->ODR ^= (3<<11);
				}
				else
				{
					set_sound(true);
					output(true);
					sleep(200);
					set_sound(false);
					output(true);
					sleep(200);
					set_sound(true);
					output(true);
					sleep(200);
					set_sound(false);
					output(true);
					game_run = false;
				}
			}
		}
		check_reset();
		output(true);
	}
}

void setup_POTEN()
{
	RCC->AHBENR |= (1<<0);
	GPIOA->MODER |= (3<<8);
	RCC->CR |= (1<<0);
	while(((RCC->CR & 0x02) >> 1) == 1);
	RCC->APB2ENR |= (1<<9);
	
	ADC1 -> CR1 |=  (1<<24)|(1<<11);
	ADC1 -> CR1 &= ~(7<<13);
	ADC1 -> CR2 &= ~(1<<11);
	ADC1 -> SMPR3 |= (2<<12);
	ADC1 -> SQR5 |= (4<<0);
	ADC1 -> CR2 |= (1<<0);
}

void check_reset()
{
	static bool turn = false;
	ADC1->CR2 |= (1<<30);
	while((ADC1->SR & (1<<1)) == 0);
	adc_data = ADC1->DR;
	if (ADC1->DR == 0x03FF && !turn)
	{
		set_text("      ");
		set_text("START");
		game_run = true;
		player1 = 0;
		player2 = 0;
		turn = true;
	}
	else if (ADC1->DR != 0x03FF && turn)
	{
		set_text("      ");
		game_run = false;
		player1 = 0;
		player2 = 0;
		turn = false;
	}
}

bool is_switch_on()
{
	ADC1->CR2 |= (1<<30);
	while((ADC1->SR & (1<<1)) == 0);
	adc_data = ADC1->DR;
	if (ADC1->DR == 0x03FF)
		return true;
	else if (ADC1->DR != 0x03FF)
	{
		set_text("      ");
		return false;
	}
		
}

void MS_EXTIConfig(void)
{
	RCC->APB2ENR |= (1<<0);
	//clear EXTICR and set PA0
	SYSCFG-> EXTICR[0] &= ~((15<<0) | (15<<4) | (15<<8));
	//set PC1 PD2
	SYSCFG-> EXTICR[0] |= ( (2<<4) | (3<<8));
	EXTI->IMR |= ((1<<0) | (1<<1) | (1<<2));
	EXTI->FTSR |= ((1<<0) | (1<<1) | (1<<2));
	
	//Enable IQR
	NVIC_EnableIRQ((IRQn_Type)6);
	NVIC_SetPriority((IRQn_Type)6, 0);
	NVIC_EnableIRQ((IRQn_Type)7);
	NVIC_SetPriority((IRQn_Type)7, 0);
	NVIC_EnableIRQ((IRQn_Type)8);
	NVIC_SetPriority((IRQn_Type)8, 0);
}

void setup_BUTTON(void)
{
	//OPEN PORT PA PC PD
	RCC->AHBENR |= ((1<<0) | (1<<2) | (1<<3)); 
	//SET OUTPUT (SINK LINE) PA11,12
	GPIOA->MODER |= (1<<22) | (1<<24);
	//SET INPUT (INPUT LINE) PA0 PC1 PD2
	GPIOA->MODER &= ~(3<<0);
	GPIOC->MODER &= ~(3<<2);
	GPIOD->MODER &= ~(3<<4);
	
	GPIOA->ODR &= ~(1<<LINE1);
	GPIOA->ODR |= (1<<LINE2);
}

void EXTI0_IRQHandler()
{
	EXTI->PR |= (1<<0);
	if((GPIOA->ODR & (1<<LINE1)) == 0)
	{
		if (player1_Button_pressed() && is_switch_on())
		{
			player1 = 1;
		}
	}
	else
	{
		if (player2_Button_pressed() && is_switch_on())
		{
			player2 = 1;
		}
	}
}
void EXTI1_IRQHandler()
{
	EXTI->PR |= (1<<1);
	if((GPIOA->ODR & (1<<LINE1)) == 0)
	{
		if (player1_Button_pressed() && is_switch_on())
		{
			player1 = 2;
		}
	}
	else 
	{
		if (player2_Button_pressed() && is_switch_on())
		{
			player2 = 2;
		}
	}
}

void EXTI2_IRQHandler()
{
	EXTI->PR |= (1<<2);
	if((GPIOA->ODR & (1<<LINE1)) == 0)
	{
		if (player1_Button_pressed() && is_switch_on())
		{
			player1 = 3;
		}
	}
	else 
	{
		if (player2_Button_pressed() && is_switch_on())
		{
			player2 = 3;
		}
	}
}


void setup_LCD()
{
	SystemClock_Config();
	LCD_GLASS_Init();
}

void setup_Buzzer()
{
	RCC->AHBENR |= (1<<0);
	GPIOA->MODER |= (1<<10);
}

void set_text(char _text[7])
{
	sprintf(text, _text);
}

void set_sound(bool state)
{
	sound_on = state;
}

void output(bool blink)
{
	//output sound
	if(sound_on)
	{
		GPIOA->ODR |= (1<<5);
	}
	else
		GPIOA->ODR &= ~(1<<5);
	
	//display text on LCD
	if(blink)
	{
		if(dalay(20))
		{
			LCD_GLASS_DisplayString((uint8_t*)text);
		}
		else
		{
			LCD_GLASS_DisplayString((uint8_t*)"      ");
		}
	}
	else
		LCD_GLASS_DisplayString((uint8_t*)text);
}

bool dalay(unsigned int delay)
{
	static unsigned int i = 0;
	static char c = 0;
	bool timeout = false;
	
	if(c%2)
	{
		timeout = true;
	}
	else
		timeout = false;
	i++;
	if(i > delay)
	{
		i = 0;
		c++;
	}
	return timeout;
} 

void sleep(unsigned int sec)
{
	LL_mDelay(sec);
}

bool check_result(uint8_t p1,uint8_t p2)
{
	switch(p1)
	{
		case 1: if (p2 == 1)
						{
							set_text("^ DD ^"); 
							return 0;
						}
						else if (p2 == 2) 
						{
							set_text("X LW ^"); 
							return 0;
						}
						else if (p2 == 3) 
						{
							set_text("0 WL ^"); 
							return 0;
						}
						break;
		case 2: if (p2 == 1)
						{
							set_text("^ WL X"); 
							return 0;
						}
						else if (p2 == 2) 
						{
							set_text("X DD X"); 
							return 0;
						}
						else if (p2 == 3) 
						{
							set_text("0 LW X"); 
							return 0;
						}
						break;
		case 3: if (p2 == 1)
						{
							set_text("^ LW 0"); 
							return 0;
						}
						else if (p2 == 2) 
						{
							set_text("X WL 0"); 
							return 0;
						}
						else if (p2 == 3) 
						{
							set_text("0 DD 0"); 
							return 0;
						}
						break;

	}
	return 1;
}

bool player1_Button_pressed()
{
	if (player1 == 0)
		return true;
	else
		return false;
}

bool player2_Button_pressed()
{
	if (player2 == 0)
		return true;
	else
		return false;
}

/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}
