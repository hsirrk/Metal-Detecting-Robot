#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include "../Common/Include/serial.h"
#include "UART2.h"
#include <math.h>
#include <string.h>


#define SYSCLK 32000000L

#define F_CPU 32000000L
#define DEF_F 100000L // 10us tick

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3
//       PA2 -|8       25|- PA15 (Used for RXD of UART2, connects to TXD of JDY40)
//       PA3 -|9       24|- PA14 (Used for TXD of UART2, connects to RXD of JDY40)
//       PA4 -|10      23|- PA13 (Used for SET of JDY40)
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10 (Reserved for RXD of UART1)
//       PB0 -|14      19|- PA9  (Reserved for TXD of UART1)
//       PB1 -|15      18|- PA8  (pushbutton)
//       VSS -|16      17|- VDD
//             ----------

#define F_CPU 32000000L

// Uses SysTick to delay <us> micro-seconds. 

volatile int PWM_Counter = 0;
volatile int pwm1=0, pwm2=0, pwm3=0, pwm4=0;

void delay(int dly)
{
	while( dly--);
}

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

void Delay_us(unsigned char us)
{
	// For SysTick info check the STM32L0xxx Cortex-M0 programming manual page 85.
	SysTick->LOAD = (F_CPU/(1000000L/us)) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

#define PIN_PERIOD (GPIOA->IDR&BIT8)

long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
	__disable_irq();
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter

	SysTick->LOAD = 0xffffff;  // 24-bit counter set to check for signal present
	SysTick->VAL = 0xffffff; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(SysTick->CTRL & BIT16) return 0;
	}
	SysTick->CTRL = 0x00; // Disable Systick counter
	
	SysTick->LOAD = 0xffffff;  // 24-bit counter reset
	SysTick->VAL = 0xffffff; // load the SysTick counter to initial value
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(SysTick->CTRL & BIT16) return 0;
		}
	}
	SysTick->CTRL = 0x00; // Disable Systick counter
	
	__enable_irq();
	
	return 0xffffff-SysTick->VAL;
}

// Interrupt service routines are the same as normal
// subroutines (or C funtions) in Cortex-M microcontrollers.
// The following should happen at a rate of 1kHz.
// The following function is associated with the TIM2 interrupt 
// via the interrupt vector table defined in startup.c
void TIM2_Handler(void) 
{
	TIM2->SR &= ~BIT0; // clear update interrupt flag
	PWM_Counter++;
	
	if(pwm1>PWM_Counter)
	{
		GPIOA->ODR |= BIT0;
	}
	else
	{
		GPIOA->ODR &= ~BIT0;
	}
	
	if(pwm2>PWM_Counter)
	{
		GPIOA->ODR |= BIT1;
	}
	else
	{
		GPIOA->ODR &= ~BIT1;
	}
	
	if(pwm3>PWM_Counter)
	{
		GPIOA->ODR |= BIT2;
	}
	else
	{
		GPIOA->ODR &= ~BIT2;
	}
	
	if(pwm4>PWM_Counter)
	{
		GPIOA->ODR |= BIT3;
	}
	else
	{
		GPIOA->ODR &= ~BIT3;
	}
	
	if (PWM_Counter > 2000) // THe period is 20ms
	{
		PWM_Counter=0;
		GPIOA->ODR |= (BIT11|BIT12);
	}   
}

void Hardware_Init(void)
{
	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

	RCC->IOPENR |= BIT0; // peripheral clock enable for port A

    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17);
	
	//servo pwm code
	// Set up output pins
	RCC->IOPENR |= BIT0; // peripheral clock enable for port A
    GPIOA->MODER = (GPIOA->MODER & ~(BIT0|BIT1)) | BIT0; // Make pin PA11 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT0; // Push-pull
    GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT2; // Make pin PA12 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT1; // Push-pull
	
	// Set up output pins
	RCC->IOPENR |= BIT0; // peripheral clock enable for port A
    GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT4; // Make pin PA11 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT2; // Push-pull
    GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT6; // Make pin PA12 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
	GPIOA->OTYPER &= ~BIT3; // Push-pull


	// Set up timer
	RCC->APB1ENR |= BIT0;  // turn on clock for timer2 (UM: page 177)
	TIM2->ARR = F_CPU/DEF_F-1;
	NVIC->ISER[0] |= BIT15; // enable timer 2 interrupts in the NVIC
	TIM2->CR1 |= BIT4;      // Downcounting    
	TIM2->CR1 |= BIT7;      // ARPE enable    
	TIM2->DIER |= BIT0;     // enable update event (reload event) interrupt 
	TIM2->CR1 |= BIT0;      // enable counting    
	
	__enable_irq();
	
		GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

	RCC->IOPENR |= BIT0; // peripheral clock enable for port A

    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17);
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

int main(void)
{
	long int count, countbase;
	char buff[80];
	int npwm;
    int cnt=0;
	int diff;
	float T = 0;
	float f = 0;
	int flag = 1;
	int vx = 0;
	int vy = 0;
	int basePWM = 1000;
	int slowPWM = 500;
	
	RCC->IOPENR |= 0x00000001; // peripheral clock enable for port A
	
	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17); 
	
	Hardware_Init();
	initUART2(9600);
	
	printf("\r\nGetting Base Count\r\n");
	waitms(1000); // Give putty some time to start.

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA 

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	SendATCommand("AT+DVID9331\r\n");
	
	printf("\r\nPress and hold a push-button attached to PA8 (pin 18) to transmit.\r\n");
	
	
	while(1)
	{
		/*if((GPIOA->IDR&BIT8)==0)
		{
			sprintf(buff, "JDY40 test %d\n", cnt++);
			eputs2(buff);
			eputc('.');
			waitms(200);
		}*/
		if(ReceivedBytes2()>0) // Something has arrived
		{
			egets2(buff, sizeof(buff));	
			printf("%s", buff);
			int extracted = sscanf(buff, "%d %d", &vy, &vx);
			if((strlen(buff)==7)){
				printf("%d %d\r\n", vy, vx);
			    if(vy == 16 && vx >= 32) {
			        // Right
			        printf("right\n\r");
			        pwm1 = pwm2 = basePWM;
			        pwm3 = pwm4 = 0;
			    } else if(vy == 16 && vx == 0) {
			        // Left
			        printf("left\n\r");
			        pwm1 = pwm2 = 0;
			        pwm3 = pwm4 = basePWM;
			    } else if(vy >= 32 && vx == 16) {
			        // Forward
			        printf("forward\n\r");
			        pwm2 = pwm3 = basePWM;
			        pwm1 = pwm4 = 0;
			    } else if(vy == 0 && vx == 16) {
			        // Backward
			        printf("backward\n\r");
			        pwm1 = pwm4 = basePWM;
			        pwm2 = pwm3 = 0;
			    } else if(vy >= 32 && vx >= 32) {
			        // Diagonal: Forward-Right
			        printf("diagonal forward-right\n\r");
			        pwm2 = basePWM;
			        pwm3 = basePWM * 0.5; // One wheel is moving slower
			        pwm1 = pwm4 = 0;
			    } else if(vy >= 32 && vx == 0) {
			        // Diagonal: Forward-Left
			        printf("diagonal forward-left\n\r");
			        pwm3 = basePWM;
			        pwm2 = basePWM * 0.5; // One wheel is moving slower
			        pwm1 = pwm4 = 0;
			    } else if(vy == 0 && vx >= 32) {
			        // Diagonal: Backward-Right
			        printf("diagonal backward-right\n\r");
			        pwm4 = basePWM;
			        pwm1 = basePWM * 0.5; // One wheel is moving slower
			        pwm2 = pwm3 = 0;
			    } else if(vy == 0 && vx == 0) {
			        // Diagonal: Backward-Left
			        printf("diagonal backward-left\n\r");
			        pwm1 = basePWM;
			        pwm4 = basePWM * 0.5; // One wheel is moving slower
			        pwm2 = pwm3 = 0;
			    } else if(vy == 16 && (vx>16&&vx<32)) {
			        // Right slow
			        printf("right slow\n\r");
			        pwm1 = pwm2 = basePWM;
			        pwm3 = pwm4 = 0;
			    } else if(vy == 16 && (vx<16&&vx>0)) {
			        // Left slow
			        printf("left slow\n\r");
			        pwm1 = pwm2 = 0;
			        pwm3 = pwm4 = slowPWM;
			    } else if((vy<=32&&vy>16) && vx == 16) {
			        // Forward slow
			        printf("forward slow\n\r");
			        pwm2 = pwm3 = slowPWM;
			        pwm1 = pwm4 = 0;
			    } else if((vy>0&&vy<16) && vx == 16) {
			        // Backward slow
			        printf("backward slow\n\r");
			        pwm1 = pwm4 = slowPWM;
			        pwm2 = pwm3 = 0;
			    } else if((vy>16&&vy<=32) && (vx>16&&vx<=32)) {
			        // Diagonal: Forward-Right slow
			        printf("diagonal forward-right slow\n\r");
			        pwm2 = slowPWM;
			        pwm3 = slowPWM * 0.5; // One wheel is moving slower
			        pwm1 = pwm4 = 0;
			    } else if((vy>16&&vy<=32) && (vx>0&&vx<16)) {
			        // Diagonal: Forward-Left slow
			        printf("diagonal forward-left slow\n\r");
			        pwm3 = slowPWM;
			        pwm2 = slowPWM * 0.5; // One wheel is moving slower
			        pwm1 = pwm4 = 0;
			    } else if((vy>0&&vy<16) && (vx>16&&vx<=32)) {
			        // Diagonal: Backward-Right slow
			        printf("diagonal backward-right slow\n\r");
			        pwm4 = slowPWM;
			        pwm1 = slowPWM * 0.5; // One wheel is moving slower
			        pwm2 = pwm3 = 0;
			    } else if((vy>0&&vy<16) && (vx>0&&vx<16)) {
			        // Diagonal: Backward-Left slow
			        printf("diagonal backward-left slow\n\r");
			        pwm1 = slowPWM;
			        pwm4 = slowPWM * 0.5; // One wheel is moving slower
			        pwm2 = pwm3 = 0;
			    } else {
			        // Stationary
			        printf("stationary\n\r");
			        pwm1 = pwm2 = pwm3 = pwm4 = 0;
			    }
	 		}
		}
		if(flag==1){
			countbase=GetPeriod(100);
			flag=0;
		}
		count=GetPeriod(100);
		if(count>0){
			T=count/(F_CPU*100.0); // Since we have the time of 100 periods, we need to divide by 100
			f=1.0/T;
		}
		
		diff = abs(count-countbase);
		printf("%05d\r\n", diff);
		sprintf(buff, "%07d\n", diff);
		waitms(150);
		eputs2(buff);
		//printf("f=%.2dHz, count=%d\r\n", (int)f, count);
		//waitms(150);
	//yellow=1 left forwards
	//white=1 left backwards
	//red=1 right forwards
	//brown=1 right backwards
	}
}