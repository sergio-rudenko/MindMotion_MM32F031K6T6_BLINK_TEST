/*********************************************************************
 *                    SEGGER Microcontroller GmbH                     *
 *                        The Embedded Experts                        *
 **********************************************************************

 -------------------------- END-OF-HEADER -----------------------------

 File    : main.c
 Purpose : Generic application start

 */

#include <stdio.h>
#include <stdlib.h>

#include "main.h"

/* private variables */
static __IO uint32_t TimingDelay;

/*********************************************************************
 *
 *       main()
 *
 *  Function description
 *   Application entry point.
 */
int main()
{
    printf("MCU started, SYSCLK = %d\n", SystemCoreClock);

    InitGPIO();
    InitDelay();

    BUZZER_ON();
    delay_ms(50);
    BUZZER_OFF();

    do {
        LED_TOGGLE();
        delay_ms(500);
    } while (1);
}

void InitGPIO()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN | BUZZER_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void InitDelay()
{
    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (1) {}
    }

    /* Configure the SysTick handler priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0);
}

void delay_ms(__IO uint32_t nTime)
{
    TimingDelay = nTime;

    while (TimingDelay != 0)
        ;
}

/* SYSTICK Interrupt Handler */
void SysTick_Handler(void)
{
    if (TimingDelay != 0x00) {
        TimingDelay--;
    }
}

/*************************** End of file ****************************/
