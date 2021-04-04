/*********************************************************************
*                    CUSTOM MMF031xxn Initialozation                 *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : system_MCU_INIT.c
Purpose : Initialize clock

*/

/** @addtogroup CMSIS
* @{
*/
#include "HAL_device.h"

/**
* @}
*/

/*!< IMPORTANT NOTE:
============================================================================== 
1. After each device reset the HSI is used as System clock source.

2. Please make sure that the selected System clock doesn't exceed your device's
maximum frequency.
*/

#define USE_HSE
//#define SYSCLK_FREQ_HZ 8000000
#define SYSCLK_FREQ_HZ 12000000
//#define SYSCLK_FREQ_HZ 24000000
//#define SYSCLK_FREQ_HZ 48000000
//#define SYSCLK_FREQ_HZ 72000000

#ifndef SYSCLK_FREQ_HZ
#define SYSCLK_FREQ_HZ 8000000
#endif /* SYSCLK_FREQ_HZ */

/**
 * @brief  Resets the RCC clock configuration to default state.
 * @param  None.
 * @retval None.
 */
static void
RCC_SetDefault()
{
    SET_BIT(RCC->CR, RCC_CR_HSION);
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_SW);

    /* RCC_CR_PLLDIV -> RCC_CR_PLLDN,  
       RCC_CR_PLLMUL -> RCC_CR_PLLDM */
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_PLLDN | RCC_CR_PLLDM);
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
    CLEAR_REG(RCC->CFGR);
    CLEAR_REG(RCC->CIR);
}

/**
* @brief  Setup the microcontroller system
*         Initialize the Embedded Flash Interface, the PLL and update the 
*         SystemCoreClock variable.
* @note   This function should be used only after reset.
* @param  None
* @retval None
*/
void SystemInit()
{
    __IO uint32_t StartUpCounter = 0;

    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    RCC_SetDefault();

    /* Configure the Flash Latency cycles and enable prefetch buffer */

    /* Enable Prefetch Buffer */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);

    /* Flash 0 wait state, bit0~2*/
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_LATENCY);
    /**
     * 000：零等待状态，当 0 < SYSCLK ≤ 24MHz
     * 001：一个等待状态，当 24MHz < SYSCLK ≤ 48MHz
     * 010：两个等待状态，当 48MHz < SYSCLK ≤ 72MHz
     */
    if (SYSCLK_FREQ_HZ <= 24000000) {
        SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_0);
    } else if (SYSCLK_FREQ_HZ > 24000000 && SYSCLK_FREQ_HZ <= 48000000) {
        SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_1);
    } else if (SYSCLK_FREQ_HZ > 48000000 && SYSCLK_FREQ_HZ <= 72000000) {
        SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_2);
    }

    /* Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers */

    /* HCLK = SYSCLK */
    SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);

    /* PCLK2 = HCLK */
    SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV1);

    /* PCLK1 = HCLK */
    /**
     * PPRE1：低速APB预分频(APB1) (APB low-speed prescaler (APB1))
     * 由软件置’1’或清’0’来控制低速APB1时钟(PCLK1)的预分频系数。
     * 注意：软件必须保证APB1时钟频率不超过36MHz。
     */
    if (SYSCLK_FREQ_HZ <= 36000000) {
        SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV1);
    } else if (SYSCLK_FREQ_HZ > 36000000 && SYSCLK_FREQ_HZ <= 72000000) {
        SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV2);
    } else if (SYSCLK_FREQ_HZ > 72000000 && SYSCLK_FREQ_HZ <= 96000000) {
        SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV4);
    }

#if defined(USE_HSE)

    /* Enable HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    /* Wait till HSE is ready and if Time out is reached exit */
    do {
        StartUpCounter++;
    } while ((READ_BIT(RCC->CR, RCC_CR_HSERDY) == 0) &&
             (StartUpCounter != HSE_STARTUP_TIMEOUT));

    //if (READ_BIT(RCC->CR, RCC_CR_HSERDY)) {
    //    /* Select HSE as system clock source */
    //    CLEAR_BIT(RCC->CFGR, RCC_CFGR_SW);
    //    SET_BIT(RCC->CFGR, RCC_CFGR_SW_HSE);

    //    /* Wait till HSE is used as system clock source */
    //    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) {}

    //} else {
    //    /* If HSE fails to start-up, the application will have wrong clock
    //     configuration. User can add here some code to deal with this error */
    //}

#endif /* USE_HSE */

    if (((SCB->CPUID & 0x00000070) == 0) &&
        ((uint32_t) * ((uint32_t *)(0x40013400)) == (0xCC4460B1U))) {
        while (1) {} //MCU is M0 q version, please check Core #define and Target MCU
    }
}

void SystemCoreClockUpdate()
{
}
