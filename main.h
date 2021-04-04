#include "HAL_conf.h"

#define LED_GPIO_PORT     GPIOA
#define LED_GPIO_PIN      GPIO_Pin_11
#define BUZZER_GPIO_PORT  GPIOA
#define BUZZER_GPIO_PIN   GPIO_Pin_12

#define DO_SET(g, p)      GPIO_SetBits(g,p)
#define DO_RESET(g, p)    GPIO_ResetBits(g,p)
#define DO_TOGGLE(g, p)   (GPIO_ReadOutputDataBit(g,p) ? GPIO_ResetBits(g,p) : GPIO_SetBits(g,p)) 

#define LED_ON()          DO_SET(LED_GPIO_PORT, LED_GPIO_PIN)
#define LED_OFF()         DO_RESET(LED_GPIO_PORT, LED_GPIO_PIN)
#define LED_TOGGLE()      DO_TOGGLE(LED_GPIO_PORT, LED_GPIO_PIN)

#define BUZZER_ON()       DO_SET(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN)
#define BUZZER_OFF()      DO_RESET(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN)

/* prototypes */
void InitGPIO();
void InitDelay();
void delay_ms(__IO uint32_t nTime);