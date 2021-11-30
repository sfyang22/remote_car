#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    LED_CLK_ENABLE();

    GPIO_InitStruture.Pin = LED_PIN;
    GPIO_InitStruture.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruture.Pull = GPIO_NOPULL;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruture);

    LED_OFF();
}

void CAR_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    GPIO_InitStruture.Pin = CAR_LED_PIN;
    GPIO_InitStruture.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruture.Pull = GPIO_NOPULL;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(CAR_LED_PORT, &GPIO_InitStruture);
}

