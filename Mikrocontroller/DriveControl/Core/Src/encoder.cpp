#include "encoder.h"
#include "constants.h"
#include "stm32f7xx_hal_tim.h"


Encoder::Encoder(TIM_HandleTypeDef* htim, GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB)
{
    this->_htim = htim;
    this->_portA = portA;
    this->_pinA = pinA;
    this->_portB = portB;
    this->_pinB = pinB;
}

void Encoder::init()
{
    // Set up the timer for the Encoder
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure Pin A
    GPIO_InitStruct.Pin = _pinA;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = ENCODER_M1_A_ALTERNATE;
    HAL_GPIO_Init(_portA, &GPIO_InitStruct);

    // Configure Pin B
    GPIO_InitStruct.Pin = _pinB;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = ENCODER_M1_B_ALTERNATE;
    HAL_GPIO_Init(_portB, &GPIO_InitStruct);

    HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL);
}

int32_t Encoder::get_count()
{
    return (int32_t) __HAL_TIM_GET_COUNTER(_htim);
}

void Encoder::reset_count()
{
    __HAL_TIM_SET_COUNTER(_htim, 0);
}

float Encoder::get_position()
{
    int32_t count = get_count();
    float position = (float)count / ENCODER_RESOLUTION;
    return position;
}