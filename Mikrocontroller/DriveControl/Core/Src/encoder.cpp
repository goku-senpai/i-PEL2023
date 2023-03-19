#include "encoder.h"

Encoder::Encoder(TIM_TypeDef* htim, GPIO_TypeDef* port, uint16_t pin, uint32_t alternate)
{
    TIM_HandleTypeDef htim_handle;
    htim_handle.Instance = htim;

}

void Encoder::init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = _pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = _alternate;
    HAL_GPIO_Init(_port, &GPIO_InitStruct);

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
