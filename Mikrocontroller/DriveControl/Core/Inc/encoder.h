#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f7xx_hal.h"
#include "constants.h"
#include "stm32f7xx_hal_tim.h"

class Encoder
{
public:
    Encoder(TIM_TypeDef* htim, GPIO_TypeDef* port, uint16_t pin, uint32_t alternate);
    void init();
    int32_t get_count();
    void reset_count();
    float get_position();

private:
    TIM_HandleTypeDef* _htim;
    GPIO_TypeDef* _port;
    uint16_t _pin;
    uint32_t _alternate;
};

#endif
