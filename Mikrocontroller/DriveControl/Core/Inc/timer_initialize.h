//
// Created by Palan on 17/03/2023.
//

#ifndef ANPAMA_TIMER_INITIALIZE_H
#define ANPAMA_TIMER_INITIALIZE_H


#include "stm32f7xx.h"
#include "stm32f7xx_hal_tim.h"

class Timer_initialize {

public:
    TIM_HandleTypeDef htim3;
    TIM_HandleTypeDef htim4;

    Timer_initialize();
    void init();

};


#endif //ANPAMA_TIMER_INITIALIZE_H
