/** @file gpio.h
 *
 * Copyright (c) 2020 IACE
 */
#ifndef STM_GPIO_H
#define STM_GPIO_H

#include "stm/hal.h"
#include "stm32f7xx_hal.h"

/**
 * Wrapper class for simple digital input/output pin
 */
class DIO {
private:
    uint32_t iPin;
    GPIO_TypeDef *port;
public:
    /**
     * set digital output level
     * @param high true/false
     */
    void set(bool high) {
        HAL_GPIO_WritePin(port, iPin, (GPIO_PinState) high);
    }

    /**
     * read digital input level
     * @return high/low
     */
    bool get() {
        return HAL_GPIO_ReadPin(port, iPin);
    }

    /**
     * toggle digital output level
     */
    void toggle() {
        HAL_GPIO_TogglePin(port, iPin);
    }

    DIO(uint32_t pin, GPIO_TypeDef *port,
        uint32_t mode = GPIO_MODE_OUTPUT_PP,
        uint32_t pull = GPIO_NOPULL) {
        this->iPin = pin;
        this->port = port;
        GPIO_InitTypeDef GPIO_InitStruct = {};
        GPIO_InitStruct.Pin = iPin;
        GPIO_InitStruct.Mode = mode;
        GPIO_InitStruct.Pull = pull;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(port, &GPIO_InitStruct);
    }
};

#endif //STM_GPIO_H
