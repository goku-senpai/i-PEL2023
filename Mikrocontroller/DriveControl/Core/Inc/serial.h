#ifndef SERIAL_H
#define SERIAL_H

#include <string>

class Serial {
public:
    Serial(USART_TypeDef *uart, GPIO_TypeDef *tx_port, uint16_t tx_pin, uint16_t rx_pin);
    void init(uint32_t baud_rate);
    bool available();
    char read();
    std::string readStringUntil(char terminator);

private:
    UART_HandleTypeDef uart_; // Change the type of uart_ to UART_HandleTypeDef
    GPIO_TypeDef *tx_port_;
    uint16_t tx_pin_;
    uint16_t rx_pin_;
};

Serial::Serial(USART_TypeDef *uart, GPIO_TypeDef *tx_port, uint16_t tx_pin, uint16_t rx_pin) :
        tx_port_(tx_port), tx_pin_(tx_pin), rx_pin_(rx_pin) {
    // Create a new UART_HandleTypeDef struct
    UART_HandleTypeDef huart;
    huart.Instance = uart;
    uart_ = huart; // Assign it to the uart_ member variable
}

void Serial::init(uint32_t baud_rate) {
    // Configure UART pins
    GPIO_InitTypeDef gpio_init;
    gpio_init.Pin = tx_pin_;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(tx_port_, &gpio_init);

    gpio_init.Pin = rx_pin_;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(tx_port_, &gpio_init);

    // Configure UART peripheral
    __HAL_RCC_USART2_CLK_ENABLE();

    uart_.Init.BaudRate = baud_rate;
    uart_.Init.WordLength = UART_WORDLENGTH_8B;
    uart_.Init.StopBits = UART_STOPBITS_1;
    uart_.Init.Parity = UART_PARITY_NONE;
    uart_.Init.Mode = UART_MODE_TX_RX;
    uart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&uart_);
}

bool Serial::available() {
    return HAL_UART_GetState(&uart_) == HAL_UART_STATE_READY;
}

char Serial::read() {
    char c;
    HAL_UART_Receive(&uart_, (uint8_t*)&c, 1, HAL_MAX_DELAY);
    return c;
}

std::string Serial::readStringUntil(char terminator) {
    std::string s = "";
    char c;
    while (available()) {
        HAL_UART_Receive(&uart_, (uint8_t*)&c, 1, HAL_MAX_DELAY);
        if (c == terminator) {
            break;
        }
        s += c;
    }
    return s;
}


#endif
