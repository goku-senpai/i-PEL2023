The pins used for USART communication depend on the specific hardware setup.
In the case of the STM32F7-Nucleo board, the GPIO pins used for USART3 and
USART6 are as follows:

USART3:
- TX: PC10
- RX: PC11

USART6:
- TX: PC6
- RX: PC7

The following code is used to set up the GPIO pins for USART3 and USART6.
This example code configures the pins to alternate function mode (AF), which is required for USART communication.

For USART3:

```
GPIO_InitTypeDef GPIO_InitStruct = {0};

// Configure USART3_TX Pin
GPIO_InitStruct.Pin = GPIO_PIN_10;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

// Configure USART3_RX Pin
GPIO_InitStruct.Pin = GPIO_PIN_11;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

For USART6:

```
GPIO_InitTypeDef GPIO_InitStruct = {0};

// Configure USART6_TX Pin
GPIO_InitStruct.Pin = GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

// Configure USART6_RX Pin
GPIO_InitStruct.Pin = GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

Make sure to include the necessary header files for

`GPIO_InitTypeDef` and `GPIO_PIN_x` definitions.

These GPIO initialization functions need to be called before
`MX_USART3_UART_Init()` and `MX_USART6_UART_Init()` so that the
pins are properly configured before the UART initialization takes place.


THE USART3 and USART6 initialization is:
