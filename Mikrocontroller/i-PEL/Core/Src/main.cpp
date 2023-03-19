#include "gpio.h"
#include "motor_controller.h"
#include "pid_controller.h"
#include "encoder.h"
#include "serial.h"
#include "timer_initialize.h"
#include "constants.h"
#include "stm32f767xx.h"
#include "stm32f7xx_hal.h"


////instance of LEDS
DIO *LedRed;
DIO *LedGreen;
DIO *LedBlue;

//instance of USART
UART_HandleTypeDef huart6;

//instance of timers
Timer_initialize timer_initialize;

// USB Serial connection to the PC
Serial pc(USART2, GPIOA, GPIO_PIN_2, GPIO_PIN_3);

// Motor controller instance
MotorController motor_controller(&timer_initialize.htim3, TIM_CHANNEL_1, GPIOB, &timer_initialize.htim4, GPIO_PIN_6, GPIO_PIN_7,
                                 POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL,
                                 DEFAULT_TARGET_START, GPIO_PIN_8);


// Encoder instance
Encoder encoder(TIM4, ENCODER_M1_A_PORT, ENCODER_M1_A_PIN, ENCODER_M1_A_ALTERNATE);

// PID controller instance
PIDController pid_controller(POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL, DEFAULT_TARGET_START);


static void MX_USART6_UART_Init(void);


void delay(uint32_t ms);



void parse_msg(const uint8_t* data, bool& bFreewheel, float& dKp, float& dKi, float& dKd, float& dSetpoint) {
    const char* c_data = reinterpret_cast<const char*>(data);
    LedRed->toggle();

    char* endptr;
    bFreewheel = strtol(c_data, &endptr, 10) != 0;
    dKp = strtof(endptr, &endptr);
    dKi = strtof(endptr, &endptr);
    dKd = strtof(endptr, &endptr);
    dSetpoint = strtof(endptr, nullptr);
    LedRed->toggle();

}

void setup() {
    //timer_initialize.init();

    HAL_Init();

    // PC Serial Configuration
    //pc.init(115200);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_DMA2_CLK_ENABLE();

    __HAL_RCC_USART6_CLK_ENABLE();


    LedRed = new DIO(LED_RED_PIN, LED_RED_PORT);
    LedGreen = new DIO(LED_GREEN_PIN, LED_GREEN_PORT);
    LedBlue = new DIO(LED_BLUE_PIN, LED_BLUE_PORT);

}

void checkmsg(const uint8_t* data) {
    const char *c_data = (const char*)data;

    // Parse the data into variables
    bool bFreewheel = true;
    float dKp = 0.0f, dKi = 0.0f, dKd = 0.0f, dSetpoint = 0.0f;

    parse_msg(data,bFreewheel,dKp,dKi,dKd,dSetpoint);
    printf("Received data: %d,%f,%f,%f,%f\n", bFreewheel, dKp, dKi,dKd, dSetpoint);

}


int main() {
    //size of buffer
    uint8_t rx_buffer[100];
    //    LedBlue->toggle();
    setup();

    printf("Starting motor controller...\n");

    //uint8_t test_msg[]="11.32";
    uint8_t test_msg[]="1 1.32 10.01 100.01 10.001\n";
    uint8_t buffer[]="\n";
    LedBlue->toggle();

    while (1) {

        checkmsg(test_msg);
        /*
         * Read Data from USB (GUI)
         *  HAL_UART_Receive(&huart6, rx_buffer,20, 300);
         *  checkmsg(rx_buffer);
         */

        HAL_UART_Receive(&huart6, rx_buffer,20, 300);
        checkmsg(rx_buffer);
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);

        //LedGreen->toggle();
        delay(250);

    }
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}


/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {
    /* USER CODE BEGIN USART6_Init 0 */

    /* USER CODE END USART6_Init 0 */

    /* USER CODE BEGIN USART6_Init 1 */

    /* USER CODE END USART6_Init 1 */
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
}
    /* USER CODE BEGIN USART6_Init 2 */

void delay(uint32_t ms) {
    HAL_Delay(ms);
}