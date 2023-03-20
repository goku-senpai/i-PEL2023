#include "gpio.h"
#include "motor_controller.h"
#include "pid_controller.h"
#include "encoder.h"
#include "serial.h"
#include "timer_initialize.h"
#include "constants.h"
#include "stm32f767xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_pcd.h"


////instance of LEDS
DIO *LedRed;
DIO *LedGreen;
DIO *LedBlue;

//instance of USART
UART_HandleTypeDef huart6;

//instance of timers
Timer_initialize timINIT;

// USB Serial connection to the PC
Serial pc(USART2, GPIOA, GPIO_PIN_2, GPIO_PIN_3);

float error;

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(hpcd->Instance == USB_OTG_FS) {
        /* Configure USB FS GPIOs */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* Configure DM and DP pins */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Enable USB FS clock */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

        /* Set USB FS interrupt priority */
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);

        /* Enable USB FS interrupt */
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    }
    else if(hpcd->Instance == USB_OTG_HS) {
        /* Configure USB HS GPIOs */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* Configure DM and DP pins */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* Enable USB HS clock */
        __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

        /* Set USB HS interrupt priority */
        HAL_NVIC_SetPriority(OTG_HS_IRQn, 6, 0);

        /* Enable USB HS interrupt */
        HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
    }
}


// Motor controller instance
MotorController MotorController(&timINIT.htim3, TIM_CHANNEL_1, GPIOB,
                                &timINIT.htim4, GPIO_PIN_6, GPIO_PIN_7,
                                POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL,
                                DEFAULT_TARGET_START, GPIO_PIN_8, 0)
                                ;


// Encoder instance
Encoder encoder(TIM4, ENCODER_M1_A_PORT, ENCODER_M1_A_PIN, ENCODER_M1_A_ALTERNATE);

// PID controller instance
PIDController pid_controller(POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL, DEFAULT_TARGET_START);


// Pseudofunctios
static void MX_USART6_UART_Init(void);
void delay(uint32_t ms);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


// constants for PID_controller and buffer
float dKp = 0.0f, dKi = 0.0f, dKd = 0.0f, dSetpoint = 0.0f, err=0.0f;
bool bFreewheel = true;


void parseMessage(const uint8_t* data, bool& bFreewheel, float& dKp, float& dKi, float& dKd, float& dSetpoint) {
    const char* c_data = reinterpret_cast<const char*>(data);
    char* endptr;
    bFreewheel = strtol(c_data, &endptr, 10) != 0;
    dKp = strtof(endptr, &endptr);
    dKi = strtof(endptr, &endptr);
    dKd = strtof(endptr, &endptr);
    dSetpoint = strtof(endptr, nullptr);
}


void setup() {
    HAL_Init();
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
    encoder.reset_count();

}

void checkmsg(const uint8_t* data) {
    const char *c_data = (const char*)data;

    // Parse the data into variables
    parseMessage(data,bFreewheel,dKp,dKi,dKd,dSetpoint);
    printf("Received data: %d,%f,%f,%f,%f\n", bFreewheel, dKp, dKi,dKd, dSetpoint);

}


//size of buffer
int iRxBuffSize = 1000;
int iTxBuffSize = 1000;

int main() {
    uint8_t rx_buffer[iRxBuffSize];
    uint8_t tx_buffer[iTxBuffSize];
    //    LedBlue->toggle();
    setup();
    while (1) {
        /**
         * Read Data from USB (GUI)
         **/
        HAL_UART_Receive(&huart6, rx_buffer,20, 300);

        /**
         * Buffer it and split it into b,f,f,f,f
         **/
        //Buffer: bool ctl_mode, float Kp, float, Ki, float Kd, float Setpoint;
        checkmsg(rx_buffer);
        LedBlue->toggle();

        // Set new PID gains with gains from recieved
        MotorController.pid_controller_.set_gains(dKp, dKi, dKd);
        //check if v ctl or pos ctl
        if (bFreewheel=1){
            pid_controller.set_mode(PIDController::SPEED_CONTROL);

            // Set new setpoint
            MotorController.set_target(dSetpoint);

            // Format the data as a string and add it to the buffer
            int num_chars = snprintf((char*)tx_buffer, iRxBuffSize, "%d,%.2f,%.2f,%.2f,%.2f", bFreewheel, dKp, dKi, dKd, dSetpoint);

            // Check if the number of characters added to the buffer is within its size limit
            if (num_chars >= iRxBuffSize) {
                // TODO:Handle the case where the buffer is not big enough
            }
        }
        else if(bFreewheel=0) {
            pid_controller.set_mode(PIDController::POSITION_CONTROL);

            // Set new setpoint
            MotorController.set_target(dSetpoint);

            // Format the data as a string and add it to the buffer
            int num_chars = snprintf((char*)rx_buffer, iRxBuffSize, "%d,%.2f,%.2f,%.2f,%.2f", bFreewheel, dKp, dKi, dKd, dSetpoint);

            // Check if the number of characters added to the buffer is within its size limit
            if (num_chars >= iRxBuffSize) {
                // TODO:Handle the case where the buffer is not big enough
            }
        }
        else LedRed->toggle();

//////////////////////////////////////////////////////////////////////////////
/**
  *      FAIL FROM HERE
**/       // Update motor controller
///        MotorController.update(0.01);                                  ///
        //print out something on IO stream to check and find out where the Problem is with
        //          th update fun.
        delay(50);;

        /**
         * Send Data to USB (GUI)
         *
         * ToDo:
         *  Data to send: time, error, pos or speed (current)
         *  add time
         *  add posi now or speed now
         **/
        error=MotorController.get_error();
        uint8_t sen_chars = snprintf((char*)tx_buffer, iTxBuffSize, "%.d,%.2f", bFreewheel,error);
        if(sen_chars < 0){
            LedRed->toggle();
            break;
        } else {
            HAL_UART_Transmit_IT(&huart6, tx_buffer, sizeof(sen_chars));
        }
             //err=MotorController.get_error(); //= error of PID
        LedBlue->toggle();

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