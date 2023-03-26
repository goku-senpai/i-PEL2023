#include <sstream>
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

/**
 * timers:
 * tim3 MOTORPWM
 * tim4 ENCODER
 * tim6 Datacomm?
 *
 * USART6 RX/TX (? doublecheck)
 *
 */



//instance of USART
UART_HandleTypeDef huart6;
UART_HandleTypeDef huart3;

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
                                POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL,
                                DEFAULT_TARGET_START, GPIO_PIN_8, 0)
                                /*
                                 *  float pos_kp, float pos_ki, float pos_kd, float max_output, float max_integral,
                                 *  float target_start, uint32_t pin_direction, bool is_position_controller)
                                 */
                                ;

// Encoder instance
Encoder encoder(&timINIT.htim4, ENCODER_M1_A_PORT, ENCODER_M1_A_PIN, ENCODER_M1_A_ALTERNATE);

// PID controller instance
PIDController pid_controller(POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL, DEFAULT_TARGET_START);


// Pseudofunctios
static void MX_USART6_UART_Init(void);
void delay(uint32_t ms);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


// constants for PID_controller and buffer
float dKp = 0.0f, dKi = 0.0f, dKd = 0.0f, dSetpoint = 0.0f, dKpold=0.0f,  dKiold=0.0f, dKdold=0.0f, err=0.0f;

bool bFreewheel = true;
int iEncCount;


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
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    LedRed = new DIO(LED_RED_PIN, LED_RED_PORT);
    LedGreen = new DIO(LED_GREEN_PIN, LED_GREEN_PORT);
    LedBlue = new DIO(LED_BLUE_PIN, LED_BLUE_PORT);
    //encoder.reset_count();
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

void exception() {
    int i = -1;
    std::stringstream errMsg;
    errMsg << "NO ENCODER found, continuing without";
    try {
        HAL_TIM_Encoder_Start(&timINIT.htim4, TIM_CHANNEL_6); //CHANNEL_ALL}
    }
    catch(int i) {
        printf("NO ENCODER found, continuing without");
        return;
    }
}

void testmessage() {
    uint8_t rx_buff[20];
    HAL_StatusTypeDef statu6 = HAL_UART_Receive(&huart6, rx_buff, 20, 300);
    if (statu6 == HAL_OK) {
        // Data received successfully
        printf("Received data from COM3(UART6)\r\n");
    } else {
        // Error receiving data
        printf("Error receiving data from COM3(UART6)\r\n");
    }
    HAL_StatusTypeDef statu3 = HAL_UART_Receive(&huart3, rx_buff, 20, 300);
    if (statu3 == HAL_OK) {
        // Data received successfully
        printf("Received data from COM3(UART3)\r\n");
    } else {
        // Error receiving data
        printf("Error receiving data from COM3(UART3)\r\n");
    }
}


int main() {
    uint8_t rx_buffer[iRxBuffSize];
    uint8_t tx_buffer[iTxBuffSize];
    setup();
    MX_USART6_UART_Init();
    LedGreen->toggle();

    testmessage();

    while (1) {
        LedBlue->toggle();
        /**
         * Read Data from USB (GUI)
         **/
        HAL_UART_Receive(&huart3, rx_buffer,20, 300);

        /**
         * Buffer it and split it into b,f,f,f,f
         **/
        //Buffer: bool ctl_mode, float Kp, float, Ki, float Kd, float Setpoint;
        checkmsg(rx_buffer);
        LedBlue->toggle();
        //LedBlue->toggle();
        if (dKp!=dKpold || dKi != dKiold || dKd!=dKdold){
            // Set new PID gains with gains from recieved if they got changed
            MotorController.pid_controller_.set_gains(dKp, dKi, dKd);
            dKp = dKpold;
            dKi = dKiold;
            dKd = dKdold;
        }
            //check if v ctl or pos ctl
        if (bFreewheel=1){
            pid_controller.set_mode(PIDController::SPEED_CONTROL);

            // Set new setpoint
            pid_controller.set_target(dSetpoint);

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
            pid_controller.set_target(dSetpoint);

            // Format the data as a string and add it to the buffer
            int num_chars = snprintf((char*)rx_buffer, iRxBuffSize, "%d,%.2f,%.2f,%.2f,%.2f", bFreewheel, dKp, dKi, dKd, dSetpoint);

            // Check if the number of characters added to the buffer is within its size limit
            if (num_chars >= iRxBuffSize) {
                printf("WARNING: Buffer size overfull!");
            }
        }
        else LedRed->toggle();


// Update motor controller
    /**
     * WHAT ENCODER DO WE USE?
     *          then, just call interrupts on the data streams (for data tx, rx and encoder)
     */
        // timer für iinterrupt, sodass update nur gekallt wird, wenn
        // encoder 1 oder 2 durch interruot, schauen in nächsten was passiert und updaten!!
        // Timer Rising edge event call: wenn rising edge: flanke steigt
        //
        iEncCount=encoder.get_count();
        printf("%.i",iEncCount);

        // TODO: check if update works
        MotorController.update(0.01,iEncCount);
        //
        // Better not a fixed tike delay, but work with interrupts
        delay(10);;

        /**
         * Send Data to USB (GUI)
         *
         * ToDo:
         *  Data to send: time, error, pos or speed (current)
         *  add time
         *  add posi now or speed now
         **/

        //gets the actual error at the moment
        error=pid_controller.get_target();
        uint8_t sen_chars = snprintf((char*)tx_buffer, iTxBuffSize, "%.d,%.2f", bFreewheel,error);
        if(sen_chars < 0){
            LedRed->toggle();
            break;
        } else {
            HAL_UART_Transmit_IT(&huart3, tx_buffer, sizeof(sen_chars));
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
    LedGreen->set(false);
    LedBlue->set(false);
    while (1) {
        LedRed->set(true);
        delay(60);
        LedRed->set(false);
        delay(30);
        LedRed->set(true);
        delay(70);
        LedRed->set(false);
        delay(530);

    }
    /* USER CODE END Error_Handler_Debug */
}


/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {
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