#include <sstream>
#include <cstring>
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;


TIM_HandleTypeDef htim2;


//instance of timers
Timer_initialize timINIT;

// USB Serial connection to the PC
Serial pc(USART1, GPIOA, GPIO_PIN_9, GPIO_PIN_10);

float error;

void TIM4_Config(void);

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    if(hpcd->Instance == USB_OTG_FS) {
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
Encoder encoder(&timINIT.htim4, ENCODER_M1_A_PORT, ENCODER_M1_A_PIN, ENCODER_M1_B_PORT, ENCODER_M1_B_PIN);

// PID controller instance
PIDController pid_controller(POS_KP, POS_KI, POS_KD, MAX_OUT, DEFAULT_MAX_INTEGRAL, DEFAULT_TARGET_START);


// Prototypefunctios
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);


void delay(uint32_t ms);


// constants for PID_controller and buffer
float dKp = 0.0f, dKi = 0.0f, dKd = 0.0f, dSetpoint = 0.0f, dKpold=0.0f,  dKiold=0.0f, dKdold=0.0f, err=0.0f;
bool bFreewheel = true;
int iEncCount;
float fEncpos;

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
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

// Configure USART1_TX Pin (PA9)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// Configure USART1_RX Pin (PA10)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// Configure LEDS
    LedRed = new DIO(LED_RED_PIN, LED_RED_PORT);
    LedGreen = new DIO(LED_GREEN_PIN, LED_GREEN_PORT);
    LedBlue = new DIO(LED_BLUE_PIN, LED_BLUE_PORT);

    //encoder.reset_count();
}


void USART2_IRQHandler() {
    HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // handle incoming data
}

void checkmsg(const uint8_t* data) {
    const char *c_data = (const char*)data;
    // Parse the data into variables
    parseMessage(data,bFreewheel,dKp,dKi,dKd,dSetpoint);
    printf("Received data: %d,%f,%f,%f,%f\n", bFreewheel, dKp, dKi,dKd, dSetpoint);
}

//size of buffer
const int iRxBuffSize = 1000;
int iTxBuffSize = iRxBuffSize;

void exception() {
    int i = -1;
    std::stringstream errMsg;
    errMsg << "NO ENCODER found, continuing without";
    try {

        //check for timer channel for original was: TIM_CHANNEL_ALL
        HAL_TIM_Encoder_Start(&timINIT.htim4, TIM_CHANNEL_ALL); //CHANNEL_ALL}
    }
    catch(int i) {
        printf("NO ENCODER found, continuing without");
        return;
    }
}

void testmessage() {
    uint8_t rx_buff[iRxBuffSize];
    HAL_StatusTypeDef statu6 = HAL_UART_Receive(&huart1, rx_buff, iRxBuffSize, 300);
    if (statu6 == HAL_OK) {
        // Data received successfully
        printf("Received data from COM3(UART6)\r\n");
    } else {
        // Error receiving data
        printf("Error receiving data from COM3(UART6)\r\n");
    }
    HAL_StatusTypeDef statu3 = HAL_UART_Receive(&huart1, rx_buff, iRxBuffSize, 300);
    if (statu3 == HAL_OK) {
        // Data received successfully
        printf("Received data from COM3(UART3)\r\n");
    } else {
        // Error receiving data
        printf("Error receiving data from COM3(UART3)\r\n");
    }
    HAL_StatusTypeDef statu2 = HAL_UART_Receive(&huart2, rx_buff, iRxBuffSize, 300);
    if (statu2 == HAL_OK) {
        // Data received successfully
        printf("Received data from COM2(UART2)\r\n");
    } else {
        // Error receiving data
        printf("Error receiving data from COM3(UART3)\r\n");
    }
}

int main() {
    HAL_Init();

    //define txrx buffer
    uint8_t rx_buffer1[iRxBuffSize];
    uint8_t rx_buffer2[iRxBuffSize];
    uint8_t rx_buffer3[iRxBuffSize];

    uint8_t tx_buffer[iTxBuffSize];
    setup();

    //initialize periphery
    MX_USART6_UART_Init();
    MX_USART3_UART_Init();
    MX_USART2_UART_Init();

    //enable the interrupts:
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    LedGreen->toggle();
    encoder.reset_count();

    testmessage();
    /*
    std::string s = pc.readStringUntil('\n'); // Use newline as the terminator
    std::strcpy((char*)rx_buffer, s.c_str());
    */
    while (1) {
        LedBlue->toggle();
        /**
         * Read Data from USB (GUI)
         **/
        HAL_UART_Receive(&huart1, rx_buffer1, iRxBuffSize, 400);
        HAL_UART_Receive(&huart2, rx_buffer2, iRxBuffSize, 400);
        HAL_UART_Receive(&huart1, rx_buffer3, iRxBuffSize, 400);

        /**
         * Buffer it and split it into b,f,f,f,f
         **/
        //Buffer: bool ctl_mode, float Kp, float, Ki, float Kd, float Setpoint;

        checkmsg(rx_buffer1);
        checkmsg(rx_buffer2);
        checkmsg(rx_buffer3);
        LedBlue->toggle();

        if (dKp!=dKpold || dKi != dKiold || dKd!=dKdold){
    // Set new PID gains with gains from recieved if they got changed
            //send control data to pid controller
            MotorController.pid_controller_.set_gains(dKp, dKi, dKd);
            dKp = dKpold;
            dKi = dKiold;
            dKd = dKdold;
        }
            //check if v ctl or pos ctl
        if (bFreewheel){
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
            int num_chars = snprintf((char*)rx_buffer1, iRxBuffSize, "%d,%.2f,%.2f,%.2f,%.2f", bFreewheel, dKp, dKi, dKd, dSetpoint);

            // Check if the number of characters added to the buffer is within its size limit
            if (num_chars >= iRxBuffSize) {
                printf("WARNING: Buffer size overfull!");
                int i =0;
                while ( i < 4) {
                    LedBlue->toggle();
                    LedGreen->set(false);
                    delay(500);
                    i++;
                }
                LedGreen->set(true);
            }
        }

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
        fEncpos = encoder.get_position();
        //set motor enable Pin to high

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
        uint8_t sen_chars = snprintf((char*)tx_buffer, iTxBuffSize, "%.d,%.2f,%.2f", bFreewheel,fEncpos,error);
        if(sen_chars < 0){
            LedRed->toggle();
        }
        else {
            HAL_UART_Transmit_IT(&huart1, tx_buffer, sizeof(sen_chars));
        }
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

/* USER CODE BEGIN USART3 */
void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN USART3 */
void MX_USART3_UART_Init(void)
{
    huart1.Instance = USART3;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}


void TIM4_Config(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};

    htim2.Instance = TIM4;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_USART6_UART_Init(void)
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }
}


void delay(uint32_t ms) {
    HAL_Delay(ms);
}