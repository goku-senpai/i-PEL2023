/** @file define.h
 *
 * Copyright (c) 2020 IACE
 */
#ifndef DEFINE_H
#define DEFINE_H


/**
 * @defgroup EXPERIMENT experiment
 * main experiment defines
 *
 * @{
 */
/** control timer peripheral for car */
#define EXP_TIMER                           TIM7
/** timer interrupt for car */
#define EXP_TIMER_IRQ                       TIM7_IRQn
/** sampling time of roboto controller in ms */
#define EXP_DT                              (100) //TODO: check diffs TO 50 trim params!!!!
/** keepalive time of heartbeat in ms */
#define EXP_KEEPALIVE                       (500)
/** Experiment timer priority*/
#define EXP_TIMER_PRIO                      4,4

/** @} */

/**
 * @defgroup TICKSERVER TickServer
 * TickServer configuration
 *
 * @{
 */
/** maximum number of TickServer clients */
#define TICK_MAX_CLIENTS                1
/** @} */

/**
 * @defgroup TRANSPORT Transport
 * Transport defines
 *
 * @{
 */
/** pin of USART receive functionality */
#define TRANSPORT_RX_PIN                    GPIO_PIN_9
/** port of USART receive functionality */
#define TRANSPORT_RX_PORT                   GPIOG
/** alternate pin function for USART receive functionality */
#define TRANSPORT_RX_ALTERNATE              GPIO_AF8_USART6

/** pin of USART transmit functionality */
#define TRANSPORT_TX_PIN                    GPIO_PIN_14
/** port of USART transmit functionality */
#define TRANSPORT_TX_PORT                   GPIOG
/** alternate pin function for USART transmit functionality */
#define TRANSPORT_TX_ALTERNATE              GPIO_AF8_USART6

/** USART peripheral */
#define TRANSPORT_UART                      USART6
/** USART interrupt */
#define TRANSPORT_UART_INTERRUPT            USART6_IRQn

/** DMA peripheral for usart */
#define TRANSPORT_DMA                       DMA2_Stream2
/** DMA channel for usart */
#define TRANSPORT_DMA_CHANNEL               DMA_CHANNEL_5
/** DMA interrupt for usart */
#define TRANSPORT_DMA_INTERRUPT             DMA2_Stream2_IRQn

/** MIN dma priority*/
#define TRANSPORT_DMA_PRIO                  2,2
/** MIN uart priority*/
#define TRANSPORT_UART_PRIO                 2,2

/** USART baudrate */
#define TRANSPORT_BAUD                      115200

/** maximum payload per transport frame */
#define TRANSPORT_MAX_PAYLOAD               80U
#define TRANSPORT_MAX_FRAMES                (1<<4)
#define TRANSPORT_MAX_FRAME_DATA            (1<<10)
#define TRANSPORT_SERIAL_BUF                128U
#define TRANSPORT_MAX_WINDOW                80U

#define TRANSPORT_FRAME_LEN_DOUBLE          (40 / sizeof(double))

/** @} */


/**
 * @defgroup LED Led
 * Car indicator defines
 *
 * @{
 */
#define LED_BLUE_PIN                  GPIO_PIN_7
/** port of blue led */
#define LED_BLUE_PORT                 GPIOB

/** pin of red led */
#define LED_RED_PIN                   GPIO_PIN_14
/** port of red led */
#define LED_RED_PORT                  GPIOB

/** pin of green led */
#define LED_GREEN_PIN                 GPIO_PIN_0
/** port of green led */
#define LED_GREEN_PORT                GPIOB
/** @} */

/**
 * @defgroup PWM Pwm
 * Motor controller defines
 *
 * @{
 */
// ~ pwm freq of 18600 Hz at 84MHz periph clock
#define MOTOR_PWM_PRESC                 (0)
#define MOTOR_PWM_PERIOD                (4500 - 1)

#define MOTOR_PWM_TIM                   TIM3

#define MOTOR_PWM_M1_PIN                GPIO_PIN_9
#define MOTOR_PWM_M1_PORT               GPIOE
#define MOTOR_PWM_M1_ALTERNATE          GPIO_AF2_TIM3
#define MOTOR_PWM_M1_TIM_CHANNEL        TIM_CHANNEL_1

#define MOTOR_ENA1_PIN                  GPIO_PIN_13
#define MOTOR_ENA1_PORT                 GPIOF
#define MOTOR_ENB1_PIN                  GPIO_PIN_12
#define MOTOR_ENB1_PORT                 GPIOF



/** @} */

/**
 * @defgroup ENCODER Encoder
 * Encoder defines
 *
 * @{
 */

#define ENCODER_RESOLUTION              8384.0f //131(Übersetzung) *16 (Encoderauflösung) *4( 2*Rise and Fall flag)

#define ENCODER_M1_A_PIN                GPIO_PIN_12
#define ENCODER_M1_A_PORT               GPIOD
#define ENCODER_M1_A_ALTERNATE          GPIO_AF2_TIM4
#define ENCODER_M1_B_PIN                GPIO_PIN_13
#define ENCODER_M1_B_PORT               GPIOD
#define ENCODER_M1_B_ALTERNATE          GPIO_AF2_TIM4
#define ENCODER_M1_TIM                  TIM4

#define M_PI                            3.14159265358979323846
#define RAD2DEG                         57.295779513082
#define DEG2RAD                         0.017453292519943
#endif //DEFINE_H
