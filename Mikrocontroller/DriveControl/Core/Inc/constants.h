#ifndef CONSTANTS_H
#define CONSTANTS_H

/**
 * @defgroup Motor setuo
 * Car indicator defines
 *
 * @{
 */
#define MOTOR_PWM_FREQ 20000
#define MOTOR_MAX_PWM_DUTY 1000


#define MOTOR_ENA1_PIN                  GPIO_PIN_13
#define MOTOR_ENA1_PORT                 GPIOF

// PID Controller constants
#define POS_KP 1
#define POS_KI 0
#define POS_KD 0

#define MAX_OUT 0.1
#define DEFAULT_TARGET_START 0
#define DEFAULT_MAX_INTEGRAL 0
#define MAX_OUTPUT 100


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


/** @} */

/**
 * @defgroup ENCODER Encoder
 * Encoder defines
 *
 * @{
 */

#define ENCODER_RESOLUTION              8384.0f //131(Übersetzung) *16 (Encoderauflösung) *4( 2*Rise and Fall flag)
#define SPEED_KF                        100


#define ENCODER_M1_A_PIN                GPIO_PIN_12
#define ENCODER_M1_A_PORT               GPIOD
#define ENCODER_M1_A_ALTERNATE          GPIO_AF2_TIM4

#define ENCODER_M1_B_PIN                GPIO_PIN_13
#define ENCODER_M1_B_PORT               GPIOE

#define ENCODER_M1_B_ALTERNATE          GPIO_AF2_TIM4


#define PID_MODE_POSITION_CONTROL       0
#define PID_MODE_SPEED_CONTROL          1
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

#define M_PI                            3.14159265358979323846
#define RAD2DEG                         57.295779513082
#define DEG2RAD                         0.017453292519943

#endif