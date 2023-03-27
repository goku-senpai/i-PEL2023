#ifndef CONSTANTS_H
#define CONSTANTS_H

/**
 * @defgroup Motor setup
 * @{
 */
#define MOTOR_PWM_FREQ 20000
#define MOTOR_MAX_PWM_DUTY 1000

#define motortimA                       tim3_ch_1
#define motortimB                       tim3_ch_2

#define MOTOR_ENA1_PIN                  GPIO_PIN_6
#define MOTOR_ENA1_PORT                 GPIOB
#define MOTOR_ENB1_PIN                  GPIO_PIN_0
#define MOTOR_ENB1_PORT                 GPIOB


/** @} */
/**
 * @defgroup ENCODER Encoder
 * @{
 */

#define ENCODER_RESOLUTION              8384.0f //131(Übersetzung) *16 (Encoderauflösung) *4( 2*Rise and Fall flag)
#define SPEED_KF                        0.1f

/** Encoder Motor 1 */
#define ENCODER_M1_A_PIN                GPIO_PIN_0
#define ENCODER_M1_A_PORT               GPIOB
#define ENCODER_M1_A_ALTERNATE          GPIO_AF2_TIM4

#define ENCODER_M1_B_PIN                GPIO_PIN_1
#define ENCODER_M1_B_PORT               GPIOB
#define ENCODER_M1_B_ALTERNATE          GPIO_AF2_TIM4

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


/** sampling time of roboto controller in ms */
#define EXP_DT                              (100) //TODO: check diffs TO 50 trim params!!!!


/**
 * @defgroup PID_MODE
 * Defines if position or speed control
 *
 * @{
 */
/** const of pos control */
#define PID_MODE_POSITION_CONTROL       0
/** const of speed control */
#define PID_MODE_SPEED_CONTROL          1


#define M_PI                            3.14159265358979323846
#define RAD2DEG                         57.295779513082
#define DEG2RAD                         0.017453292519943


#endif