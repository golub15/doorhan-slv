#ifndef __BOARD
#define __BOARD

#include "main.h"

#define BEGIN_CRITICAL \
    __critical         \
    {
#define END_CRITICAL }

// Port B

#define LED_B_GPIO_PORT (GPIOB)
#define LED_B_GPIO_PIN (GPIO_PIN_4)

#define LED_A_GPIO_PORT (GPIOB)
#define LED_A_GPIO_PIN (GPIO_PIN_5)

#define DP1_GPIO_PORT (GPIOB)
#define DP1_GPIO_PIN (GPIO_PIN_6)

// Port C
#define DP3_GPIO_PORT (GPIOC)
#define DP3_GPIO_PIN (GPIO_PIN_1)

#define DP4_GPIO_PORT (GPIOC)
#define DP4_GPIO_PIN (GPIO_PIN_2)

#define START_GPIO_PORT (GPIOC)
#define START_GPIO_PIN (GPIO_PIN_3)

#define PED_GPIO_PORT (GPIOC)
#define PED_GPIO_PIN (GPIO_PIN_4)

#define PH_OP_GPIO_PORT (GPIOC)
#define PH_OP_GPIO_PIN (GPIO_PIN_5)

#define PH_CL_GPIO_PORT (GPIOC)
#define PH_CL_GPIO_PIN (GPIO_PIN_6)

#define STOP_GPIO_PORT (GPIOC)
#define STOP_GPIO_PIN (GPIO_PIN_7)

// Port F
#define BTN_GPIO_PORT (GPIOF)
#define BTN_GPIO_PIN (GPIO_PIN_4)

// Port D

#define SW_CL_GPIO_PORT (GPIOD)
#define SW_CL_GPIO_PIN (GPIO_PIN_0)

#define SW_OP_GPIO_PORT (GPIOD)
#define SW_OP_GPIO_PIN (GPIO_PIN_2)

#define INT_LAMP_GPIO_PORT (GPIOD)
#define INT_LAMP_GPIO_PIN (GPIO_PIN_3)

#define MOTOR_DIR_GPIO_PORT (GPIOD)
#define MOTOR_DIR_GPIO_PIN (GPIO_PIN_4)

#define MOTOR_PWR_GPIO_PORT (GPIOD)
#define MOTOR_PWR_GPIO_PIN (GPIO_PIN_5)

#define EXT_LAMP_GPIO_PORT (GPIOD)
#define EXT_LAMP_GPIO_PIN (GPIO_PIN_6)

#define LOW_VOLT_GPIO_PORT (GPIOD)
#define LOW_VOLT_GPIO_PIN (GPIO_PIN_7)

// PC3 - START
// PC4 - PED
// PC5 - PH_OP
// PC6 - PH_CL
// PC7 - STOP

// PD0 - SW.CL
// PD2 - SW.OP

// PC2 - DP4
// PC1 - DP3
// PE5 - DP2
// PB6 - DP1

// PB7 - RF_(ПУЛЬТЫ)
// PF4 - BTN

// PB0 - MOTOR_CURRENT
// PB1

// PB5 - LED_A
// PB4 - LED_B (ERROR)

// PD3 - INT_LAMP
// PD4 - MOTOR_REVERSE
// PD5 - MOTOR_POWER
// PD6 - EXT_LAMP
// PD7 - VOLTAGE_DETECTOR (DIGITAL)

#define TRIG_SET (0x01 << 0)
#define TRIG_RESET (0x01 << 1)
#define TRIG_DELAY_F (0x01 << 2)
#define TRIG_HOLD (0x01 << 3)
#define TRIG_DELAY (0x01 << 4)


#define TRIG_CLICK TRIG_RESET
#define TRIG_PRESSED TRIG_SET

#define DEBOUNCE_DELAY 50                   // ms
#define HOLD_TIME (20 * 2) // 2 seconds
#define OFF_DELAY 2500                      // ms

typedef struct SwitchControl
{
    uint8_t input_value; // Значение дискретного входа

    uint8_t state; // Готовое значение (с debounce)
    uint8_t flags; // Флаги сброса / установки

    uint32_t time; // Последнее время срабатывания

} SwitchControl_t;

typedef struct LedBlinkHandle
{
    uint8_t blink_times;
    uint8_t blink_times_loop;
    // uint8_t blink_phase;
    uint8_t state;
    uint32_t off_time;
    uint32_t on_time;
    uint32_t time;
} LedBlinkHandle_t;

void StartBlink(LedBlinkHandle_t *lb, uint8_t times_once, uint8_t times_loop);
void HandleLed(LedBlinkHandle_t *lb, uint8_t pin);

uint8_t InputTrig(SwitchControl_t *sc, uint8_t mode);
void UpdateSwitch(SwitchControl_t *sc);

unsigned long millis();
void delay(uint32_t time);
#endif