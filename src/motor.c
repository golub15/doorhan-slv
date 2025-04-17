#include <motor.h>

extern MotorState_t motor_state;
extern DevConfig_t device_config;

// For current calculation
volatile uint16_t sine_arr[SINE_ARR_SIZE];
volatile uint8_t sine_finished = 0;

static uint32_t lv_timer = 0;

float calc, result;
uint32_t current_result, current_previous;

// static float RMS_OLD_Value = 0;
// static float P_k1_k1;

uint32_t cur_arr[5];
uint8_t cur_arr_counter = 0;

// static float Q = 0.01; // Q: Regulation noise, Q increases, dynamic response becomes faster, and convergence stability becomes worse
// static float R = 0.5;  // R: Test noise, R increases, dynamic response becomes slower, convergence stability becomes better
// static float Kg = 0;
// static float P_k_k1 = 1;
// static float kalman_rms_old = 0;

// static float kalman_filter(float RMS_Value)
// {
//     float x_k1_k1, x_k_k1;

//     float Z_k;

//     float kalman_rms;
//     Z_k = RMS_Value;
//     x_k1_k1 = kalman_rms_old;

//     x_k_k1 = x_k1_k1;
//     P_k_k1 = P_k1_k1 + Q;

//     Kg = P_k_k1 / (P_k_k1 + R);

//     kalman_rms = x_k_k1 + Kg * (Z_k - kalman_rms_old);
//     P_k1_k1 = (1 - Kg) * P_k_k1;
//     P_k_k1 = P_k1_k1;

//     RMS_OLD_Value = RMS_Value;
//     kalman_rms_old = kalman_rms;

//     return kalman_rms;
// }

static float calculate_rms(uint16_t *values)
{
    calc = 0;
    for (int i = 0; i < SINE_ARR_SIZE; i++)
    {
        calc = calc + ((values[i] - SINE_OFFSET) * (values[i] - SINE_OFFSET));
    }
    calc = calc / SINE_ARR_SIZE;
    calc = sqrtf(calc);

    result = (calc);
    return result;
}

static uint8_t prev_zc = 0;

void StartMotor(uint8_t dir, uint32_t max_time)
{
    if (motor_state.power && (dir == motor_state.direction) ||
        motor_state.errors & MOTOR_LV_ERR_MASK)
        return;

    uint8_t percent = 80;

    if (motor_state.power && motor_state.direction != dir)
    {
        StopMotor(0);
        delay(100);
        // GPIO_WriteLow(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
        // delay(ZERO_CROSS_DELAY); // Ждем отключения симистора (zero cross)
        // percent = 30;
    }

    if ((dir - 1) ^ device_config.motor_dir_alignment)
    {
        GPIO_WriteHigh(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_GPIO_PIN);
    }
    else
    {
        GPIO_WriteLow(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_GPIO_PIN);
    }

    GPIO_WriteHigh(EXT_LAMP_GPIO_PORT, EXT_LAMP_GPIO_PIN);
    delay(RELAY_SWITCH_DELAY);
    // Исключить залипание реле

    uint32_t now = millis();
    while (millis() - now < 1000)
    {

        IWDG_ReloadCounter();
        uint8_t cur_zc = GPIO_ReadInputPin(LOW_VOLT_GPIO_PORT, LOW_VOLT_GPIO_PIN);

        if (prev_zc != cur_zc)
        {

            if (!cur_zc)
            {
                if (percent > 1)
                    percent -= 1;

                GPIO_WriteLow(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
                uint32_t nCount = (16 * 7) * percent;
                while (nCount != 0)
                {
                    nCount--;
                }
                GPIO_WriteHigh(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
            }
            
            prev_zc = cur_zc;
        }
    }

    GPIO_WriteHigh(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);

    motor_state.direction = dir;
    motor_state.power = 0x01;
    motor_state.millis_on = millis();
    motor_state.max_time = max_time;
}

// if run-out остновка на выбеге

void StopMotor(uint8_t run_out)
{
    if (!motor_state.power)
        return;

    GPIO_WriteLow(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);

    delay(214);

    if (!run_out)
    {

        uint32_t now = millis();
        uint8_t cycles = 0;
        prev_zc = 0;
        while (millis() - now < 200)
        {
            IWDG_ReloadCounter();

            uint8_t cur_zc = GPIO_ReadInputPin(LOW_VOLT_GPIO_PORT, LOW_VOLT_GPIO_PIN);
            if (prev_zc != cur_zc)
            {
                if (!cur_zc)
                {
                    if ((cycles % 3 == 0 && cycles < 14) || (cycles % 2 == 0 && cycles > 13))
                    {
                        delay(2);
                        GPIO_WriteHigh(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
                        delay(2);
                        GPIO_WriteLow(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
                    }

                    cycles++;
                }
                prev_zc = cur_zc;
            }
        }

        // Полная остановка путем реверса мотора
        //  GPIO_WriteReverse(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_GPIO_PIN);
        //  delay(RELAY_SWITCH_DELAY);

        // GPIO_WriteHigh(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
        // delay(1);
        // GPIO_WriteLow(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
        // delay(ZERO_CROSS_DELAY);
    }

    delay(RELAY_SWITCH_DELAY);
    GPIO_WriteLow(EXT_LAMP_GPIO_PORT, EXT_LAMP_GPIO_PIN);
    GPIO_WriteLow(MOTOR_DIR_GPIO_PORT, MOTOR_DIR_GPIO_PIN);
    delay(RELAY_SWITCH_DELAY);

    motor_state.power = 0;
    motor_state.last_dir = motor_state.direction;
    motor_state.direction = DIR_NO_MOVE;
}

void MotorUpdate()
{

    uint8_t low_voltage = GPIO_ReadInputPin(LOW_VOLT_GPIO_PORT, (GPIO_Pin_TypeDef)LOW_VOLT_GPIO_PIN);
    if (!low_voltage)
    {
        lv_timer = millis();
        motor_state.errors &= ~MOTOR_LV_ERR_MASK;
    }

    if (millis() - lv_timer > LOW_VOLTAGE_DELAY && !(motor_state.errors & MOTOR_LV_ERR_MASK))
    {
        motor_state.errors |= MOTOR_LV_ERR_MASK;
        StopMotor(0);
    }

    if (sine_finished)
    {
        current_result = (uint32_t)calculate_rms(sine_arr);
        // if (cur_arr_counter < 5 && motor_state.power)
        // {
        //     cur_arr[cur_arr_counter++] = current_result;

        //     if (cur_arr_counter >= 50)
        //     {
        //         cur_arr_counter = 0;
        //     }
        // }

        sine_finished = 0;
    }

    // Проверка тока и макс. времени работы после пуска двигателя
    if (motor_state.power)
    {
        motor_state.time_on = millis() - motor_state.millis_on;
        // Высокий ток - ошибка

        if (motor_state.time_on > motor_state.max_time)
        {
            motor_state.errors |= MOTOR_OT_ERR_MASK;
            StopMotor(0);
        }

        if (motor_state.time_on > IGNORE_OC_TIME)
        {
            if (current_result > motor_state.max_current)
                motor_state.max_current = current_result;

            if (current_result > device_config.max_motor_force)
            {
                motor_state.errors |= MOTOR_OC_ERR_MASK;
                StopMotor(0);
            }
        }
    }
    // uint8_t dp3 = GPIO_ReadInputPin(DP3_GPIO_PORT, (GPIO_Pin_TypeDef)DP3_GPIO_PIN);
    // uint8_t dp4 = GPIO_ReadInputPin(DP4_GPIO_PORT, (GPIO_Pin_TypeDef)DP4_GPIO_PIN);

    // if (!dp4)
    // {
    //     GPIO_WriteHigh(MOTOR_DIR_GPIO_PORT, (GPIO_Pin_TypeDef)MOTOR_DIR_GPIO_PIN);
    // }
    // else
    // {
    //     GPIO_WriteLow(MOTOR_DIR_GPIO_PORT, (GPIO_Pin_TypeDef)MOTOR_DIR_GPIO_PIN);
    // }

    // if (!dp3)
    // {
    //     GPIO_WriteHigh(MOTOR_PWR_GPIO_PORT, (GPIO_Pin_TypeDef)MOTOR_PWR_GPIO_PIN);
    // }
    // else
    // {
    //     GPIO_WriteLow(MOTOR_PWR_GPIO_PORT, (GPIO_Pin_TypeDef)MOTOR_PWR_GPIO_PIN);
    // }
}
