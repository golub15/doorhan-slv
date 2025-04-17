#ifndef __MOTOR
#define __MOTOR

#include "main.h"
#include "math.h"

#define SINE_ARR_SIZE 20 // Буффер для расчета тока
#define SINE_OFFSET 512

#define MOTOR_OT_ERR_MASK (0x01U << 0) // Ошибка маск. времени
#define MOTOR_LV_ERR_MASK (0x01U << 1) // Ошибка низкого напряжения
#define MOTOR_OC_ERR_MASK (0x01U << 2) // Ошибка перегрузки по току (по силе)

#define DIR_NO_MOVE 0x00U
#define DIR_OPEN 0x01U
#define DIR_CLOSE 0x02U

typedef struct MotorState
{
    uint8_t direction;  // направление
    uint8_t last_dir;   // направление
    uint8_t power;      // питание (тиристор)
    uint8_t errors;     // Ошибки (по битовым маскаи)
    uint32_t millis_on; // метка. времени с момента включения
    uint32_t time_on;   // Время работы в мс.
    uint32_t max_time;  // Макс. допустимое время работы
    uint32_t max_current; // Max force while motor
} MotorState_t;

void StartMotor(uint8_t dir, uint32_t max_time);
void StopMotor(uint8_t run_out); // if run-out остновка на выбеге

static float calculate_rms(uint16_t *values);
void MotorUpdate();
// Adding time
// Check time

// LowVoltage check
// OverCurrent check (force)
#endif
