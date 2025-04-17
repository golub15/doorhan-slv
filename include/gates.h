#ifndef __GATES
#define __GATES

#include "main.h"

#define GATE_UNKNOWN (0x00U)
#define GATE_OPEN (0x01U)
#define GATE_CLOSE (0x02U)

typedef struct
{
    uint16_t auto_close_time;
    uint16_t max_motor_force;
    uint8_t motor_dir_alignment;
    uint8_t open_after_close;

    // struct
    // {
    //     uint8_t over_force;
    //     uint8_t over_time;
    //     uint8_t low_voltage;
    // } error_stats; // Счетчики на каждый тип аварии (ошибки)

} DevConfig_t;

typedef struct GateState
{
    uint8_t position;    // Тек. положение
    uint8_t current_dir; // Тек. направление
} GateState_t;

void SafeOpen();
void SafeClose();

void GatesOpen();   // Просто открыть ворота на заданое время с safety
void GatesUpdate(); // Обработка концевых и времени

#endif