#include <gates.h>

extern GateState_t gate_state;
extern MotorState_t motor_state;
extern DevConfig_t device_config;

extern volatile uint16_t auto_cl_val;
extern volatile uint16_t force_val;

static uint8_t init = 0;
static uint8_t prev_motor_error = 0;

static uint8_t  auto_close = 1; // Цикл открытия ворот
static uint32_t last_opening_time = 0;
static uint32_t time_init = 0;

// Счетчик попыток открыться после превышения силы
static uint8_t try_OС_clear = 0;

static LedBlinkHandle_t led_1 = {
    .time = 0,
    .blink_times = 2,
};

static LedBlinkHandle_t led_2 = {
    .time = 0,
    .blink_times = 2,
};

static SwitchControl_t photo = {

    .state = 0,
};

static SwitchControl_t btn = {

    .state = 0,
};

static SwitchControl_t start = {

    .state = 0,
};

static SwitchControl_t sw_close = {

    .state = 0,
};

static SwitchControl_t sw_open = {

    .state = 0,
};

static SwitchControl_t stop = {

    .state = 0,
};

static SwitchControl_t auto_close_blink = {

    .state = 0,
};

// typedef struct ProgTimer
// {
//     uint32_t period;
//     uint8_t flags;

// } ProgTimer_t;

#define PTIM_START (0x01 << 0)
#define PTIM_TRIG (0x01 << 1)

// static uint8_t trigger_on_position;

// uint8_t ValueTrig(uint8_t val, uint8_t *val_trig){

//     uint8_t v = *val_trig != val;
//     *val_trig = val;
//     return v;

// }

// void PtimerUpdate(ProgTimer_t *pt)
// {
//     if ((pr->flags & PTIM_START) && millis() - pt->time > pt->period)
//     {
//         pt->flags |= PTIM_TRIG;
//         pt->flags &= ~PTIM_START;
//     }
// }

// void GateInit(uint32_t init_delay)
// {
//     gate_state.position = GATE_UNKNOWN;
// }

void SafeOpen()
{
    if (sw_open.state > 0)
        return;
    StartMotor(DIR_OPEN, MAX_OPENING_TIME);
}

void SafeClose()
{
    if (sw_close.state > 0)
        return;
    StartMotor(DIR_CLOSE, MAX_OPENING_TIME);
}

void GatesOpen()
{
    if (gate_state.position == GATE_OPEN && !motor_state.power)
        return;

    if (motor_state.direction == DIR_CLOSE && !device_config.open_after_close)
        return;

    SafeOpen();
}

void GatesUpdate()
{
    HandleLed(&led_1, LED_A_GPIO_PIN);
    HandleLed(&led_2, LED_B_GPIO_PIN);

    btn.input_value = !GPIO_ReadInputPin(BTN_GPIO_PORT, BTN_GPIO_PIN);
    start.input_value = !GPIO_ReadInputPin(START_GPIO_PORT, START_GPIO_PIN);   // НО
    sw_close.input_value = GPIO_ReadInputPin(SW_CL_GPIO_PORT, SW_CL_GPIO_PIN); // НЗ
    sw_open.input_value = GPIO_ReadInputPin(SW_OP_GPIO_PORT, SW_OP_GPIO_PIN);  // НЗ
    photo.input_value = GPIO_ReadInputPin(PH_CL_GPIO_PORT, PH_CL_GPIO_PIN);    // НЗ

    stop.input_value = GPIO_ReadInputPin(STOP_GPIO_PORT, STOP_GPIO_PIN); // НЗ
    stop.input_value |= !GPIO_ReadInputPin(DP4_GPIO_PORT, DP4_GPIO_PIN); // НО

    UpdateSwitch(&photo);
    UpdateSwitch(&btn);
    UpdateSwitch(&auto_close_blink);

    UpdateSwitch(&stop);

    UpdateSwitch(&sw_close);
    UpdateSwitch(&sw_open);

    UpdateSwitch(&start);

    // Обработка безопасности

    // 1. Кнопка СТОП
    // + DIP4

    // СТОП сигнал есть
    if (InputTrig(&stop, TRIG_SET))
    {
        gate_state.position = GATE_UNKNOWN;
        init = 0;

        StopMotor(0);
        StartBlink(&led_1, 0, 0);
        StartBlink(&led_2, 10, 10);
    }

    // СТОП сигнал снят
    if (InputTrig(&stop, TRIG_RESET))
    {
        StartBlink(&led_2, 1, 0);
        GPIO_WriteHigh(LED_A_GPIO_PORT, LED_A_GPIO_PIN);
    }

    // 2. Фотоэлементы
    if (InputTrig(&photo, TRIG_SET) && motor_state.direction == DIR_CLOSE)
    {
        SafeOpen();
    }

    // 3. если мотор работает но концвые удержаны больше 3 сек то ошибка
    if (motor_state.power && (sw_close.state * DEBOUNCE_DELAY > 3000 || sw_open.state * DEBOUNCE_DELAY > 3000))
    {
        motor_state.errors |= MOTOR_OC_ERR_MASK;
        StopMotor(0);
    }

    if (stop.state)
    {
        // > val * 1.05
        uint16_t force_val_now = 0;
        BEGIN_CRITICAL
        force_val_now = (1023 - force_val);
        END_CRITICAL

        StartBlink(&led_1, 0, 0);
        if (force_val_now > motor_state.max_current + (motor_state.max_current / 20))
        {
            GPIO_WriteLow(LED_A_GPIO_PORT, LED_A_GPIO_PIN);
        }
        else if (force_val_now > motor_state.max_current)
            GPIO_WriteLow(LED_A_GPIO_PORT, LED_A_GPIO_PIN);
        else
            GPIO_WriteHigh(LED_A_GPIO_PORT, LED_A_GPIO_PIN);

        device_config.max_motor_force = force_val_now;

        // if (sw_open.state || sw_close.state || photo.state)
        //     GPIO_WriteLow(LED_A_GPIO_PORT, LED_A_GPIO_PIN);
        // else
        //     GPIO_WriteHigh(LED_A_GPIO_PORT, LED_A_GPIO_PIN);

        return;
    }

    // Обработчик ошибок/аварий
    if (motor_state.errors != prev_motor_error)
    {
        // Запоминаем если появились ошибки
        // Ждем и пробуем открыться или закрыться (по тек. положени)
        StartBlink(&led_2, 1, motor_state.errors);

        if (!motor_state.errors)
        {
            // 1. низкое напряжение = как только норма = сброс
            if (prev_motor_error & MOTOR_LV_ERR_MASK && gate_state.position != GATE_CLOSE)
            {
                init = 0;
                gate_state.position = GATE_UNKNOWN;
            }
        }

        // 2. Перегрузка по силе (току),
        // при закрытии, ждем, затем, открываемся
        // при открытии, ждем время и заново

        if (motor_state.errors & MOTOR_OC_ERR_MASK)
        {
            init = 0;
            gate_state.position = GATE_UNKNOWN;


            motor_state.errors = 0x00;
            //motor_state.errors |= MOTOR_OC_ERR_MASK;
        }

        //3. Превышение времени открытия
        //Считаем что в нужном положенни

        if (motor_state.errors & MOTOR_OT_ERR_MASK)
        {
            // Нет концевого после открытия, считаем что открылись
            gate_state.position = motor_state.last_dir;
            last_opening_time = millis();
            motor_state.errors = 0x00;
        }

        prev_motor_error = motor_state.errors;
    }

    // Мигать при автозакрытии
    if (InputTrig(&auto_close_blink, TRIG_SET))
        StartBlink(&led_1, 1, 100);
    else if (InputTrig(&auto_close_blink, TRIG_RESET))
        StartBlink(&led_1, 1, 0);

    // Кнопка открывает ворота
    if (InputTrig(&btn, TRIG_CLICK))
    {
        GatesOpen();
    }

    // Закрытие после препятствий фото-элемента
    if (InputTrig(&photo, TRIG_RESET))
    {
        last_opening_time = millis();
    }

    // Сигнал на старт установлен
    if (InputTrig(&start, TRIG_SET))
    {
        GatesOpen();
        if (try_OС_clear >= TRY_OPEN_AFTER_OC)
        {
            try_OС_clear = 0;
            init = 0;
        }
        auto_close = 0; // Если идет удрежание
    }

    // Сигнал на старт сброшен
    if (InputTrig(&start, TRIG_RESET))
    {
        last_opening_time = millis();
        auto_close = 1;
    }

    // Автозакрытие
    if (!photo.state &&
        auto_close &&
        gate_state.position == GATE_OPEN &&
        motor_state.direction == DIR_NO_MOVE)
    {
        if (millis() - last_opening_time > device_config.auto_close_time)
            SafeClose();
        else
        {
            auto_close_blink.input_value = 1;
        }
    }
    else
        auto_close_blink.input_value = 0;

    // Добавить временной гистерезис для концевых (прошло_время_с_сработки > 1000мс)
    if (sw_close.state > 1 && (((InputTrig(&sw_close, TRIG_DELAY) || motor_state.direction == DIR_CLOSE) && motor_state.power) || gate_state.position == GATE_UNKNOWN))
    {
        gate_state.position = GATE_CLOSE;
        StopMotor(0);
    }
    else if (sw_open.state > 1 && (((InputTrig(&sw_open, TRIG_DELAY) || motor_state.direction == DIR_OPEN) && motor_state.power) || gate_state.position == GATE_UNKNOWN))
    {
        StopMotor(0);

        gate_state.position = GATE_OPEN;
        last_opening_time = millis();
        try_OС_clear = 0;

        init = 0x00;
    }

    // Начало работы
    // пробуем открыться, если не получилось, по обычной логике
    if (!init && !(motor_state.errors & MOTOR_LV_ERR_MASK) &&
        motor_state.direction == DIR_NO_MOVE &&
        gate_state.position == GATE_UNKNOWN)
    {
        init = 0x01;
        time_init = millis();
    }

    // if (PtimerTrig(&init_delay) && gate_state.position == GATE_UNKNOWN)
    // {
    //     StartMotor(DIR_OPEN, MAX_CLOSEING_TIME);
    // }

    if (init && millis() - time_init >
                    (motor_state.errors & MOTOR_OC_ERR_MASK ? OVER_FORCE_DELAY : INIT_DELAY))
    {
        if (motor_state.errors & MOTOR_OC_ERR_MASK)
        {

            if (try_OС_clear <= 10)
            {
                SafeOpen();
                try_OС_clear++;
                motor_state.errors &= ~MOTOR_OC_ERR_MASK;
            }
        }
        else
            SafeOpen();

        init = 0x00;
    }
}
