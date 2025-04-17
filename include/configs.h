#define OVER_FORCE_DELAY        10000 // ms Задержка пуска после аварии по силе
#define ZERO_CROSS_DELAY        100    // ms Ожидание отсутвия тока через симистор
#define RELAY_SWITCH_DELAY      50    // ms Переключение контактов реле

#define STOP_REVERSE_TIME       100   // ms Время обратной подачи движения для остановки

#define IGNORE_OC_TIME          1100   // ms Задержка чтения тока после пуска
#define LOW_VOLTAGE_DELAY       3000  // ms Макс. время низкого напряжения до аварии
#define INIT_DELAY              5000  // ms Задержка первого пуска, после включения

#define TRY_OPEN_AFTER_OC       10    // 
#define MAX_AUTOCLOSE_DELAY     30000


#define MAX_OPENING_TIME        30500  // мс Макс. время открытия
#define MAX_CLOSEING_TIME       30500  // мс Макс. время закрытия


#define EEPROM_ADDR_START       0x40000