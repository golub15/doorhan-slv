/* Includes ------------------------------------------------------------------*/
#include <main.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t mot_cur_val;
volatile uint16_t reverse_val;
volatile uint16_t auto_cl_val;
volatile uint16_t force_val;

GateState_t gate_state;
MotorState_t motor_state;
DevConfig_t device_config;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */

// typedef struct eerpom_struct
// {
//   uint32_t a;
//   uint32_t b;
// } eerpom_struct_t;

// union
// {
//   eerpom_struct_t es;
//   uint8_t sector[32];
// } eeprom_data;

void main(void)
{
  // CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // шина тактировния APB
  /* Clear High speed internal clock prescaler */
  CLK->CKDIVR &= (uint8_t)(0x01);

  GPIO_Init(GPIOD,
            INT_LAMP_GPIO_PIN | MOTOR_DIR_GPIO_PIN |
                MOTOR_PWR_GPIO_PIN | EXT_LAMP_GPIO_PIN,
            GPIO_MODE_OUT_PP_LOW_SLOW);

  IWDG_Enable();
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: LSI/128 */
  IWDG_SetPrescaler(IWDG_Prescaler_256);

  /* Set counter reload value to obtain 250ms IWDG Timeout.
    Counter Reload Value = 250ms/IWDG counter clock period
                         = 250ms / (LSI/128)
                         = 0.25s / (LsiFreq/128)
                         = LsiFreq/(128 * 4)
                         = LsiFreq/512
   */

  // 16*10^6 / 256 / 256 = ~244 ms
  IWDG_SetReload((uint8_t)255);
  IWDG_ReloadCounter();

  // FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  // FLASH_Unlock(FLASH_MEMTYPE_DATA);

  // for (int i = 0; i < sizeof(eeprom_data.sector); i++)
  // {
  //   eeprom_data.sector[i] = *(PointerAttr uint8_t *)(MemoryAddressCast)(addr + i);
  //   //*(PointerAttr uint8_t *)(MemoryAddressCast)add = (uint8_t)234;
  // }

  // eeprom_data.es.a = 0xAA55AA55;
  // eeprom_data.es.a ;

  // for (int i = 0; i < sizeof(eeprom_data.sector); i++)
  // {
  //   *(PointerAttr uint8_t *)(MemoryAddressCast)(addr + i) = 0x00;
  //   *(PointerAttr uint8_t *)(MemoryAddressCast)(addr + i) = eeprom_data.sector[i];
  //   if(*(PointerAttr uint8_t *)(MemoryAddressCast)(addr + i) != eeprom_data.sector[i]){
  //     nop();
  //   }
  //   //*(PointerAttr uint8_t *)(MemoryAddressCast)add = (uint8_t)234;
  // }

  // FLASH_Lock(FLASH_MEMTYPE_DATA);
  /* Read a byte at a specified address */
  // val = FLASH_ReadByte(add);

  // CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  // TIM4_DeInit();
  //  set timer 4 prescale factor (typ. @16MHz: 64)
  // TIM4->PSCR = (uint8_t)(TIM4_PRESCALER_64);
  // // set timer 4 autoreload value/period (typ. @t16MHz: 250-1)
  // TIM4->ARR = (uint8_t)(250 - 1);
  // /* Clear TIM4 update flag by writing 0. Writing ones has no effect */
  // TIM4->SR1 = (uint8_t)(~TIM4_FLAG_UPDATE);
  // /* Enable update interrupt */
  // TIM4->IER |= TIM4_IT_UPDATE;
  // /* Enable TIM4 */
  // TIM4->CR1 |= TIM4_CR1_CEN;

  // // set timer 4 prescale factor and period (typ. @16MHz: 64*250=1ms)
  // TIM4_TimeBaseInit(TIM4_PRESCALER_64, (uint8_t) 250-1);
  // /* Clear TIM4 update flag */
  // TIM4_ClearFlag(TIM4_FLAG_UPDATE);	// TIM4->SR1 = (uint8_t)(~0x01);
  // /* Enable update interrupt */
  // TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);	// TIM4->IER |= (uint8_t)TIM4_IT;
  // /* Enable TIM4 */
  // TIM4_Cmd(ENABLE);	// TIM4->CR1 |= TIM4_CR1_CEN;

  GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOB, GPIO_PIN_1, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);

  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_3, ADC1_PRESSEL_FCPU_D18, ADC1_EXTTRIG_TIM, ENABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_CHANNEL0, DISABLE);
  ADC1_SchmittTriggerConfig(ADC1_SCHMITTTRIG_CHANNEL1, DISABLE);

  ADC1_DataBufferCmd(ENABLE);
  ADC1_ScanModeCmd(ENABLE);

  ADC1_ClearFlag(ADC1_FLAG_EOC);
  // Включаем прерывание от заполнения буфера.
  ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);

  ADC1_ConversionConfig(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_3, ADC1_ALIGN_RIGHT);

  // TIM1

  /* Set the Autoreload value */
  TIM1->ARRH = (uint8_t)((10 - 1) >> 8);
  TIM1->ARRL = (uint8_t)(10 - 1);
  /* Set the Prescaler value */
  TIM1->PSCRH = (uint8_t)((1600 - 1) >> 8);
  TIM1->PSCRL = (uint8_t)((1600 - 1));
  /* Select the Counter Mode */
  TIM1->CR1 = (uint8_t)((uint8_t)(TIM1->CR1 & (uint8_t)(~(TIM1_CR1_CMS | TIM1_CR1_DIR))) | (uint8_t)(TIM1_COUNTERMODE_UP));
  /* Set the Repetition Counter value */
  TIM1->RCR = 0x00U;

  // Select output trigger
  TIM1->CR2 = (uint8_t)((uint8_t)(TIM1->CR2 & (uint8_t)(~TIM1_CR2_MMS)) |
                        (uint8_t)TIM1_TRGOSOURCE_UPDATE);

  TIM1->IER |= TIM1_IT_UPDATE;
  TIM1->CR1 |= TIM1_CR1_CEN;

  // TIM1_TimeBaseInit(TIM4_PRESCALER_64, TIM1_COUNTERMODE_UP, 250 - 1, 0);
  //  TIM1_Cmd(ENABLE);
  //  ADC1_StartConversion();
  /* Initialize I/Os in Output CmdMode */
  GPIO_Init(GPIOB,
            LED_A_GPIO_PIN | LED_B_GPIO_PIN,
            GPIO_MODE_OUT_PP_HIGH_FAST);

  GPIO_Init(GPIOB,
            DP1_GPIO_PIN,
            GPIO_MODE_IN_FL_NO_IT);

  GPIO_Init(GPIOC,
            START_GPIO_PIN | PED_GPIO_PIN | PH_OP_GPIO_PIN |
                PH_CL_GPIO_PIN | STOP_GPIO_PIN | DP3_GPIO_PIN | DP4_GPIO_PIN,
            GPIO_MODE_IN_FL_NO_IT);

  GPIO_Init(GPIOF,
            BTN_GPIO_PIN,
            GPIO_MODE_IN_FL_NO_IT);

  GPIO_Init(GPIOD,
            SW_CL_GPIO_PIN | SW_OP_GPIO_PIN | LOW_VOLT_GPIO_PIN,
            GPIO_MODE_IN_FL_NO_IT);

  enableInterrupts();

  GPIO_WriteLow(EXT_LAMP_GPIO_PORT, EXT_LAMP_GPIO_PIN);
  GPIO_WriteLow(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
  delay(500);

  device_config.motor_dir_alignment = GPIO_ReadInputPin(DP1_GPIO_PORT, (GPIO_Pin_TypeDef)DP1_GPIO_PIN) > 0 ? 0x00 : 0x01;
  device_config.open_after_close = GPIO_ReadInputPin(DP3_GPIO_PORT, (GPIO_Pin_TypeDef)DP3_GPIO_PIN) > 0 ? 0x00 : 0x01;

  BEGIN_CRITICAL
  device_config.auto_close_time = ((MAX_AUTOCLOSE_DELAY / 1024) * (1023 - auto_cl_val));
  device_config.max_motor_force = (1023 - force_val);
  END_CRITICAL

  gate_state.position = GATE_UNKNOWN;
  motor_state.direction = DIR_NO_MOVE;
  motor_state.errors = 0x00;
  motor_state.max_current = 0;

  //  GPIO_WriteHigh(EXT_LAMP_GPIO_PORT, EXT_LAMP_GPIO_PIN);

  while (1)
  {
    // GPIO_WriteReverse(MOTOR_PWR_GPIO_PORT, MOTOR_PWR_GPIO_PIN);
    // delay(1000);

    if (!GPIO_ReadInputPin(DP4_GPIO_PORT, DP4_GPIO_PIN))
    {
      BEGIN_CRITICAL
      device_config.auto_close_time = ((MAX_AUTOCLOSE_DELAY / 1024) * (1023 - auto_cl_val));
      device_config.max_motor_force = (1023 - force_val);
      END_CRITICAL
    }

    MotorUpdate();
    GatesUpdate();

    IWDG_ReloadCounter();
  }
}