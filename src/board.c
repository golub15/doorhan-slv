#include "board.h"

volatile uint32_t timer4_millis = 0;

uint32_t millis()
{
	uint32_t m;

	// disable interrupts while we read timer4_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer4_millis)
	BEGIN_CRITICAL
	m = timer4_millis;
	END_CRITICAL

	return m;
}

void StartBlink(LedBlinkHandle_t *lb, uint8_t times_once, uint8_t times_loop)
{
	lb->time = millis();
	lb->blink_times = times_once;
	lb->blink_times_loop = times_loop;
}

void HandleLed(LedBlinkHandle_t *lb, uint8_t pin)
{
	if (lb->blink_times && millis() > lb->time)
	{
		if (lb->state)
		{
			GPIO_WriteHigh(LED_B_GPIO_PORT, (GPIO_Pin_TypeDef)pin);
			lb->state = 0;

			lb->blink_times--;

			if (lb->blink_times == 0)
			{
				lb->blink_times = lb->blink_times_loop;
				lb->time = millis() + 2000;
			}
			else
				lb->time = millis() + 500;
		}
		else
		{
			lb->time = millis() + 100;
			GPIO_WriteLow(LED_B_GPIO_PORT, (GPIO_Pin_TypeDef)pin);
			lb->state = 1;
		}
	}
}

uint8_t InputTrig(SwitchControl_t *sc, uint8_t mode)
{
	uint8_t prev = (sc->flags & mode);
	sc->flags &= ~mode;

	return prev;
}

void UpdateSwitch(SwitchControl_t *sc)
{
	if (sc->input_value)
	{
		if (!sc->state || (sc->state <= 250 && millis() - sc->time > DEBOUNCE_DELAY))
		{
			sc->state += 1;
			sc->time = millis();

			if (sc->state == 2)
				sc->flags |= TRIG_SET;
		}
	}
	else if (sc->state)
	{
		sc->time = millis();
		sc->flags |= TRIG_RESET;
		if (sc->state > 1)
			sc->flags &= ~TRIG_DELAY_F;

		if (sc->state > HOLD_TIME)
			sc->flags |= TRIG_HOLD;

		sc->state = 0;
	}

	if (millis() - sc->time > OFF_DELAY && !(sc->flags & TRIG_DELAY_F))
	{
		sc->flags |= (TRIG_DELAY | TRIG_DELAY_F);
	}
}

void delay(uint32_t time)
{
	uint32_t cur_time = millis();
	while (millis() - cur_time < time)
	{
		IWDG_ReloadCounter();
	}
}