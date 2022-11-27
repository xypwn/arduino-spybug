// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#include "sys.hh"

void (*full_reset)() = nullptr;

static volatile bool wdt_in_sleep_mode = false;
ISR (WDT_vect) {
	if (wdt_in_sleep_mode)
		wdt_disable();
	else
		full_reset();
}

/* Based on https://github.com/rocketscream/Low-Power. */
void low_power_sleep_minutes(unsigned long t) {
	wdt_in_sleep_mode = true;
	ADCSRA &= ~_BV(ADEN); /* Disable ADC. */
	for (unsigned long i = 0; 8ul * i < 60ul * t; i++) {
		// Power Down for 8s
		wdt_enable(WDTO_8S);   /* Start watchdog timer for 8s. */
		WDTCSR |= (1 << WDIE); /* Enable watchdog interrupt. */
		do {
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			cli();
			sleep_enable();
			sleep_bod_disable();
			sei();
			sleep_cpu();
			sleep_disable();
			sei();
		} while (0);
	}
	ADCSRA |= _BV(ADEN); /* Re-enable ADC. */
	wdt_in_sleep_mode = false;
}
