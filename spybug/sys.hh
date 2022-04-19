// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#pragma once

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

static void (*full_reset)() = nullptr;

void low_power_sleep_minutes(unsigned long t);

static inline void wdt_enable_with_full_reset() {
	wdt_enable(WDTO_8S);   /* Start watchdog timer for 8s. */
	WDTCSR |= (1 << WDIE); /* Enable watchdog interrupt. */
}

static inline void disable_recording_interrupts() {
	TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));
}
