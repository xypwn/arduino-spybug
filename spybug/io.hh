// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#pragma once

#include <Arduino.h>

void io_setup();

#define print_special(x) { Serial.print(x); }
#define die(fmt, ...) { disable_recording_interrupts(); if (settings.serial_log) { printf(F("Fatal: ")); printf(fmt, ##__VA_ARGS__); Serial.flush(); } while(1); }
#define dbg(fmt, ...) { printf(F("Debug: ")); printf(fmt, ##__VA_ARGS__); }
#define info(fmt, ...) { if (settings.serial_log) printf(fmt, ##__VA_ARGS__); }
#define info_special(x) { if (settings.serial_log) Serial.print(x); }
