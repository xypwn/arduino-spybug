// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#pragma once

#include <EEPROM.h>

#define EEADDR_SETTINGS 0x00
struct EEPROM_Settings_Class {
	unsigned long recording_delay = 0l;
	bool          serial_log      = true;

	inline void load() { EEPROM.get(EEADDR_SETTINGS, *this); }
	inline void save() { EEPROM.put(EEADDR_SETTINGS, *this); }
};

extern EEPROM_Settings_Class settings;
