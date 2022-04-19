// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#include "settings.hh"

void setup() {
	Serial.begin(9600);
	Serial.print("Resetting EEPROM...");
	EEPROM_Settings_Class settings;
	settings.save();
	Serial.println("done.");
}

void loop() {}
