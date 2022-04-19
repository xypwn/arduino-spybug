// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#include "io.hh"

static int serial_putch(char c, FILE *f) {
	(void)f;
	return Serial.write(c) == 1 ? 0 : 1;
}

static int serial_getch(FILE *f) {
	(void)f;
	while(Serial.available() == 0);
	return Serial.read();
}

static FILE serial_in_out;

void io_setup() {
	fdev_setup_stream(&serial_in_out, serial_putch, serial_getch, _FDEV_SETUP_RW);
	stdout = stdin = stderr = &serial_in_out;
}
