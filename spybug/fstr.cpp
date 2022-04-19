// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#include "fstr.hh"

size_t fstrlen(const __FlashStringHelper *s) {
	PGM_P sp = (PGM_P)s;
	size_t len = 0;
	while (pgm_read_byte(sp++))
		len++;
	return len;
}

bool fstreq(const char *a, const __FlashStringHelper *b_fsh) {
	PGM_P b = (PGM_P)b_fsh;
	while (1) {
		if (*a != pgm_read_byte(b))
			return false;
		if (*a == 0)
			return true;
		a++; b++;
	}
}

int printf(const __FlashStringHelper *fmt, ...) {
	size_t len = fstrlen(fmt);
	char buf[len + 1];
	buf[len] = 0;
	memcpy_P(buf, fmt, len + 1);

	va_list args;
	va_start(args, fmt);
	int ret = vprintf(buf, args);
	va_end(args);
	return ret;
}

