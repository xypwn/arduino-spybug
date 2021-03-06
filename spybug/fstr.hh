// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#pragma once

#include <Arduino.h>

size_t fstrlen(const __FlashStringHelper *s);

bool fstreq(const char *a, const __FlashStringHelper *b_fsh);

int printf(const __FlashStringHelper *fmt, ...);
