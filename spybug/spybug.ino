// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

/*
	*** SD Card Wiring ***
	SD       | Nano
	______________________
	D0  (DO) |  D12 (MISO)
	VSS      |  GND
	CLK      |  D13 (SCK)
	VDD      |  5V or 3V3
	CMD (DI) |  D11 (MOSI)
	D3  (CS) |  D10 (SS)

	WARNING: SD cards are not designed for 5V; I have been using 5V anyways
	and everything seems fine, but beware that there is a significant risk
	of immediate or premature failure when not using a buffer circuit.
	
	SD pin D3 is the chip select pin. It can be set manually in PIN_SS.

	*** Microphone Wiring (MAX9814 w/ electret microphone) ***
	Mic      | Nano
	______________________
	VCC      |  5V
	GND      |  GND
	Out      |  A0
	
	Out defaults to A0 (AdcChannel0), but can be set manually in ADC_CHANNEL.
*/

#include <SD.h>
#include <SPI.h>

#include "aaa_config.hh"
#include "cmd.hh"
#include "fstr.hh"
#include "io.hh"
#include "settings.hh"
#include "sys.hh"

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000
#error "This program only works on ATmega328P devices with a clock frequency of 16MHz!"
#endif

enum AdcChannel : uint8_t {
	AdcChannel0    = 0,
	AdcChannel1    = 1,
	AdcChannel2    = 2,
	AdcChannel3    = 3,
	AdcChannel4    = 4,
	AdcChannel5    = 5,
	AdcChannel6    = 6,
	AdcChannel7    = 7,
	AdcChannelTemp = 8,
	AdcChannel1V1  = 14,
	AdcChannelGnd  = 15,
};

File file;
#if defined(SAMPLE_MODE_U8)
#define SAMPLE_BUF_SIZE 256
#define SAMPLE_BUF_TYPE uint8_t
#elif defined(SAMPLE_MODE_S16)
#define SAMPLE_BUF_SIZE 160
#define SAMPLE_BUF_TYPE int16_t
#endif
volatile SAMPLE_BUF_TYPE sample_buffer[2][SAMPLE_BUF_SIZE];
volatile bool which_buffer = 0;
volatile uint16_t samples_in_buffer[2] = {0, 0};
volatile unsigned long samples_hanging = 0;
volatile unsigned long samples_written = 0;
volatile unsigned long samples_dropped = 0;
#ifdef DEBUG_RECORDING
volatile unsigned long dbg_total = 0;
volatile unsigned long dbg_sum = 0;
volatile unsigned long dbg_samples = 0;
volatile int16_t dbg_min = 32767;
volatile int16_t dbg_max = -32768;
#endif

ISR(TIMER1_COMPA_vect) {
	/* Only write to file, if one of the buffers is full (meaning no access conflicts). */
	if (samples_in_buffer[!which_buffer] == SAMPLE_BUF_SIZE) {
		TIMSK1 &= ~_BV(OCIE1A);
		sei();
		const size_t bufsz = sizeof(SAMPLE_BUF_TYPE) * samples_in_buffer[!which_buffer];
		if (file.write((char*)sample_buffer[!which_buffer], bufsz) != bufsz) {
			info(F("Lost "));
			info_special((float)samples_hanging / (float)(F_CPU / TIMER_COMPARE)); /* Printf doesn't handle floats. */
			info(F(" seconds of recording.\n"));
			die(F("Error writing to SD card. You can ignore this if you removed the SD card intentionally.\n"));
		}
		samples_hanging += samples_in_buffer[!which_buffer];
		samples_in_buffer[!which_buffer] = 0;
		if (samples_hanging >= FLUSH_SAMPLES) {
			samples_written += samples_hanging;
			samples_hanging = 0;
			wav_write_header(samples_written);
			file.flush();
		}
		TIMSK1 |= _BV(OCIE1A);
	}
}

ISR(TIMER1_COMPB_vect) {
	// Retrieve ADC Value and Write to Buffer
#if defined(SAMPLE_MODE_U8)
#ifdef U8_AMPLIFY_X2
	uint8_t l = ADCL; /* Read ADC registers. (Order matters!) */
	uint8_t h = ADCH;
	uint8_t adcval = (h << 7) | (l >> 1);
#else
	uint8_t adcval = ADCH;
#endif
#elif defined(SAMPLE_MODE_S16)
	uint8_t l = ADCL;
	uint8_t h = ADCH;
	int16_t adcval = (h << 8) | l;
	adcval -= 0x0200; /* Make integer signed. */
	adcval <<= 6; /* Turn 10-bit integer into 16-bit integer. */
#endif
	if (samples_in_buffer[which_buffer] >= SAMPLE_BUF_SIZE)
		which_buffer = !which_buffer;
	if (samples_in_buffer[which_buffer] < SAMPLE_BUF_SIZE)
		sample_buffer[which_buffer][samples_in_buffer[which_buffer]++] = adcval;
	else
		samples_dropped++;
#ifdef DEBUG_RECORDING
	dbg_total++;
	dbg_samples++;
	dbg_sum += adcval;
	if (adcval < dbg_min)
		dbg_min = adcval;
	if (adcval > dbg_max)
		dbg_max = adcval;
#endif
}

static void wav_write_header(uint32_t nsamples) {
	unsigned long old_pos = file.position();

	if (!file.seek(0))
		die(F("Error seeking to position 0!\n"));

	const uint16_t channels        = 1;
	const uint32_t riff_chunk_size = sizeof(SAMPLE_BUF_TYPE) * nsamples * channels + 4 + 24 + 8;
	const uint32_t fmt_chunk_size  = 16;
	const uint16_t fmt_tag         = 1; /* 1 = PCM. */
	const uint32_t sample_rate     = F_CPU / TIMER_COMPARE;
	const uint32_t data_rate       = sizeof(SAMPLE_BUF_TYPE) * channels * sample_rate;
	const uint16_t block_align     = sizeof(SAMPLE_BUF_TYPE) * channels;
	const uint16_t bits_per_sample = sizeof(SAMPLE_BUF_TYPE) * 8;
	const uint32_t data_size       = sizeof(SAMPLE_BUF_TYPE) * nsamples * channels;

	// RIFF
	if(file.write((char*)"RIFF", 4) != 4
	|| file.write((char*)&riff_chunk_size, 4) != 4
	|| file.write((char*)"WAVE", 4) != 4
	// fmt
	|| file.write((char*)"fmt ", 4) != 4
	|| file.write((char*)&fmt_chunk_size, 4) != 4
	|| file.write((char*)&fmt_tag, 2) != 2
	|| file.write((char*)&channels, 2) != 2
	|| file.write((char*)&sample_rate, 4) != 4
	|| file.write((char*)&data_rate, 4) != 4
	|| file.write((char*)&block_align, 2) != 2
	|| file.write((char*)&bits_per_sample, 2) != 2
	// data
	|| file.write((char*)"data", 4) != 4
	|| file.write((char*)&data_size, 4) != 4)
		die(F("Error writing WAV header to SD card!\n"));

	if (old_pos > file.position()) {
		if(!file.seek(old_pos))
			die(F("Error seeking to position %lu!\n"), old_pos);
	}
}

void setup() {
	// Serial Setup
	Serial.begin(9600); /* Set baud rate. */
	io_setup(); /* Add printf support. */
	// Component Switch Setup
#ifdef PIN_COMPONENT_SWITCH
	pinMode(PIN_COMPONENT_SWITCH, OUTPUT);
#endif
	// Load EEPROM Data
	settings.load();
	// Handle Commands
	info(F("Type anything in the next 4s to enter command mode.\n"));
	for (size_t i = 0; i < 4 * 4; i++) {
		if (Serial.available())
			cmd();
		delay(250);
	}
	// Delayed Triggering
	if (settings.recording_delay) {
#ifdef PIN_COMPONENT_SWITCH
		digitalWrite(PIN_COMPONENT_SWITCH, !COMPONENT_SWITCH_ON);
#endif
		info(F("Sleeping for %lu minute%s before starting to record...\n"), settings.recording_delay, settings.recording_delay == 1 ? "" : "s");
		Serial.flush();
		/* Using this function, an Arduino Nano (with its voltage regulator and TTL module removed) draws ~6μA. */
		low_power_sleep_minutes(settings.recording_delay);
		/* Reset wait time. */
		settings.recording_delay = 0;
		settings.save();
	}
	// Activate Components
#ifdef PIN_COMPONENT_SWITCH
	digitalWrite(PIN_COMPONENT_SWITCH, COMPONENT_SWITCH_ON);
	delay(500); /* Wait for components to initialize. */
#endif
	// Start Watchdog (wdt_enable() doesn't fully reset)
	wdt_enable_with_full_reset();
	// SD Card Setup
	if (!SD.begin(PIN_SS))
		die(F("Error initializing SD card!\n"));
	// Determine Filename
	unsigned int filenum = 0;
	char filename[32];
	do {
		filenum++;
		snprintf(filename, 32, REC_FILE_FMT, filenum);
	} while (SD.exists(filename));
	// Open File
	file = SD.open(filename, O_READ | O_WRITE | O_CREAT); /* Seeking doesn't seem to work with FILE_WRITE?! */
	info(F("Recording to file '%s'.\n"), filename);
	if (!file)
		die(F("Error opening '%s' for writing!\n"), filename);
	wav_write_header(0);
	// ADC Setup
	DIDR0 |= (0xF & ADC_CHANNEL); /* Disable digital input. */
	ADCSRA = _BV(ADEN) /* Enable ADC. */
	       | _BV(ADATE) /* Enable auto-trigger. */
#if defined(ADC_PRESCALE_64) /* Up to ~18kHz. */
	       | _BV(ADPS2) | _BV(ADPS1); /* ADC prescaler division factor: 64. */
#elif defined(ADC_PRESCALE_32) /* Up to ~27kHz. */
	       | _BV(ADPS2) | _BV(ADPS0); /* ADC prescaler division factor: 32. */
#elif defined(ADC_PRESCALE_16) /* Up to ~60kHz. */
	       | _BV(ADPS2); /* ADC prescaler division factor: 16. */
#endif
	ADCSRB = _BV(ADTS2) | _BV(ADTS0); /* Auto-trigger source select: "Timer/Counter1 Compare Match B". */
	ADMUX = _BV(REFS0) /* Use AREF pin (VCC by default) as reference voltage. */
#if defined(SAMPLE_MODE_U8) && !defined(U8_AMPLIFY_X2)
	      | _BV(ADLAR) /* Left adjust ADC output so we only need to read ADCH. */
#endif
		  | (0xF & ADC_CHANNEL); /* Select our ADC input channel. */
	// Timer Setup
	TCCR1A = _BV(WGM13) | _BV(WGM12) | _BV(WGM11); /* Set timer 1 on A channel to ICR1 fast PWM. (Required to make channel B fire at the correct speed). */
	TCCR1B = _BV(WGM13) | _BV(WGM12) /* Make timer 1 on B channel compare to ICR1 in CTC (Clear Timer on Compare match) mode. */
	       | _BV(CS10); /* Set timer prescaler division factor to 1. */
	ICR1 = TIMER_COMPARE; /* Set timer compare value: freqency = CPU frequency (16MHz) / TIMER_COMPARE. */
	TIMSK1 = _BV(OCIE1A)  /* Use interrupt A for updating the data on the SD card. */
	       | _BV(OCIE1B); /* Enable "Output Compare B Match Interrupt". */
}


void loop() {
	delay(2000);
	wdt_reset(); /* Reset watchdog timer. */
#ifdef DEBUG_RECORDING
	dbg(F("n=%lu\tavg=%lu\tmin=%d\tmax=%d\n"), dbg_total, dbg_sum / (dbg_samples ? dbg_samples : 1), dbg_min, dbg_max);
	dbg_sum = 0;
	dbg_samples = 0;
	dbg_min = 32767;
	dbg_max = -32768;
#endif
	info(F("samples: written=%lu, hanging=%lu, dropped=%lu\n"), samples_written, samples_hanging, samples_dropped);
}
