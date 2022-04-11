/* Wiring:
	SD       | Nano
	______________________
	D0  (DO) |  D12 (MISO)
	VSS      |  GND
	CLK      |  D13 (SCK)
	VDD      |  3V3
	CMD (DI) |  D11 (MOSI)
	D3  (CS) |  D10 (SS)
	
	SD pin D3 is the chip select pin (must be set manually in PIN_SS).
*/

#include <SD.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>

/************************
 BEGIN USER CONFIGURATION
 ************************/
//#define DEBUG_RECORDING
//#define SERIAL_OUTPUT

#define SAMPLE_MODE_U8
//#define SAMPLE_MODE_S16

//#define ADC_PRESCALE_16 /* Up to ~60kHz. */
//#define ADC_PRESCALE_32 /* Up to ~27kHz. */
#define ADC_PRESCALE_64 /* Up to ~18kHz. */

#define U8_EXTRA_PRECISION /* (U8 sampling mode only) use 9th ADC reading bit and chop off 1st bit for more precision (sacrificing half of the bandwidth) */

#define RECORDING_DELAY_IN_MINUTES 0 /* Wait n minutes before starting to record. */
#define ADC_CHANNEL AdcChannel0
#define TIMER_COMPARE 1000 /* 16MHz / 1000 = 16kHz. */
#define FLUSH_SAMPLES 64000 /* Flush WAV file every n samples. */
#define PIN_SS 10
/**********************
 END USER CONFIGURATION
 **********************/

#ifdef SERIAL_OUTPUT
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

static void setup_serial_in_out() {
	fdev_setup_stream(&serial_in_out, serial_putch, serial_getch, _FDEV_SETUP_RW);
	stdout = stdin = stderr = &serial_in_out;
}

static size_t fstrlen(const __FlashStringHelper *s) {
	PGM_P sp = (PGM_P)s;
	size_t len = 0;
	while (pgm_read_byte(sp++))
		len++;
	return len;
}

static int printf(const __FlashStringHelper *fmt, ...) {
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

#define die(fmt, ...) { disable_recording_interrupts(); printf(F("Fatal: ")); printf(fmt, ##__VA_ARGS__); Serial.flush(); while(1); }
#define dbg(fmt, ...) { printf(F("Debug: ")); printf(fmt, ##__VA_ARGS__); }
#define print_special(x) Serial.print(x)
#else
#define printf(fmt, ...) {}
#define die(fmt, ...) { disable_recording_interrupts(); while(1); }
#define dbg(fmt, ...) {}
#define print_special(x) {}
#endif

#if defined(RECORDING_DELAY_IN_MINUTES) && RECORDING_DELAY_IN_MINUTES != 0
#include <LowPower.h> /* https://github.com/rocketscream/Low-Power */
static void low_power_sleep_minutes(unsigned long t) {
	for (unsigned long i = 0; 8ul * i < 60ul * t; i++) {
		/* Power down for 8s. */
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	}
}
#endif

static void start_watchdog_with_full_reset() {
	MCUSR &= ~B00001000; /* Clear reset flag. */
	WDTCSR |= B00011000; /* Prepare prescaler change. */
	WDTCSR = B00100001; /* Set watchdog timeout to 8s. */
	// Enable Watchdog Timer
	WDTCSR |= B01000000;
	MCUSR = MCUSR & B11110111;
}

static inline void disable_recording_interrupts() {
	TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B));
}

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
			printf(F("Lost "));
			print_special((float)samples_hanging / (float)(F_CPU / TIMER_COMPARE)); /* Printf doesn't handle floats. */
			printf(F(" seconds of recording.\n"));
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
#ifdef U8_EXTRA_PRECISION
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
#ifdef SERIAL_OUTPUT
	Serial.begin(9600); /* Set baud rate. */
	setup_serial_in_out(); /* Add printf support. */
#endif
	// Delay Triggering
#if defined(RECORDING_DELAY_IN_MINUTES) && RECORDING_DELAY_IN_MINUTES != 0
	printf(F("Waiting %u minute%s before starting to record...\n"), RECORDING_DELAY_IN_MINUTES, RECORDING_DELAY_IN_MINUTES == 1 ? "" : "s");
	Serial.flush();
	low_power_sleep_minutes(RECORDING_DELAY_IN_MINUTES); /* Draws ~12.5mA instead of ~30mA when using delay(). */
#endif
	// Start Watchdog (wdt_enable() doesn't fully reset)
	start_watchdog_with_full_reset();
	// SD Card Setup
	if (!SD.begin(PIN_SS))
		die(F("Error initializing SD card!\n"));
	// Determine Filename
	unsigned int filenum = 0;
	char filename[32];
	do {
		filenum++;
		snprintf(filename, 32, "rec_%03u.wav", filenum);
	} while (SD.exists(filename));
	// Open File
	file = SD.open(filename, O_READ | O_WRITE | O_CREAT); /* Seeking doesn't seem to work with FILE_WRITE?! */
	printf(F("Recording to file '%s'.\n"), filename);
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
#if defined(SAMPLE_MODE_U8) && !defined(U8_EXTRA_PRECISION)
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
	delay(1000);
	wdt_reset(); /* Reset watchdog timer. */
#ifdef DEBUG_RECORDING
	dbg(F("n=%lu\tavg=%lu\tmin=%d\tmax=%d\n"), dbg_total, dbg_sum / (dbg_samples ? dbg_samples : 1), dbg_min, dbg_max);
	dbg_sum = 0;
	dbg_samples = 0;
	dbg_min = 32767;
	dbg_max = -32768;
#endif
	printf(F("samples: written=%lu, hanging=%lu, dropped=%lu\n"), samples_written, samples_hanging, samples_dropped);
}
