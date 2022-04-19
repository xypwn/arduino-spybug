#pragma once

/************************
 BEGIN USER CONFIGURATION
 ************************/
//#define DEBUG_RECORDING

#define PIN_COMPONENT_SWITCH 2 /* Use a digital signal to switch on/off the microphone and SD card for less power draw. */
#define COMPONENT_SWITCH_ON HIGH

#define SAMPLE_MODE_U8
//#define SAMPLE_MODE_S16

//#define ADC_PRESCALE_16 /* Up to ~60kHz. */
//#define ADC_PRESCALE_32 /* Up to ~27kHz. */
#define ADC_PRESCALE_64 /* Up to ~18kHz. */

//#define U8_AMPLIFY_X2 /* (U8 sampling mode only) amplify audio by factor 2. */

#define ADC_CHANNEL AdcChannel0
#define TIMER_COMPARE 1000 /* 16MHz / 1000 = 16kHz. */
#define FLUSH_SAMPLES 64000 /* Flush WAV file every n samples. */
#define PIN_SS 10
/**********************
 END USER CONFIGURATION
 **********************/
