# Arduino Spy Bug

A simple voice recorder for Arduino using an SD card and an electret microphone amplifier circuit.

Independent of any third-party libraries.

## Wiring (also documented in spybug/spybug.ino)
### SD Card Wiring
SD       | Nano
---------|------------
D0  (DO) |  D12 (MISO)
VSS      |  GND
CLK      |  D13 (SCK)
VDD      |  5V or 3V3
CMD (DI) |  D11 (MOSI)
D3  (CS) |  D10 (SS)

WARNING: SD cards are not designed for 5V; I have been using 5V anyways
and everything seems fine, but beware that there is a significant risk
of immediate or premature failure when not using a buffer circuit.

SD pin D3 is the chip select pin. It can be set manually in `PIN_SS`.

### Microphone Wiring (MAX9814 w/ electret microphone)
Mic      | Nano
---------|------------
VCC      |  5V
GND      |  GND
Out      |  A0

Out defaults to A0 (AdcChannel0), but can be set manually in `ADC_CHANNEL`.
