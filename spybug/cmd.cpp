// Copyright 2022 Darwin Schuppan <darwin@nobrain.org>
// SPDX license identifier: MIT

#include "cmd.hh"

#include <Arduino.h>
#include <SD.h>

#include "aaa_config.hh"
#include "fstr.hh"
#include "io.hh"
#include "settings.hh"
#include "sys.hh"

static bool sd_initialized = false;

static void try_init_sd() {
	if (sd_initialized)
		return;
	printf(F("Initializing SD card..."));
	#ifdef PIN_COMPONENT_SWITCH
		digitalWrite(PIN_COMPONENT_SWITCH, COMPONENT_SWITCH_ON);
		delay(500); /* Wait for components to initialize. */
	#endif
	if (SD.begin(PIN_SS)) {
		printf(F("Done.\n"));
		sd_initialized = true;
	} else
		printf(F("\nError initializing SD card.\n"));
}

static void sd_error() {
	printf(F("Error reading from SD card.\n"));
	sd_initialized = false;
}

static void command_loop() {
	// Process Commands
	if (Serial.available()) {
		char args[6][20];
		size_t len = 0;
		size_t n_args = 0;
		int c = getchar();
		while (c != '\n' && n_args < 4) {
			if (c == ' ')
				c = getchar();
			else {
				do {
					args[n_args][len++] = c;
					c = getchar();
				} while (c != ' ' && c != '\n' && len < 20-1);
				args[n_args][len] = 0;
				len = 0;
				n_args++;
			}
		}
		if (n_args >= 1) {
			if (fstreq(args[0], F("set"))) {
				if ((n_args == 3 || n_args == 4) && fstreq(args[1], F("wait"))) {
					float n = atof(args[2]);
					unsigned long mins;
					if (n_args == 4 && (fstreq(args[3], F("hours")) || fstreq(args[3], F("hour"))))
						mins = 60.f * n;
					else
						mins = n;
					settings.recording_delay = mins;
					settings.save();
					printf(F("Set waiting time to %lu minutes or "), settings.recording_delay);
					print_special((float)settings.recording_delay / 60.f);
					printf(F(" hours.\n"));
				} else if (n_args == 3 && fstreq(args[1], F("serial"))) {
					if (fstreq(args[2], F("on"))) {
						settings.serial_log = true;
						settings.save();
						printf(F("Serial log enabled.\n"));
					} else if (fstreq(args[2], F("off"))) {
						settings.serial_log = false;
						settings.save();
						printf(F("Serial log disabled.\n"));
					} else {
						printf(F("Usage: 'set serial [on|off]'.\n"));
					}
				} else {
					printf(F("Usage: 'set wait <number> [minutes|hours]' or 'set serial [on|off]'.\n"));
				}
			} else if (fstreq(args[0], F("get"))) {
				if (n_args == 2 && fstreq(args[1], F("wait"))) {
					printf(F("Current waiting time: %lu minutes or "), settings.recording_delay);
					print_special((float)settings.recording_delay / 60.f);
					printf(F(" hours.\n"));
				} else {
					printf(F("Usage: 'get wait'.\n"));
				}
			} else if (fstreq(args[0], F("sd"))) {
				try_init_sd();
				if (sd_initialized) {
					if (n_args == 2 && fstreq(args[1], F("list"))) {
						File root = SD.open("/");
						if (root) {
							printf(F("Files:\n"));
							bool files = false;
							size_t cols = 0;
							while (1) {
								File entry = root.openNextFile();
								if (!entry)
									break;
								files = true;
								if (cols >= 3) {
									cols = 0;
									printf(F("\n"));
								}
								printf(F("  %s"), entry.name());
								cols++;
								entry.close();
							}
							if (!files)
								printf(F("[NONE]"));
							printf(F("\n"));
						} else
							sd_error();
						root.close();
					} else if (n_args == 3 && fstreq(args[1], F("remove"))) {
						if (SD.remove(args[2]))
							printf(F("Deleted '%s'.\n"), args[2]);
						else
							printf(F("Error removing '%s'.\n"), args[2]);
					} else if (n_args == 2 && fstreq(args[1], F("remove_all"))) {
						File root = SD.open("/");
						if (root) {
							printf(F("Deleted:\n"));
							bool deleted = false;
							size_t cols = 0;
							while (1) {
								File entry = root.openNextFile();
								if (!entry)
									break;
								const char *name = entry.name();
								unsigned int garbage;
								if (!entry.isDirectory() && sscanf(name, REC_FILE_FMT, &garbage) == 1) {
									if (cols >= 3) {
										cols = 0;
										printf(F("\n"));
									}
									deleted = true;
									SD.remove(name);
									printf(F("  %s"), name);
									cols++;
								}
								entry.close();
							}
							if (!deleted)
								printf(F("[NONE]"));
							printf(F("\n"));
						} else
							sd_error();
						root.close();
					} else if (n_args != 1) {
						printf(F("Usage: 'sd list', 'sd remove <filename>' or 'sd remove_all'.\n"));
					}
				}
			} else if (fstreq(args[0], F("help"))) {
				printf(F("Commands:\n"));
				printf(F("  help                               --  Display this page.\n"));
				printf(F("  exit                               --  Leave command mode.\n"));
				printf(F("  get wait                           --  Display current delay setting.\n"));
				printf(F("  set wait <number> [minutes|hours]  --  Change current delay setting.\n"));
				printf(F("  set serial [on|off]                --  Write log to serial output.\n"));
				printf(F("  sd                                 --  Initialize the SD card.\n"));
				printf(F("  sd list                            --  List file on SD card.\n"));
				printf(F("  sd remove <filename>               --  Delete a file from SD card.\n"));
				printf(F("  sd remove_all                      --  Delete all recordings from SD card.\n"));
			} else if (fstreq(args[0], F("exit"))) {
				printf(F("Bye!\n"));
				Serial.flush();
				full_reset();
			} else
				printf(F("Invalid command: '%s'. Type 'help' for a list of commands.\n"), args[0]);
		} else {
			printf(F("Please specify a command. Type 'help' for a list of commands.\n"));
		}
	}
}

void cmd() {
	printf(F("You are now in command mode. Reset to exit. Type 'help' for a list of commands.\n"));
	while (Serial.available()) Serial.read();
	while (1) {
		command_loop();
		delay(50);
	}
}
