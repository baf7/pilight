/*
	Copyright (C) 2013 - 2014 CurlyMo

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../core/pilight.h"
#include "../core/common.h"
#include "../core/dso.h"
#include "../core/log.h"
#include "../core/json.h"
#include "../core/irq.h"
#include "../config/hardware.h"
#include "../../wiringx/wiringX.h"
#include "315gpio.h"

static int gpio_315_in = 0;
static int gpio_315_out = 0;

static unsigned short gpio315HwInit(void) {
	if(wiringXSupported() == 0) {
		if(wiringXSetup() == -1) {
			return EXIT_FAILURE;
		}
		if(gpio_315_out >= 0) {
			if(wiringXValidGPIO(gpio_315_out) != 0) {
				logprintf(LOG_ERR, "invalid 315sender pin: %d", gpio_315_out);
				return EXIT_FAILURE;
			}
			pinMode(gpio_315_out, OUTPUT);
		}
		if(gpio_315_in >= 0) {
			if(wiringXValidGPIO(gpio_315_in) != 0) {
				logprintf(LOG_ERR, "invalid 315receiver pin: %d", gpio_315_in);
				return EXIT_FAILURE;
			}
			if(wiringXISR(gpio_315_in, INT_EDGE_BOTH) < 0) {
				logprintf(LOG_ERR, "unable to register 315interrupt for pin %d", gpio_315_in);
				return EXIT_SUCCESS;
			}
		}
		return EXIT_SUCCESS;
	} else {
		logprintf(LOG_ERR, "the 315gpio module is not supported on this hardware", gpio_315_in);
		return EXIT_FAILURE;
	}
}

static unsigned short gpio315HwDeinit(void) {
	return EXIT_SUCCESS;
}

static int gpio315Send(int *code, int rawlen, int repeats) {
	int r = 0, x = 0;
	if(gpio_315_out >= 0) {
		for(r=0;r<repeats;r++) {
			for(x=0;x<rawlen;x+=2) {
				digitalWrite(gpio_315_out, 1);
				usleep((__useconds_t)code[x]);
				digitalWrite(gpio_315_out, 0);
				if(x+1 < rawlen) {
					usleep((__useconds_t)code[x+1]);
				}
			}
		}
		digitalWrite(gpio_315_out, 0);
	} else {
		sleep(1);
	}
	return EXIT_SUCCESS;
}

static int gpio315Receive(void) {
	if(gpio_315_in >= 0) {
		return irq_read(gpio_315_in);
	} else {
		sleep(1);
		return 0;
	}
}

static unsigned short gpio315Settings(JsonNode *json) {
	if(strcmp(json->key, "receiver") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_315_in = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	}
	if(strcmp(json->key, "sender") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_315_out = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void gpio315Init(void) {
	hardware_register(&gpio315);
	hardware_set_id(gpio315, "315gpio");

	options_add(&gpio315->options, 'r', "receiver", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");
	options_add(&gpio315->options, 's', "sender", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");

	gpio315->minrawlen = 1000;
	gpio315->maxrawlen = 0;
	gpio315->mingaplen = 5100;
	gpio315->maxgaplen = 10000;
	
	gpio315->hwtype=RF315;
	gpio315->comtype=COMOOK;
	gpio315->init=&gpio315HwInit;
	gpio315->deinit=&gpio315HwDeinit;
	gpio315->sendOOK=&gpio315Send;
	gpio315->receiveOOK=&gpio315Receive;
	gpio315->settings=&gpio315Settings;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "315gpio";
	module->version = "0.1";
	module->reqversion = "7.0";
	module->reqcommit = "10";
}

void init(void) {
	gpio315Init();
}
#endif
