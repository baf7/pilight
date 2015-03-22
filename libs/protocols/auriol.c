/*
	Copyright (C) 2014 Bram1337 & CurlyMo

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
#include <math.h>

#include "pilight.h"
#include "common.h"
#include "dso.h"
#include "log.h"
#include "protocol.h"
#include "hardware.h"
#include "binary.h"
#include "gc.h"
#include "auriol.h"

typedef struct auriol_settings_t {
	double id;
	double temp;
	struct auriol_settings_t *next;
} auriol_settings_t;

static struct auriol_settings_t *auriol_settings = NULL;

static void auriolParseCode(void) {
	int i = 0, x = 0;
	int channel = 0, id = 0, battery = 0;
	double temp_offset = 0.0, temperature = 0.0;
	for(i=1;i<auriol->rawlen-1;i+=2) {
		auriol->binary[x++] = auriol->code[i];
	}

	// id = binToDecRev(auriol->binary, 0, 7); using channel instead of battery id as id
	battery = auriol->binary[8];
	channel = 1 + binToDecRev(auriol->binary, 10, 11); // channel as id
	temperature = (double)binToDecRev(auriol->binary, 12, 23)/10;
	// checksum = (double)binToDecRev(auriol->binary, 24, 31); been unable to deciper it
	struct auriol_settings_t *tmp = auriol_settings;
	while(tmp) {
		if(fabs(tmp->id-id) < EPSILON) {
			temp_offset = tmp->temp;
			break;
		}
		tmp = tmp->next;
	}

	temperature += temp_offset;

	if (channel != 4) {
		auriol->message = json_mkobject();
		json_append_member(auriol->message, "id", json_mknumber(channel, 0));
		json_append_member(auriol->message, "temperature", json_mknumber(temperature, 1));
		json_append_member(auriol->message, "battery", json_mknumber(battery, 0));
	}
}

static int auriolCheckValues(struct JsonNode *jvalues) {
	struct JsonNode *jid = NULL;

	if((jid = json_find_member(jvalues, "id"))) {
		struct auriol_settings_t *snode = NULL;
		struct JsonNode *jchild = NULL;
		struct JsonNode *jchild1 = NULL;
		double id = -1;
		int match = 0;

		jchild = json_first_child(jid);
		while(jchild) {
			jchild1 = json_first_child(jchild);
			while(jchild1) {
				if(strcmp(jchild1->key, "id") == 0) {
					id = jchild1->number_;
				}
				jchild1 = jchild1->next;
			}
			jchild = jchild->next;
		}

		struct auriol_settings_t *tmp = auriol_settings;
		while(tmp) {
			if(fabs(tmp->id-id) < EPSILON) {
				match = 1;
				break;
			}
			tmp = tmp->next;
		}

		if(!match) {
			if(!(snode = MALLOC(sizeof(struct auriol_settings_t)))) {
				logprintf(LOG_ERR, "out of memory");
				exit(EXIT_FAILURE);
			}
			snode->id = id;
			snode->temp = 0;

			json_find_number(jvalues, "temperature-offset", &snode->temp);

			snode->next = auriol_settings;
			auriol_settings = snode;
		}
	}
	return 0;
}

static void auriolGC(void) {
	struct auriol_settings_t *tmp = NULL;
	while(auriol_settings) {
		tmp = auriol_settings;
		auriol_settings = auriol_settings->next;
		FREE(tmp);
	}
	if(auriol_settings != NULL) {
		FREE(auriol_settings);
	}
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void auriolInit(void) {

	protocol_register(&auriol);
	protocol_set_id(auriol, "auriol");
	protocol_device_add(auriol, "auriol", "Auriol Weather Stations");
	protocol_plslen_add(auriol, 269);
	auriol->devtype = WEATHER;
	auriol->hwtype = RF433;
	auriol->pulse = 15;
	auriol->rawlen = 66;

	options_add(&auriol->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "[1-3]");
	options_add(&auriol->options, 't', "temperature", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
	options_add(&auriol->options, 'b', "battery", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[01]$");

	// options_add(&auriol->options, 0, "decimals", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *)1, "[0-9]");
	options_add(&auriol->options, 0, "temperature-offset", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *)0, "[0-9]");
	options_add(&auriol->options, 0, "decimals", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "[0-9]");
	options_add(&auriol->options, 0, "show-temperature", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");
	options_add(&auriol->options, 0, "show-battery", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");

	auriol->parseCode=&auriolParseCode;
	auriol->checkValues=&auriolCheckValues;
	auriol->gc=&auriolGC;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "auriol";
	module->version = "1.2";
	module->reqversion = "5.0";
	module->reqcommit = "187";
}

void init(void) {
	auriolInit();
}
#endif
