/*
	Copyright (C) 2013 - 2016 CurlyMo & Niek

	This Source Code Form is subject to the terms of the Mozilla Public
	License, v. 2.0. If a copy of the MPL was not distributed with this
	file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../operator.h"
#include "../../core/dso.h"
#include "isnot.h"

static void operatorIsnotCallback(char *a, char *b, char **ret) {
	if(strcmp(a, b) != 0) {
		strcpy(*ret, "1");
	} else {
		strcpy(*ret, "0");
	}
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void operatorIsnotInit(void) {
	event_operator_register(&operator_isnot, "ISNOT");
	operator_isnot->callback_string = &operatorIsnotCallback;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "ISNOT";
	module->version = "1.0";
	module->reqversion = "5.0";
	module->reqcommit = "87";
}

void init(void) {
	operatorIsnotInit();
}
#endif
