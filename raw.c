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
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "pilight.h"
#include "protocol.h"
#include "operator.h"
#include "action.h"
#include "common.h"
#include "config.h"
#include "hardware.h"
#include "log.h"
#include "datetime.h"
#include "ssdp.h"
#include "socket.h"
#include "wiringX.h"
#include "threads.h"
#include "irq.h"
#include "dso.h"
#include "gc.h"

static unsigned short main_loop = 1;

int main_gc(void) {
	log_shell_disable();
	main_loop = 0;

	datetime_gc();
	ssdp_gc();
	event_operator_gc();
	event_action_gc();
	options_gc();
	socket_gc();

	config_gc();
	protocol_gc();
	whitelist_free();
	threads_gc();

#ifndef _WIN32
	wiringXGC();
#endif
	dso_gc();
	log_gc();
	gc_clear();

	FREE(progname);
	xfree();

#ifdef _WIN32
	WSACleanup();
#endif

	return EXIT_SUCCESS;
}

void *receiveOOK(void *param) {
	int duration = 0;

	struct hardware_t *hw = (hardware_t *)param;
	while(main_loop && hw->receiveOOK) {
		duration = hw->receiveOOK();
		if(duration > 0) {
			printf("%s: %d\n", hw->id, duration);
		}
	};
	return NULL;
}

void *receivePulseTrain(void *param) {
	struct rawcode_t r;
	int i = 0;

	struct hardware_t *hw = (hardware_t *)param;
	while(main_loop && hw->receivePulseTrain) {
		hw->receivePulseTrain(&r);
		if(r.length == -1) {
			main_gc();
			break;
		} else if(r.length > 0) {
			for(i=0;i<r.length;i++) {
				printf("%s: %d\n", hw->id, r.pulses[i]);
			}
		}
	};
	return NULL;
}

int main(int argc, char **argv) {
	// memtrack();

	atomicinit();
	struct options_t *options = NULL;
	char *args = NULL;
	char *configtmp = MALLOC(strlen(CONFIG_FILE)+1);
	pid_t pid = 0;

	strcpy(configtmp, CONFIG_FILE);

	gc_attach(main_gc);

	/* Catch all exit signals for gc */
	gc_catch();

	log_shell_enable();
	log_file_disable();
	log_level_set(LOG_NOTICE);

	wiringXLog = logprintf;

	if(!(progname = MALLOC(12))) {
		logprintf(LOG_ERR, "out of memory");
		exit(EXIT_FAILURE);
	}
	strcpy(progname, "pilight-raw");

	options_add(&options, 'H', "help", OPTION_NO_VALUE, 0, JSON_NULL, NULL, NULL);
	options_add(&options, 'V', "version", OPTION_NO_VALUE, 0, JSON_NULL, NULL, NULL);
	options_add(&options, 'C', "config", OPTION_HAS_VALUE, 0, JSON_NULL, NULL, NULL);

	while (1) {
		int c;
		c = options_parse(&options, argc, argv, 1, &args);
		if(c == -1)
			break;
		if(c == -2)
			c = 'H';
		switch (c) {
			case 'H':
				printf("Usage: %s [options]\n", progname);
				printf("\t -H --help\t\tdisplay usage summary\n");
				printf("\t -V --version\t\tdisplay version\n");
				printf("\t -C --config\t\tconfig file\n");
				goto close;
			break;
			case 'V':
				printf("%s v%s\n", progname, PILIGHT_VERSION);
				goto close;
			break;
			case 'C':
				configtmp = REALLOC(configtmp, strlen(args)+1);
				strcpy(configtmp, args);
			break;
			default:
				printf("Usage: %s [options]\n", progname);
				goto close;
			break;
		}
	}
	options_delete(options);

#ifdef _WIN32
	if((pid = check_instances(L"pilight-raw")) != -1) {
		logprintf(LOG_ERR, "pilight-raw is already running");
		goto close;
	}
#endif

	if((pid = isrunning("pilight-daemon")) != -1) {
		logprintf(LOG_ERR, "pilight-daemon instance found (%d)", (int)pid);
		goto close;
	}

	if((pid = isrunning("pilight-debug")) != -1) {
		logprintf(LOG_ERR, "pilight-debug instance found (%d)", (int)pid);
		goto close;
	}

	if(config_set_file(configtmp) == EXIT_FAILURE) {
		FREE(configtmp);
		return EXIT_FAILURE;
	}

	protocol_init();
	config_init();
	if(config_read() != EXIT_SUCCESS) {
		FREE(configtmp);
		goto close;
	}
	FREE(configtmp);

	/* Start threads library that keeps track of all threads used */
	threads_start();

	struct conf_hardware_t *tmp_confhw = conf_hardware;
	while(tmp_confhw) {
		if(tmp_confhw->hardware->init) {
			if(tmp_confhw->hardware->init() == EXIT_FAILURE) {
				logprintf(LOG_ERR, "could not initialize %s hardware mode", tmp_confhw->hardware->id);
				goto close;
			}
			if(tmp_confhw->hardware->comtype == COMOOK) {
				threads_register(tmp_confhw->hardware->id, &receiveOOK, (void *)tmp_confhw->hardware, 0);
			} else if(tmp_confhw->hardware->comtype == COMPLSTRAIN) {
				threads_register(tmp_confhw->hardware->id, &receivePulseTrain, (void *)tmp_confhw->hardware, 0);
			}
		}
		tmp_confhw = tmp_confhw->next;
	}

	while(main_loop) {
		sleep(1);
	}

close:
	if(args != NULL) {
		FREE(args);
	}
	if(main_loop == 1) {
		main_gc();
	}
	return (EXIT_FAILURE);
}
