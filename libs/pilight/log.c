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
#include <errno.h>
#include <syslog.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <libgen.h>

#include "../../pilight.h"
#include "common.h"
#include "gc.h"
#include "log.h"

static FILE *lf=NULL;

static char *logfile = NULL;
static char *logpath = NULL;
static int filelog = 1;
static int shelllog = 0;
static int loglevel = LOG_DEBUG;
static char debug_log[128];

int log_gc(void) {
	if(shelllog == 1) {
		fprintf(stderr, "DEBUG: garbage collected log library\n");
	}
	if(lf) {
		if(fclose(lf) != 0) {
			return 0;
		}
		else {
			lf = NULL;
		}
	}
	if(logfile) {
		sfree((void *)&logfile);
	}
	if(logpath) {
		sfree((void *)&logpath);
	}
	return 1;
}

void logprintf(int prio, const char *format_str, ...) {
	int save_errno = errno;
	char line[1024];
	va_list ap;
	struct stat sb;
	char fmt[64], buf[64];
	struct timeval tv;
	struct tm *tm;
	int restore_shell = 0;
	int restore_file = 0;

	if(logfile == NULL) {
		if(shelllog == 0) {
			restore_shell = 1;
		}
		shelllog = 1;
		if(filelog == 1) {
			restore_file = 1;
		}
		filelog = 0;
	}

	if(loglevel >= prio) {
		gettimeofday(&tv, NULL);
		if((tm = localtime(&tv.tv_sec)) != NULL) {
			strftime(fmt, sizeof(fmt), "%b %d %H:%M:%S", tm);
			snprintf(buf, sizeof(buf), "%s:%03u", fmt, (unsigned int)tv.tv_usec);
		}

		sprintf(debug_log, "[%22.22s] %s: ", buf, progname);

		if(filelog == 1 && prio < LOG_DEBUG) {
			memset(line, '\0', 1024);
			strcat(line, debug_log);
			va_start(ap, format_str);
			if(prio==LOG_WARNING)
				strcat(line, "WARNING: ");
			if(prio==LOG_ERR)
				strcat(line, "ERROR: ");
			if(prio==LOG_INFO)
				strcat(line, "INFO: ");
			if(prio==LOG_NOTICE)
				strcat(line, "NOTICE: ");
			if(prio==LOG_DEBUG)
				strcat(line, "DEBUG: ");
			if(prio==LOG_STACK)
				strcat(line, "STACK: ");
			vsprintf(&line[strlen(line)], format_str, ap);
			strcat(line, "\n");

			if((stat(logfile, &sb)) >= 0) {
				if(!(lf = fopen(logfile, "a"))) {
					filelog = 0;
				}
			} else {
				if(sb.st_nlink == 0) {
					if(!(lf = fopen(logfile, "a"))) {
						filelog = 0;
					}
				}
				if(sb.st_size > LOG_MAX_SIZE) {
					fclose(lf);
					char tmp[strlen(logfile)+5];
					strcpy(tmp, logfile);
					strcat(tmp, ".old");
					rename(logfile, tmp);
					if(!(lf = fopen(logfile, "a"))) {
						filelog = 0;
					}
				}
			}
			if(lf) {
				fwrite(line, sizeof(char), strlen(line), lf);
				fflush(lf);
				fclose(lf);
				lf = NULL;
			}
			va_end(ap);
		}

		if(shelllog == 1) {
			fputs(debug_log, stderr);
			va_start(ap, format_str);
			if(prio==LOG_WARNING)
				fprintf(stderr, "WARNING: ");
			if(prio==LOG_ERR)
				fprintf(stderr, "ERROR: ");
			if(prio==LOG_INFO)
				fprintf(stderr, "INFO: ");
			if(prio==LOG_NOTICE)
				fprintf(stderr, "NOTICE: ");
			if(prio==LOG_DEBUG)
				fprintf(stderr, "DEBUG: ");
			if(prio==LOG_STACK)
				fprintf(stderr, "STACK: ");
			vfprintf(stderr, format_str, ap);
			fputc('\n', stderr);
			fflush(stderr);
			va_end(ap);
		}
	}
	errno = save_errno;
	if(restore_shell) {
		shelllog = 0;
	}
	if(restore_file) {
		filelog = 1;
	}
}

void logperror(int prio, const char *s) {
	// int save_errno = errno;
	// if(logging == 0)
		// return;

	// if(s != NULL) {
		// logprintf(prio, "%s: %s", s, strerror(errno));
	// } else {
		// logprintf(prio, "%s", strerror(errno));
	// }
	// errno = save_errno;
}

void log_file_enable(void) {
	filelog = 1;
}

void log_file_disable(void) {
	filelog = 0;
}

void log_shell_enable(void) {
	shelllog = 1;
}

void log_shell_disable(void) {
	shelllog = 0;
}

void log_file_set(char *log) {
	struct stat s;
	struct stat sb;
	char *filename = basename(log);
	size_t i = (strlen(log)-strlen(filename));
	logpath = realloc(logpath, i+1);
	memset(logpath, '\0', i+1);
	strncpy(logpath, log, i);

	if(strcmp(filename, log) != 0) {
		int err = stat(logpath, &s);
		if(err == -1) {
			if(ENOENT == errno) {
				logprintf(LOG_ERR, "the log folder %s does not exist", logpath);
				sfree((void *)&logpath);
				exit(EXIT_FAILURE);
			} else {
				logprintf(LOG_ERR, "failed to run stat on log folder %s", logpath);
				sfree((void *)&logpath);
				exit(EXIT_FAILURE);
			}
		} else {
			if(S_ISDIR(s.st_mode)) {
				logfile = realloc(logfile, strlen(log)+1);
				strcpy(logfile, log);
			} else {
				logprintf(LOG_ERR, "the log folder %s does not exist", logpath);
				sfree((void *)&logpath);
				exit(EXIT_FAILURE);
			}
		}
	} else {
		logfile = realloc(logfile, strlen(log)+1);
		strcpy(logfile, log);
	}

	char tmp[strlen(logfile)+5];
	strcpy(tmp, logfile);
	strcat(tmp, ".old");

	if((stat(tmp, &sb)) == 0) {
		if(sb.st_nlink > 0) {
			if((stat(logfile, &sb)) == 0) {
				if(sb.st_nlink > 0) {
					remove(tmp);
					rename(logfile, tmp);
				}
			}
		}
	}

	if(lf == NULL && filelog == 1) {
		if((lf = fopen(logfile, "a")) == NULL) {
			filelog = 0;
			shelllog = 1;
			logprintf(LOG_ERR, "could not open logfile %s", logfile);
			sfree((void *)&logpath);
			sfree((void *)&logfile);
			exit(EXIT_FAILURE);
		} else {
			fclose(lf);
			lf = NULL;
		}
	}

	sfree((void *)&logpath);
}

void log_level_set(int level) {
	loglevel = level;
}

int log_level_get(void) {
	return loglevel;
}

void logerror(const char *format_str, ...) {
	char line[1024];
	va_list ap;
	struct stat sb;
	FILE *f = NULL;
	char fmt[64], buf[64];
	struct timeval tv;
	struct tm *tm;
	char date[128];

	memset(line, '\0', 1024);
	gettimeofday(&tv, NULL);
	if((tm = localtime(&tv.tv_sec)) != NULL) {
		strftime(fmt, sizeof(fmt), "%b %d %H:%M:%S", tm);
		snprintf(buf, sizeof(buf), "%s:%03u", fmt, (unsigned int)tv.tv_usec);
	}

	sprintf(date, "[%22.22s] %s: ", buf, progname);
	strcat(line, date);
	va_start(ap, format_str);
	vsprintf(&line[strlen(line)], format_str, ap);
	strcat(line, "\n");

	if((stat("/var/log/pilight.err", &sb)) >= 0) {
		if(!(f = fopen("/var/log/pilight.err", "a"))) {
			return;
		}
	} else {
		if(sb.st_nlink == 0) {
			if(!(f = fopen("/var/log/pilight.err", "a"))) {
				return;
			}
		}
		if(sb.st_size > LOG_MAX_SIZE) {
			fclose(f);
			char tmp[strlen("/var/log/pilight.err")+5];
			strcpy(tmp, "/var/log/pilight.err");
			strcat(tmp, ".old");
			rename("/var/log/pilight.err", tmp);
			if(!(f = fopen("/var/log/pilight.err", "a"))) {
				return;
			}
		}
	}
	if(f) {
		fwrite(line, sizeof(char), strlen(line), f);
		fflush(f);
		fclose(f);
		f = NULL;
	}
	va_end(ap);
}
