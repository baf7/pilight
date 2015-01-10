/*
	Copyright (c) 2014 CurlyMo <curlymoo1@gmail.com>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>

#include "log.h"

#include "wiringX.h"
#include "hummingboard.h"
#include "raspberrypi.h"
#include "bananapi.h"

static struct platform_t *platform = NULL;
static int setup = -2;

/* Both the delayMicroseconds and the delayMicrosecondsHard
   are taken from wiringPi */
static void delayMicrosecondsHard(unsigned int howLong) {
	struct timeval tNow, tLong, tEnd ;

	gettimeofday(&tNow, NULL);
	tLong.tv_sec  = (__time_t)howLong / 1000000;
	tLong.tv_usec = (__suseconds_t)howLong % 1000000;
	timeradd(&tNow, &tLong, &tEnd);

	while(timercmp(&tNow, &tEnd, <))
		gettimeofday(&tNow, NULL);
}

void delayMicroseconds(unsigned int howLong) {
	struct timespec sleeper;
	long int uSecs = (__time_t)howLong % 1000000;
	unsigned int wSecs = howLong / 1000000;

	if(howLong == 0) {
		return;
	} else if(howLong  < 100) {
		delayMicrosecondsHard(howLong);
	} else {
		sleeper.tv_sec  =(__time_t)wSecs;
		sleeper.tv_nsec = (long)(uSecs * 1000L);
		nanosleep(&sleeper, NULL);
	}
}

void platform_register(struct platform_t **dev, const char *name) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	*dev = malloc(sizeof(struct platform_t));
	(*dev)->name = NULL;
	(*dev)->pinMode = NULL;
	(*dev)->digitalWrite = NULL;
	(*dev)->digitalRead = NULL;
	(*dev)->identify = NULL;
	(*dev)->waitForInterrupt = NULL;
	(*dev)->isr = NULL;
	(*dev)->I2CRead = NULL;
	(*dev)->I2CReadReg8 = NULL;
	(*dev)->I2CReadReg16 = NULL;
	(*dev)->I2CWrite = NULL;
	(*dev)->I2CWriteReg8 = NULL;
	(*dev)->I2CWriteReg16 = NULL;

	if(!((*dev)->name = malloc(strlen(name)+1))) {
		logprintf(LOG_ERR, "out of memory");
		exit(0);
	}
	strcpy((*dev)->name, name);
	(*dev)->next = platforms;
	platforms = (*dev);
}

int wiringXGC(void) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	int i = 0;
	if(platform) {
		i = platform->gc();
	}
	platform = NULL;
	struct platform_t *tmp = platforms;
	while(platforms) {
		tmp = platforms;
		free(platforms->name);
		platforms = platforms->next;
		free(tmp);
	}
	free(platforms);
	logprintf(LOG_DEBUG, "garbage collected wiringX library");
	return i;
}

void pinMode(int pin, int mode) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->pinMode) {
			if(platform->pinMode(pin, mode) == -1) {
				logprintf(LOG_ERR, "%s: error while calling pinMode", platform->name);
				wiringXGC();
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support pinMode", platform->name);
			wiringXGC();
		}
	}
}

void digitalWrite(int pin, int value) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->digitalWrite) {
			if(platform->digitalWrite(pin, value) == -1) {
				logprintf(LOG_ERR, "%s: error while calling digitalWrite", platform->name);
				wiringXGC();
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support digitalWrite", platform->name);
			wiringXGC();
		}
	}
}

int digitalRead(int pin) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->digitalRead) {
			int x = platform->digitalRead(pin);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling digitalRead", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support digitalRead", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int waitForInterrupt(int pin, int ms) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->waitForInterrupt) {
			int x = platform->waitForInterrupt(pin, ms);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling waitForInterrupt", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support waitForInterrupt", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXISR(int pin, int mode) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->isr) {
			int x = platform->isr(pin, mode);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling isr", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support isr", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXI2CRead(int fd) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->I2CRead) {
			int x = platform->I2CRead(fd);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling I2CRead", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support I2CRead", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXI2CReadReg8(int fd, int reg) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->I2CReadReg8) {
			int x = platform->I2CReadReg8(fd, reg);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling I2CReadReg8", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support I2CReadReg8", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXI2CReadReg16(int fd, int reg) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->I2CReadReg16) {
			int x = platform->I2CReadReg16(fd, reg);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling I2CReadReg16", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support I2CReadReg16", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXI2CWrite(int fd, int data) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->I2CWrite) {
			int x = platform->I2CWrite(fd, data);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling I2CWrite", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support I2CWrite", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXI2CWriteReg8(int fd, int reg, int data) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->I2CWriteReg8) {
			int x = platform->I2CWriteReg8(fd, reg, data);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling I2CWriteReg8", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support I2CWriteReg8", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXI2CWriteReg16(int fd, int reg, int data) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->I2CWriteReg16) {
			int x = platform->I2CWriteReg16(fd, reg, data);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling I2CWriteReg16", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support I2CWriteReg16", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXI2CSetup(int devId) {
	if(platform) {
		if(platform->I2CSetup) {
			int x = platform->I2CSetup(devId);
			if(x == -1) {
				logprintf(LOG_ERR, "%s: error while calling I2CSetup", platform->name);
				wiringXGC();
			} else {
				return x;
			}
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support I2CSetup", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

char *wiringXPlatform(void) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	return platform->name;
}

int wiringXValidGPIO(int gpio) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

	if(platform) {
		if(platform->validGPIO) {
			return platform->validGPIO(gpio);
		} else {
			logprintf(LOG_ERR, "%s: platform doesn't support gpio number validation", platform->name);
			wiringXGC();
		}
	}
	return -1;
}

int wiringXSetup(void) {
	logprintf(LOG_STACK, "%s(...)", __FUNCTION__);

#ifndef __FreeBSD__
	if(setup == -2) {
		hummingboardInit();
		raspberrypiInit();
		bananapiInit();

		int match = 0;
		struct platform_t *tmp = platforms;
		while(tmp) {
			if(tmp->identify() >= 0) {
				platform = tmp;
				match = 1;
				break;
			}
			tmp = tmp->next;
		}

		if(match == 0) {
			logprintf(LOG_ERR, "hardware not supported");
			wiringXGC();
			return -1;
		} else {
			logprintf(LOG_DEBUG, "running on a %s", platform->name);
		}
		setup = platform->setup();
		return setup;
	} else {
		return setup;
	}
#endif
	return 0;
}
