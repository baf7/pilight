/*
	Copyright (C) 2013 - 2016 CurlyMo

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include "../core/pilight.h"
#include "../core/common.h"
#include "../core/dso.h"
#include "../core/log.h"
#include "../core/json.h"
#include "../core/threadpool.h"
#include "../core/eventpool.h"
#include "../hardware/hardware.h"
#include "../../wiringx/wiringX.h"
#include "433gpio.h"

static int gpio_433_in = 0;
static int gpio_433_out = 0;
static int doPause = 0;

typedef struct timestamp_t {
	unsigned long first;
	unsigned long second;
} timestamp_t;

typedef struct data_t {
	int rbuffer[1024];
	int rptr;
	void *(*callback)(void *);
} data_t;

struct timestamp_t timestamp;

#if defined(__arm__) || defined(__mips__)
static void *reason_received_pulsetrain_free(void *param) {
	struct reason_received_pulsetrain_t *data = param;
	FREE(data);
	return NULL;
}

// Definitions for usr_parseHeader
#define WAIT_FOR_END_OF_HEADER	0
#define WAIT_FOR_END_OF_DATA	1
#define WAIT_FOR_END_OF_DATA_2	2
#define WAIT_FOR_END_OF_DATA_2A	3
#define WAIT_FOR_END_OF_DATA_3	4
#define PREAMB_SYNC_L	488	// V2.1 - clk = 1024 Hz
#define PREAMB_SYNC_L_MAX	586	// PREAMB_SYNC_L * 1,2
#define PREAMB_SYNC_MIN	732
#define PREAMB_SYNC_H	976	// V2.1 - clk = 1024 Hz
#define PREAMB_SYNC_MAX	1120	// A single value above this value is added to the next value
#define PREAMB_SYNC_DMAX	PREAMB_SYNC_H+PREAMB_SYNC_L
#define O21_FOOTER	11018	// GAP pulse Oregon V2.1
#define PRE_AMB_HEADER_CNT_MAX	33	// Max # of preamb pulses
#define PRE_AMB_HEADER_CNT	22
#define L_HEADER_21	12

int header_21[L_HEADER_21] = {PREAMB_SYNC_L,PREAMB_SYNC_L,PREAMB_SYNC_H,PREAMB_SYNC_L,PREAMB_SYNC_L,PREAMB_SYNC_H,PREAMB_SYNC_L,PREAMB_SYNC_L,PREAMB_SYNC_H,PREAMB_SYNC_L,PREAMB_SYNC_L,PREAMB_SYNC_H};
int duration_previous = 0;
int flag_oregon_21 = 0;
int latch_duration = 0;
int preamb_pulse_counter = 0;
int preamb_duration = 0;
int preamb_state = 0;
int p_header_21 = 0;
// End definitions for usr_parseHeader

static void usr_parseHeader(struct hardware_t *gpio433, struct data_t *data, int *duration) {
// --------------------------------------------------------------------------------------------------------------------
// Convert OREGON RAW STREAM into compatible pilight format
// Support for pre-amb header/sync detection:
// oregon21: length 32: maximum length of valid pre-amb pulses is 32 (32 clock duration pulses)
//		 SYNC is coded as 10101010 and handled by the protocol itself (possible intergap value is 11018, 31232)
// Planned to be added pre-amb support for
// oregon30: length 24: maximum valid length
//		 SYNC is coded as 0101 and handled ny the protocol itself (possible intergap value 23424)
// Maybe:
// oregon10: length 12: maximum length
//		 SYNC is coded as three pulses and handled by the protocol itself (footer 35088)
// Analyse stream for repetitive pulses with half the duration of the clock frequency 2924µS for V1.0
// Analyse stream for repetitive pulses with the duration of the clock frequency 976µS for V2.1
// Analyse stream for repetitive pulses with half the duration of the clock frequency 976µS for V3.0
//  - they are in V2.1 members of the Pre-Amb sequence
//  - they are uncommon in regular header/footer based streams and other protocols
//  -> Other protocols will never get beyhond the WAIT_FOR_END_OF_HEADER state
//
// oregon_space: 9000µS footer pulses: 500/2000 - ONE, 500/4000 - ZERO
// The followign oregon devices do not require modification in daemon.c
//  - devices: SL-109H, AcuRite 09955
// Marker criteria is the first deviating pulse:
// - distance of pulses between marker is rawlen of pulse stream
// - duration of repetitive pulses is footer
// As pilight will only accept footer values that do not deviate by more than -/+170µS the protocol driver has to
// cover an extended footer value range (two additional protocol_plslen_add function calls with a difference of +/- 11)
// Date: 15/01/15
// --------------------------------------------------------------------------------------------------------------------
// Ignore Footer pulses at the beginning of a pulsestream by not resetting the buffer.
// Allow at least the buffer to fill up with 20 pulses, as the shortest valid payload stream currently known has more
// This may lead to not detecting the 1st pulsetrain properly, but it allows handling of protocols with Header values
// --------------------------------------------------------------------------------------------------------------------

	if(*duration > 0) {
		data->rbuffer[data->rptr] = *duration;
		switch (preamb_state) {

			case WAIT_FOR_END_OF_HEADER:
			flag_oregon_21 = 0;
			if (*duration > PREAMB_SYNC_MIN) {	// Check for pulses with at least clock duration
				preamb_pulse_counter++;
				preamb_duration += data->rbuffer[data->rptr];
			} else {
				// Check if we found the deviating pulse after a series of consecutive pulses
				// and that the deviating pulse qualifies as a SYNC pulse
				if (preamb_pulse_counter > PRE_AMB_HEADER_CNT) {
					flag_oregon_21 = 1;	 // flag for footer based protocols: 2nd transmission
					preamb_state = WAIT_FOR_END_OF_DATA;
					preamb_pulse_counter = 0;
					data->rptr = 0;	// Reset Pointer
					data->rbuffer[data->rptr] = *duration;	// Restore the 1st SYNC pulse
				} else {
					// Below threshold, so we have to reset and will continue to check
					preamb_duration = 0;
					preamb_pulse_counter = 0;
				}
			}
			break;

			case WAIT_FOR_END_OF_DATA:
			if (*duration > 5100) {	// Regular GAP detected search for Header
				if (flag_oregon_21 == 1) {	// 2nd footer is also oregon_21
					*duration = O21_FOOTER;
					data->rbuffer[data->rptr]   = *duration;
				}
				preamb_state = WAIT_FOR_END_OF_HEADER;
			}
			if (*duration > PREAMB_SYNC_MIN) {
				preamb_pulse_counter++;
				preamb_duration += data->rbuffer[data->rptr];
			} else {
				// Check if we found the deviating pulse after a series of consecutive pulses
				if (preamb_pulse_counter > PRE_AMB_HEADER_CNT) {
					latch_duration = *duration;	// Remember this pulse for WFD2
					data->rbuffer[data->rptr] = O21_FOOTER;	// Emulate Footer
					*duration = O21_FOOTER;
					preamb_state = WAIT_FOR_END_OF_DATA_2;
				} else {
				// Below threshold, so we continue to wait
					preamb_duration = 0;
					preamb_pulse_counter = 0;
				}
			}
			break;

			case WAIT_FOR_END_OF_DATA_2:
			data->rptr = 0;	// Restore 1st SYNC byte already received
			preamb_pulse_counter = 1;
			data->rbuffer[data->rptr++] = latch_duration;

			if (*duration > O21_FOOTER) {
				// We only end up here if we have have detected all characteristics of an oregon pulestrain
				// As oregon is footerless (we may want to add maxgaplen to our evaluation),
				// we replace the undefined pause with the pilight definition of an oregon footer
				*duration = O21_FOOTER;
				data->rbuffer[data->rptr] = *duration;
			} else {
				// We have detected all characteristics of an oregon pulsestream.
				// pilight may have missed one or more pulses
				// The length of the received pulse is not in the expected length range
				// to resynchronize, lets do a special analysis of the next pulse.
				// The length of both pulses together are close to the absolute
				// duration and based on our knowledge of the SYNC structure
				// We use this knowledge and attempt to recreate the header pulse sequence in state 2A
				// If this is unsuccessful, we discard all received data and clear the receive buffer
				if (*duration  > PREAMB_SYNC_DMAX) {
					duration_previous = *duration;
					preamb_state = WAIT_FOR_END_OF_DATA_2A;
				}
			}
			break;

			case WAIT_FOR_END_OF_DATA_2A:
			// We get the next pulse, add the duration of the previous one
			// and attempt to reconstruct the SYNC pulse sequence
			// after that we continue with state 2
			*duration += duration_previous;
			duration_previous = 0;
			p_header_21 = 1;	// The next pulse is short
			// Rebuild missing SYNC Header: 488, 976, 488 488 976 488 488 976
			while ( (*duration > 100) && (p_header_21 < (unsigned int)L_HEADER_21) ) {
				data->rbuffer[data->rptr++] = header_21[p_header_21];
				// panic - take the exit, set state to WFH
				// e.q. the pulse duration was longer than the SYNC Header sequence
				//      or the buffer length was exceeded
				// We can not reconstruct the SYNC Header
				if(data->rptr > MAXPULSESTREAMLENGTH-1) {
					data->rptr = 1;	// Get's set to zero after end of the while loop
					preamb_duration = 0;
					preamb_pulse_counter = 1;	//Get's set to zero after end of the while loop
					preamb_state = WAIT_FOR_END_OF_HEADER;
					break;
				}
				*duration -= header_21[p_header_21];
				preamb_pulse_counter++;
				p_header_21++;
			}
			data->rptr--;	// Adjust pointer
			preamb_pulse_counter--;	// Adjust counter

			// if panic - ensure that we do not override the WFH state otherwise we enter state 3
			if (preamb_state!=WAIT_FOR_END_OF_HEADER) {
				preamb_state = WAIT_FOR_END_OF_DATA_3;
			}
			break;

			case WAIT_FOR_END_OF_DATA_3:
			if (*duration > 5100) {
				preamb_state = WAIT_FOR_END_OF_HEADER;
				*duration = O21_FOOTER;	// Replace GAP with defined footer value
				data->rbuffer[data->rptr]   = *duration;
			}
			break;

			default:
			break;
		} // Switch preamb_state

		data->rptr++;
		if(data->rptr > MAXPULSESTREAMLENGTH-1) {
			data->rptr = 0;
			preamb_duration = 0;
			preamb_pulse_counter = 0;
			preamb_state = WAIT_FOR_END_OF_HEADER;
		}
		if(*duration > gpio433->mingaplen) {
			// Let's do a little filtering here as well
			if(data->rptr >= gpio433->minrawlen && data->rptr <= gpio433->maxrawlen) {
				struct reason_received_pulsetrain_t *data1 = MALLOC(sizeof(struct reason_received_pulsetrain_t));
				if(data1 == NULL) {
					OUT_OF_MEMORY
				}
				data1->length = data->rptr;
				memcpy(data1->pulses, data->rbuffer, data->rptr*sizeof(int));
				data1->hardware = gpio433->id;

				eventpool_trigger(REASON_RECEIVED_PULSETRAIN, reason_received_pulsetrain_free, data1);
			}
			data->rptr = 0;
			// Avoid that short footer pulses (X10, IMPULS, ...) interfere with long header /sync pulses
			if(data->rptr > 20) {
				// Avoid a buffer reset and preserve potential Header / SYNC / Start pulses
				data->rptr = 0;
				preamb_duration = 0;
				preamb_pulse_counter = 0;
				if (preamb_state == WAIT_FOR_END_OF_DATA) {
					preamb_state = WAIT_FOR_END_OF_DATA_2;
				}
			}
		} // if *duration > 5100
	}
}

static void sys_parseHeader(struct hardware_t *gpio433, struct data_t *data, int *duration) {
// data->rbuffer data->rbuffer
// data->rptr data->rptr
// hw gpio433
	if(*duration > 0) {
		data->rbuffer[data->rptr++] = *duration;
		if(data->rptr > MAXPULSESTREAMLENGTH-1) {
			data->rptr = 0;
		}
		if(*duration > gpio433->mingaplen) {
			// Let's do a little filtering here as well
			if(data->rptr >= gpio433->minrawlen && data->rptr <= gpio433->maxrawlen) {
				struct reason_received_pulsetrain_t *data1 = MALLOC(sizeof(struct reason_received_pulsetrain_t));
				if(data1 == NULL) {
					OUT_OF_MEMORY
				}
				data1->length = data->rptr;
				memcpy(data1->pulses, data->rbuffer, data->rptr * sizeof(int));
				data1->hardware = gpio433->id;

				eventpool_trigger(REASON_RECEIVED_PULSETRAIN, reason_received_pulsetrain_free, data1);
			}
			data->rptr = 0;
		}
	}
}


static int client_callback(struct eventpool_fd_t *node, int event) {
	struct data_t *data = node->userdata;
	int duration = 0;

	if(doPause == 1) {
		return 0;
	}
	switch(event) {
		case EV_CONNECT_SUCCESS: {
			eventpool_fd_enable_highpri(node);
			timestamp.first = 0;
			timestamp.second = 0;
		} break;
		case EV_HIGHPRI: {
			eventpool_fd_enable_highpri(node);
			uint8_t c = 0;

			(void)read(node->fd, &c, 1);
			lseek(node->fd, 0, SEEK_SET);

			struct timeval tv;
			gettimeofday(&tv, NULL);
			timestamp.first = timestamp.second;
			timestamp.second = 1000000 * (unsigned int)tv.tv_sec + (unsigned int)tv.tv_usec;

			duration = (int)((int)timestamp.second-(int)timestamp.first);

			if(duration > 0) {
				data->rbuffer[data->rptr] = duration;
				data->rptr++;
				if(data->rptr > MAXPULSESTREAMLENGTH-1) {
					data->rptr = 0;
				}
				if(duration > gpio433->mingaplen) {
					// Let's do a little filtering here as well
					if(data->rptr >= gpio433->minrawlen && data->rptr <= gpio433->maxrawlen) {
						struct reason_received_pulsetrain_t *data1 = MALLOC(sizeof(struct reason_received_pulsetrain_t));
						if(data1 == NULL) {
							OUT_OF_MEMORY
						}
						data1->length = data->rptr;
						memcpy(data1->pulses, data->rbuffer, data->rptr*sizeof(int));
						data1->hardware = gpio433->id;

						eventpool_trigger(REASON_RECEIVED_PULSETRAIN, reason_received_pulsetrain_free, data1);
					}
					data->rptr = 0;
				}
			}


//			usr_parseHeader(gpio433, data, &duration);

		} break;
		case EV_DISCONNECTED: {
			FREE(node->userdata);
			eventpool_fd_remove(node);
		} break;
	}
	return 0;
}
#endif

static unsigned short gpio433HwInit(void *(*callback)(void *)) {
#if defined(__arm__) || defined(__mips__)
	char *platform = GPIO_PLATFORM;
	if(settings_select_string(ORIGIN_MASTER, "gpio-platform", &platform) != 0 || strcmp(platform, "none") == 0) {
		logprintf(LOG_ERR, "no gpio-platform configured");
		return EXIT_FAILURE;
	}
	if(wiringXSetup(platform, logprintf) < 0) {
		return EXIT_FAILURE;
	}
	if(gpio_433_out >= 0) {
		if(wiringXValidGPIO(gpio_433_out) != 0) {
			logprintf(LOG_ERR, "invalid sender pin: %d", gpio_433_out);
			return EXIT_FAILURE;
		}
		pinMode(gpio_433_out, PINMODE_OUTPUT);
	}
	if(gpio_433_in >= 0) {
		if(wiringXValidGPIO(gpio_433_in) != 0) {
			logprintf(LOG_ERR, "invalid receiver pin: %d", gpio_433_in);
			return EXIT_FAILURE;
		}
		if(wiringXISR(gpio_433_in, ISR_MODE_BOTH) < 0) {
			logprintf(LOG_ERR, "unable to register interrupt for pin %d", gpio_433_in);
			return EXIT_SUCCESS;
		}
	}
	if(gpio_433_in > 0) {
		int fd = wiringXSelectableFd(gpio_433_in);

		struct data_t *data = MALLOC(sizeof(struct data_t));
		if(data == NULL) {
			OUT_OF_MEMORY;
		}
		memset(data->rbuffer, '\0', sizeof(data->rbuffer));
		data->rptr = 0;
		data->callback = callback;

		eventpool_fd_add("433gpio", fd, client_callback, NULL, data);
	}
	return EXIT_SUCCESS;
#else
	logprintf(LOG_ERR, "the 433gpio module is not supported on this hardware", gpio_433_in);
	return EXIT_FAILURE;
#endif
}

static int gpio433Send(int *code, int rawlen, int repeats) {
	int r = 0, x = 0;
	if(gpio_433_out >= 0) {
		for(r=0;r<repeats;r++) {
			for(x=0;x<rawlen;x+=2) {
				digitalWrite(gpio_433_out, 1);
				usleep((__useconds_t)code[x]);
				digitalWrite(gpio_433_out, 0);
				if(x+1 < rawlen) {
					usleep((__useconds_t)code[x+1]);
				}
			}
		}
		digitalWrite(gpio_433_out, 0);
	} else {
		usleep(10);
	}
	return EXIT_SUCCESS;
}

static void *receiveStop(void *param) {
	doPause = 1;
	return NULL;
}

static void *receiveStart(void *param) {
	doPause = 0;
	return NULL;
}

static unsigned short gpio433Settings(JsonNode *json) {
	if(strcmp(json->key, "receiver") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_in = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	}
	if(strcmp(json->key, "sender") == 0) {
		if(json->tag == JSON_NUMBER) {
			gpio_433_out = (int)json->number_;
		} else {
			return EXIT_FAILURE;
		}
	}
	return EXIT_SUCCESS;
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void gpio433Init(void) {
	hardware_register(&gpio433);
	hardware_set_id(gpio433, "433gpio");

	options_add(&gpio433->options, 'r', "receiver", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");
	options_add(&gpio433->options, 's', "sender", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9-]+$");

	gpio433->minrawlen = 1000;
	gpio433->maxrawlen = 0;
	gpio433->mingaplen = 5100;
	gpio433->maxgaplen = 10000;

	gpio433->hwtype=RF433;
	gpio433->comtype=COMOOK;
	gpio433->init=&gpio433HwInit;
	// gpio433->deinit=&gpio433HwDeinit;
	gpio433->sendOOK=&gpio433Send;
	// gpio433->receiveOOK=&gpio433Receive;
	gpio433->settings=&gpio433Settings;

	eventpool_callback(REASON_SEND_BEGIN, receiveStop);
	eventpool_callback(REASON_SEND_END, receiveStart);
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
	module->name = "433gpio";
	module->version = "2.0";
	module->reqversion = "8.0";
	module->reqcommit = NULL;
}

void init(void) {
	gpio433Init();
}
#endif
