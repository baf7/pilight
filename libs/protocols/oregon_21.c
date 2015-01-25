/*
	Copyright (C) 2014 wo_rasp & CurlyMo

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see <http://www.gnu.org/licenses/>
*/
/*
Change Log:
0.9     - 1st release
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "../../pilight.h"
#include "common.h"
#include "dso.h"
#include "log.h"
#include "protocol.h"
#include "hardware.h"
#include "binary.h"
#include "gc.h"
#include "oregon_21.h"

#define PRINT_DEBUG
#define OREGON_21		       		oregon_21Weather
#define PULSE_OREGON_21_SYNC		976			  // 16 pairs Pre-Amble
#define PULSE_OREGON_21_SYNC_L		PULSE_OREGON_21_SYNC-192  // 784 minimum
#define PULSE_OREGON_21_SYNC_H		PULSE_OREGON_21_SYNC+192  // 1186 maximum
#define PULSE_OREGON_21_SHORT		488     // OREGON_21 Docu Clock and short pulse duration
#define PULSE_OREGON_21_SHORT_L		PULSE_OREGON_21_SHORT-268 // 220 min
#define PULSE_OREGON_21_SHORT_H		PULSE_OREGON_21_SHORT+192 // 680 max
#define PULSE_OREGON_21_LONG		976     // OREGON_21 Docu long pulse duration
#define PULSE_OREGON_21_LONG_L		PULSE_OREGON_21_LONG-288 // 688 min
#define PULSE_OREGON_21_LONG_H		PULSE_OREGON_21_LONG+432 // 1408 max
#define PULSE_OREGON_21_FOOTER		324     // GAP 11018/34
#define PULSE_OREGON_21_FOOTER_L	(PULSE_OREGON_21_FOOTER-25)*PULSE_DIV
#define PULSE_OREGON_21_FOOTER_H	(PULSE_OREGON_21_FOOTER+25)*PULSE_DIV+PULSE_OREGON_21_SHORT_H
// Bin / Rawlength definitions
// binlen protocol: 168 to 224
// PREAMB + SYNC + (14 to 21 data nibbles + 2 to 5 nibbles post amble)*manchester_encoding
// rawlenmin: 32+12+(14*8+2*8)*1= 172
// rawlenmax: 32+12+(21*8+5*8)*2= 460
#define BIN_ARRAY_CLASSIC			28		      // 28 - 16 is the Systemboundary
#define BINLEN_OREGON_21_PROT		BIN_ARRAY_CLASSIC*8     // 128 is the systemboundary
#define RAWLEN_OREGON_21_PROT		428		     // 255 is the systemboundary in the protocol_t structure

#define MINRAWLEN_OREGON_21_PROT	140     // all Data bit toogle
#define MAXRAWLEN_OREGON_21_PROT	428     // all Data bit equal

static void OREGON_21WeatherCreateMessage(	int device_id, int id, int unit, int battery, int temp, int humidity,
					  						int uv, int wind_dir, int wind_speed, int wind_avg, int rain, int rain_total, int pressure) {
	OREGON_21->message = json_mkobject();
	json_append_member(OREGON_21->message, "device_id", json_mknumber(device_id,0));
	json_append_member(OREGON_21->message, "id", json_mknumber(id,0));
	json_append_member(OREGON_21->message, "unit", json_mknumber(unit,0));
	json_append_member(OREGON_21->message, "battery", json_mknumber(battery,0));
	if (temp != -1) json_append_member(OREGON_21->message, "temperature", json_mknumber(temp,0));
	if (humidity != -1) json_append_member(OREGON_21->message, "humidity", json_mknumber(humidity,0));
	if (uv != -1) json_append_member(OREGON_21->message, "uv", json_mknumber(uv,0));
	if (wind_dir != -1) json_append_member(OREGON_21->message, "wind_dir", json_mknumber(wind_dir,0));
	if (wind_speed != -1) json_append_member(OREGON_21->message, "wind_speed", json_mknumber(wind_speed,0));
	if (wind_avg != -1) json_append_member(OREGON_21->message, "wind_avg", json_mknumber(wind_avg,0));
	if (rain != -1) json_append_member(OREGON_21->message, "rain", json_mknumber(rain,0));
	if (rain_total != -1) json_append_member(OREGON_21->message, "rain_total", json_mknumber(rain_total,0));
	if (pressure != -1) json_append_member(OREGON_21->message, "pressure", json_mknumber(pressure,0));
}

static void OREGON_21WeatherParseCode(void) {
	int i, x, s_0;
	int pBin = 0, pRaw = 0;
	int protocol_sync = 1;
	int rDataLow = 0, rDataTime = 0;
	int sign;
	int device_id = -1;
	int id = -1;
	int unit = -1;
	int battery = -1;
	int temp = -1;
	int humidity = -1;
	int uv = -1;
	int wind_dir = -1;
	int wind_speed = -1;
	int wind_avg = -1;
	int rain = -1;
	int rain_total = -1;
	int pressure = -1;
//
// The duration of pulses depends on the state of RF
// a,b,c
// a - 0- off,  1- on
// b - 0- min   1- max
// #define VAR_PULSE
#ifdef VAR_PULSE
	int dur_short[2][2] = { {220, 680}, {220, 680} };
	int dur_long[2][2] = { {688, 1408}, {688, 1408} };
#endif
	int rf_state = 0; 			// 0 - off, 1- on
//
//      int cksum = 0;
//
#define PULSE_OREGON_21_SHORT_L		PULSE_OREGON_21_SHORT-268	// 220 min
#define PULSE_OREGON_21_SHORT_H		PULSE_OREGON_21_SHORT+192	// 680 max
#define PULSE_OREGON_21_LONG		976     					// OREGON_21 Docu long pulse duration
#define PULSE_OREGON_21_LONG_L		PULSE_OREGON_21_LONG-288	// 688 min
#define PULSE_OREGON_21_LONG_H		PULSE_OREGON_21_LONG+432	// 1408 max
#define PULSE_OREGON_21_FOOTER		324							// GAP 11018/34
#define PULSE_OREGON_21_FOOTER_L	(PULSE_OREGON_21_FOOTER-25)*PULSE_DIV
#define PULSE_OREGON_21_FOOTER_H	(PULSE_OREGON_21_FOOTER+25)*PULSE_DIV+PULSE_OREGON_21_SHORT_H
// Bin / Rawlength definitions
// Decode Manchester pulse stream into binary
// and remove all inverted bits
	pRaw = 0;
	s_0 = 0;
	rf_state = 0;
	for  (i=0;i<BINLEN_OREGON_21_PROT;i++) {
		OREGON_21->binary[i]=0;
	}
#ifdef PRINT_DEBUG
	logprintf(LOG_DEBUG, "OREGON_21: Start decoding\n");
#endif
	while (pRaw<=OREGON_21->rawlen) {
		switch (protocol_sync) {
			case 0:									// Wait for end of Pre-Amble pulses, or terminate if more than 36 Pre-Amble bits found
				rDataTime = OREGON_21->raw[pRaw++];
				if( (rDataTime > PULSE_OREGON_21_SYNC_L) && (rDataTime < PULSE_OREGON_21_SYNC_H) ) {
					s_0++;			  				// The Pre Amble has a max of 32 long pulses
					if (s_0 > 48) {		 			// Allow leading noise pulses
						protocol_sync = 95;
					}
				}
#ifdef VAR_PULSE
				if( (rDataTime > dur_short[rf_state][0]) && (rDataTime < dur_short[rf_state][1]) ) {
#endif
#ifndef VAR_PULSE
				if( (rDataTime > PULSE_OREGON_21_SHORT_L) && (rDataTime < PULSE_OREGON_21_SHORT_H) ) {
#endif
					protocol_sync = 1;				// We found the first short pulse, indicating SYNC nibbles
					s_0 = 0;						// Reset Long Pulse counter, SYNC ends with the 4th long pulses
					rf_state = 1;					// ON
				}
			break;
			case 1: // The fourth long pulse defines the end of SYNC
				rDataTime = OREGON_21->raw[pRaw++];
				rf_state ^= 1;
printf("\np_s: %d rdt: %d pRaw: %d s_0: %d ", protocol_sync, rDataTime, pRaw, s_0);
#ifdef VAR_PULSE
				if( (rDataTime > dur_long[rf_state][0]) && (rDataTime < dur_long[rf_state][1]) ) {
#endif
#ifndef VAR_PULSE
				if( (rDataTime > PULSE_OREGON_21_LONG_L) && (rDataTime < PULSE_OREGON_21_LONG_H) ) {
#endif
					s_0++;							// The 4th SYNC pulse is a logical "1"
					if (s_0 > 3) {					// rDataLow 1-"0", -1-"1"
						protocol_sync = 2;
						rDataLow = -1;
						rDataTime = 0;				// A falling edge at Short Pulse duration defines no state change
						pBin = 0;					// Logical Bit pointer
					}
				}
			break;
			case 2:
				// We found the last SYNC Byte, it defines a "1" bit state
				// Now we can start decoding
				// We expect either two short pulses defining no change in bit state,
				// or we expect a long pulse, defining a bit toogle condition
				// The 1st bit is inverted, the 2nd bit is the noninverted state
				// rf_state is on
				rDataTime= OREGON_21->raw[pRaw++];
				rf_state ^= 1;
				// two short pulses define a bit with no logical state change
#ifdef VAR_PULSE
				if( (rDataTime > dur_short[rf_state][0]) && (rDataTime < dur_short[rf_state][1]) ) {
#endif
#ifndef VAR_PULSE
				if( rDataTime > PULSE_OREGON_21_SHORT_L && rDataTime < PULSE_OREGON_21_SHORT_H) {
#endif
					// No Data pulse yet, Sync only, there must be another Short pulse
					rDataTime=OREGON_21->raw[pRaw++];
					rf_state ^= 1;
#ifdef VAR_PULSE
					if( (rDataTime > dur_short[rf_state][0]) && (rDataTime < dur_short[rf_state][1]) ) {
#endif
#ifndef VAR_PULSE
					if( rDataTime > PULSE_OREGON_21_SHORT_L && rDataTime < PULSE_OREGON_21_SHORT_H) {
#endif
						// Data pulse, No Toogle
					} else {
						// Protocol Error, there is no valid single short pulse
#ifdef PRINT_DEBUG
						logprintf(LOG_DEBUG, "OREGON_21: Error missing 2nd short pulse");
#endif
						protocol_sync = 96;
					}
				} else {
#ifdef VAR_PULSE
					if( (rDataTime > dur_long[rf_state][0]) && (rDataTime < dur_long[rf_state][1]) ) {
#endif
#ifndef VAR_PULSE
					if( rDataTime > PULSE_OREGON_21_LONG_L && rDataTime < PULSE_OREGON_21_LONG_H) {
#endif
						// Data pulse, Toogle
						rDataLow = -rDataLow;
					} else {
						// Protocol Error or footer, there is no long pulse
#ifdef PRINT_DEBUG
						logprintf(LOG_DEBUG, "OREGON_21: Missing expected long Pulse. Check for footer condition");
#endif
						protocol_sync = 3;
					}
				}
				if ( rDataLow == 1 ) {
					OREGON_21->binary[pBin]=0;
				} else {
					OREGON_21->binary[pBin]=1;
				}
				if (protocol_sync == 2) {	       // Continue only if no footer or error condition set
					// Get the 2nd bit and check if it is inverted
					rDataTime= OREGON_21->raw[pRaw++];
					rf_state ^= 1;
					// Short Pulse changes are neutral
#ifdef VAR_PULSE
					if( (rDataTime > dur_short[rf_state][0]) && (rDataTime < dur_short[rf_state][1]) ) {
#endif
#ifndef VAR_PULSE
					if( rDataTime > PULSE_OREGON_21_SHORT_L && rDataTime < PULSE_OREGON_21_SHORT_H) {
#endif
						// No Data pulse yet, Sync only, there must be another Short pulse
						rDataTime=OREGON_21->raw[pRaw++];
						rf_state ^= 1;
#ifdef VAR_PULSE
							if( (rDataTime > dur_short[rf_state][0]) && (rDataTime < dur_short[rf_state][1])) {
#endif
#ifndef VAR_PULSE
							if( rDataTime > PULSE_OREGON_21_SHORT_L && rDataTime < PULSE_OREGON_21_SHORT_H) {
#endif
							// Data pulse, No Toogle
						} else {
							// Protocol Error, there is no single short pulse
#ifdef PRINT_DEBUG
							logprintf(LOG_DEBUG, "OREGON_21: Error missing 2nd inverted short pulse");
#endif
							protocol_sync = 96;
						}
					} else {
#ifdef VAR_PULSE
						if( (rDataTime > dur_long[rf_state][0]) && (rDataTime < dur_long[rf_state][1]) ) {
#endif
#ifndef VAR_PULSE
						if( rDataTime > PULSE_OREGON_21_LONG_L && rDataTime < PULSE_OREGON_21_LONG_H) {
#endif
							// Data pulse, Toogle
							rDataLow = - rDataLow;
						} else {
							// Protocol Error, footer condition is detected before
#ifdef PRINT_DEBUG
							logprintf(LOG_DEBUG, "OREGON_21: Error missing inverted long pulse");
#endif
							protocol_sync = 96;
						}
					}
					if ( rDataLow == 1 ) {
						if (OREGON_21->binary[pBin]==1) {
							OREGON_21->binary[pBin++]=0;
						} else {
#ifdef PRINT_DEBUG
							logprintf(LOG_DEBUG, "OREGON_21: Bit Error in pulse stream");
#endif
							protocol_sync = 96;
						}
					} else {
						if (OREGON_21->binary[pBin]==0) {
							OREGON_21->binary[pBin++]=1;
						} else {
#ifdef PRINT_DEBUG
							logprintf(LOG_DEBUG, "OREGON_21: Bit Error in pulse stream");
#endif
							protocol_sync = 96;
						}
					}
					if (pBin>BINLEN_OREGON_21_PROT) {
#ifdef PRINT_DEBUG
						logprintf(LOG_DEBUG, "OREGON_21: Too many binary bits decoded. Analysis abandoned");
#endif
						protocol_sync = 96;
					}
				}
			break;
			case 3:
			// We found most probably a footer, check for footer pulse
#ifdef PRINT_DEBUG
				logprintf(LOG_DEBUG, "OREGON_21: End of data. pulse: %d - pRaw: %d - bin: %d", rDataTime, --pRaw, --pBin);
#endif
				if ( (rDataTime > PULSE_OREGON_21_FOOTER_L) && (rDataTime < PULSE_OREGON_21_FOOTER_H) ) {
					protocol_sync = 98;
				} else {
#ifdef PRINT_DEBUG
				logprintf(LOG_DEBUG, "OREGON_21: Unknown Footer length");
#endif
				protocol_sync = 96;
			}
			break;
			case 95:
			// We did not find a valid Pre-Amble
#ifdef PRINT_DEBUG
				logprintf(LOG_DEBUG, "OREGON_21: No valid Pre-Amble data found.");
#endif
			case 96:
				protocol_sync = 99;
#ifdef PRINT_DEBUG
				logprintf(LOG_DEBUG, "OREGON_21: End of data. pulse: %d - pRaw: %d - bin: %d", rDataTime, pRaw, pBin);
#endif
			break;
			case 97:
			// We decoded a footer pulse without decoding the correct number of binary bits
#ifdef PRINT_DEBUG
				logprintf(LOG_DEBUG, "OREGON_21: Err 97 unexpected EOF.");
#endif
			case 98:
#ifdef PRINT_DEBUG
				logprintf(LOG_DEBUG, "OREGON_21: End of data. pulse: %d - pRaw: %d - bin: %d", rDataTime, pRaw, pBin);
#endif
			// We have reached the end of processing raw data
			case 99:
			default:
			break;
		}
		if (protocol_sync > 95) {
#ifdef PRINT_DEBUG
			logprintf(LOG_DEBUG, "**** OREGON_21 RAW CODE ****");
#endif
			if(log_level_get() >= LOG_DEBUG) {
				for(x=0;x<pRaw;x++) {
					printf("%d ", OREGON_21->raw[x]);
				}
				printf("\n");
			}
			break;
		} // Switch
	}

	if (protocol_sync > 97) {
	// ad checksum check later on

		device_id       =  (binToDec(OREGON_21->binary, 0, 3) << 12);   // System ID
		device_id       += (binToDec(OREGON_21->binary,4,7) << 8);
		device_id       += (binToDec(OREGON_21->binary,8,11) << 4);
		device_id       += (binToDec(OREGON_21->binary,12,15));
		id	      =  (binToDec(OREGON_21->binary, 16,19));				// Channel
		unit	    =  (binToDec(OREGON_21->binary, 20,23) << 4);   	// Unit
		unit	    += (binToDec(OREGON_21->binary, 24,27));
		battery	 =  (binToDec(OREGON_21->binary, 28,31) & 0x4);  		// Battery Flag Clear if result 0
		// Decode the payload of various V2.1 devices
		switch (device_id) {
			case 7456:					      // 1D20, F824, F8B4
			case 63524:
			case 63668:
			case 52259:						// THGR 328N - CC23
				temp    =  (binToDec(OREGON_21->binary, 32,35));		// Temp
				temp    += (binToDec(OREGON_21->binary, 36,39)*10);
				temp    += (binToDec(OREGON_21->binary, 40,43)*100);
				sign    =  (binToDec(OREGON_21->binary, 44,47));		// If set Temp is negative
			if(sign!=0)temp = -temp;
				humidity =  (binToDec(OREGON_21->binary, 48,51));       // Humidity
				humidity += (binToDec(OREGON_21->binary, 52,55)*10);
			break;
			case 60480:					     // EC40, C844
			case 51268:
				temp    =  (binToDec(OREGON_21->binary, 32,35));		// Temp
				temp    += (binToDec(OREGON_21->binary, 36,39)*10);
				temp    += (binToDec(OREGON_21->binary, 40,43)*100);
				sign    =  (binToDec(OREGON_21->binary, 44,47));		// If set Temp is negative
			if(sign!=0)temp = -temp;
			break;
			case 60526:					     // EC70, D874
			case 55412:
				uv      =  (binToDec(OREGON_21->binary, 32,35));		// uv index
				uv      += (binToDec(OREGON_21->binary, 36,39)*10);
			break;
			case 6532:					      // 1984, 1994
			case 6548:
				wind_dir    =  (int)(binToDec(OREGON_21->binary, 32,35)*22.5); // wind direction
				wind_speed  =  (binToDec(OREGON_21->binary, 44,47));    // wind speed
				wind_speed +=  (binToDec(OREGON_21->binary, 48,51)*10); // wind speed
				wind_speed +=  (binToDec(OREGON_21->binary, 52,55)*100);	// wind speed
				wind_avg    =  (binToDec(OREGON_21->binary, 56,59));    // wind average speed
				wind_avg   +=  (binToDec(OREGON_21->binary, 60,63)*10); // wind average speed
				wind_avg   +=  (binToDec(OREGON_21->binary, 64,67)*100);	// wind average speed
			break;
			case 10516:					     // 2914
				rain     =  (binToDec(OREGON_21->binary, 32,35));       // rain rate 0,01 inch
				rain    +=  (binToDec(OREGON_21->binary, 36,39)*10);
				rain    +=  (binToDec(OREGON_21->binary, 40,43)*100);
				rain    +=  (binToDec(OREGON_21->binary, 44,47)*1000);
				rain_total   =  (binToDec(OREGON_21->binary, 52,55));   // rain rate total, ignoring least sig. digit
				rain_total   +=  (binToDec(OREGON_21->binary, 56,59)*10);
				rain_total   +=  (binToDec(OREGON_21->binary, 60,63)*100);
				rain_total   +=  (binToDec(OREGON_21->binary, 64,67)*1000);
			break;
			case 11536:					     // 2D10
				rain     =  (binToDec(OREGON_21->binary, 32,35));       // rain rate 0,1 mm
				rain    +=  (binToDec(OREGON_21->binary, 36,39)*10);
				rain    +=  (binToDec(OREGON_21->binary, 40,43)*100);
				rain_total   =  (binToDec(OREGON_21->binary, 44,47));   // rain rate total
				rain_total   +=  (binToDec(OREGON_21->binary, 48,51)*10);
				rain_total   +=  (binToDec(OREGON_21->binary, 52,55)*100);
				rain_total   +=  (binToDec(OREGON_21->binary, 56,59)*1000);
				rain_total   +=  (binToDec(OREGON_21->binary, 60,63)*10000);
			break;
			case 23904:					     // 5D60
				temp    =  (binToDec(OREGON_21->binary, 32,35));		// Temp
				temp    += (binToDec(OREGON_21->binary, 36,39)*10);
				temp    += (binToDec(OREGON_21->binary, 40,43)*100);
				sign    =  (binToDec(OREGON_21->binary, 44,47));		// If set Temp is negative
				if(sign!=0)temp = -temp;
				humidity =  (binToDec(OREGON_21->binary, 48,51));       // Humidity
				humidity += (binToDec(OREGON_21->binary, 52,55)*10);
				pressure	=  (binToDec(OREGON_21->binary, 64,67) << 8);   // pressure Hg
				pressure	+= (binToDec(OREGON_21->binary, 68,71) << 4);
				pressure	+= (binToDec(OREGON_21->binary, 72,75));
			break;

			default:
				logprintf(LOG_DEBUG, "OREGON_21: Unknown device_id: %d", device_id);
			break;
		}


#ifdef PRINT_DEBUG
		logprintf(LOG_DEBUG, "**** OREGON_21 BIN CODE ****");
#endif
		if(log_level_get() >= LOG_DEBUG) {
			printf("\n device: %d - id: %d - unit: %d - batt: %d - temp: %d - humi: %d - uv: %d",device_id, id, unit, battery, temp, humidity, uv);
			printf("\n wind_dir: %d - wind_speed: %d - wind_avg: %d - rain: %d - rain_total: %d - pressure: %d\n", wind_dir, wind_speed, wind_avg, rain, rain_total, pressure);
		}

		OREGON_21WeatherCreateMessage(device_id, id, unit, battery, temp, humidity, uv, wind_dir, wind_speed, wind_avg, rain, rain_total, pressure);

	} else {
#ifdef PRINT_DEBUG
		printf("\n*****-> protocol_sync: %d <-*****\n",protocol_sync);
		logprintf(LOG_DEBUG, "OREGON_21 Parsecode Error");
#endif
	}
}

#ifndef MODULE
__attribute__((weak))
#endif

void oregon_21WeatherInit(void) {
	protocol_register(&OREGON_21);
	protocol_set_id(OREGON_21, "oregon_21");
	protocol_device_add(OREGON_21, "oregon_21", "Oregon v2.1 protocol sensors");
	protocol_plslen_add(OREGON_21, PULSE_OREGON_21_FOOTER);	// Footer length ratio: (11018/PULSE_DIV)
	// *** add additional footer length here
	// protocol_plslen_add(OREGON_21, nnn);					// Footer length value nnn=(Footer Length in µS)/34
	OREGON_21->devtype = SENSOR;
	OREGON_21->hwtype = RF433;
	OREGON_21->pulse = 2;									// LONG pulse
	OREGON_21->rawlen = RAWLEN_OREGON_21_PROT;				// dynamically depending on 1st or repeated frame and binary value of 1st bit
	OREGON_21->minrawlen = MINRAWLEN_OREGON_21_PROT;
	OREGON_21->maxrawlen = MAXRAWLEN_OREGON_21_PROT;
	// Hardware Sync: 4 SYNC (S.S.L)
	// START SL 1st Data bit=1 or LL 1st Data bit=0
	OREGON_21->binlen = BINLEN_OREGON_21_PROT;

	options_add(&OREGON_21->options, 'd', "device_id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-9]{1,4}|[1-5][0-9]{4}|6[0-4][0-9]{3}|65[0-4][0-9]{2}|655[0-2][0-9]|6553[0-5])$");
	options_add(&OREGON_21->options, 'u', "unit", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-9]{1,2}|1[0-9]{1,2}|2[0-4][0-9]|25[0-5])$");
	options_add(&OREGON_21->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-4])$");
	options_add(&OREGON_21->options, 'b', "battery", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^([0-4])$");

	options_add(&OREGON_21->options, 't', "temperature", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
	options_add(&OREGON_21->options, 'h', "humidity", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9]{1,2}$");

//	options_add(&OREGON_21->options, 't', "temperature", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
//	options_add(&OREGON_21->options, 'h', "humidity", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,2}$");
	options_add(&OREGON_21->options, 'v', "uv", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,2}$");
	options_add(&OREGON_21->options, 'w', "wind_dir", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
	options_add(&OREGON_21->options, 'j', "wind_speed", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
	options_add(&OREGON_21->options, 'k', "wind_avg", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
	options_add(&OREGON_21->options, 'l', "rain", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,4}$");
	options_add(&OREGON_21->options, 'm', "rain_total", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,5}$");
	options_add(&OREGON_21->options, 'n', "pressure", OPTION_OPT_VALUE, DEVICES_OPTIONAL, JSON_NUMBER, NULL, "^[0-9]{1,3}$");

	options_add(&OREGON_21->options, 0, "decimals", OPTION_OPT_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "[0-9]");
	options_add(&OREGON_21->options, 0, "readonly", OPTION_OPT_VALUE, GUI_SETTING, JSON_NUMBER, (void *)0, "^[10]{1}$");
	options_add(&OREGON_21->options, 0, "show-humidity", OPTION_OPT_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");
	options_add(&OREGON_21->options, 0, "show-temperature", OPTION_OPT_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");

	OREGON_21->parseCode=&OREGON_21WeatherParseCode;
}

#ifdef MODULE
void compatibility(struct module_t *module) {
	module->name =  "oregon_21";
	module->version =  "1.02";
	module->reqversion =  "5.0";
	module->reqcommit =  NULL;
}

void init(void) {
	oregon_21WeatherInit();
}
#endif
