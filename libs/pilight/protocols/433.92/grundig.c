#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../core/pilight.h"
#include "../../core/common.h"
#include "../../core/dso.h"
#include "../../core/log.h"
#include "../protocol.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "grundig.h"

#define PULSE_MULTIPLIER	2
#define MIN_PULSE_LENGTH	170
#define MAX_PULSE_LENGTH	180
#define AVG_PULSE_LENGTH	175
#define RAW_LENGTH			36

static int validate(void) {
        //----------------------------------
        printf("\n\n-----GRUNDIG validate-----\n\n");
        if(grundig->rawlen == RAW_LENGTH) {
                if(grundig->raw[grundig->rawlen-1] >= (int)(MIN_PULSE_LENGTH*PULSE_DIV) &&
                   grundig->raw[grundig->rawlen-1] <= (int)(MAX_PULSE_LENGTH*PULSE_DIV)) {
                	return 0;
                }
        }
        return -1;
}

static void createMessage(int systemcode, int unitcode, int state) {
        //--------------------------
        printf("\n\n-----GRUNDIG createMessage-----\n\n");
        grundig->message = json_mkobject();
        json_append_member(grundig->message, "systemcode", json_mknumber(systemcode, 0));
        json_append_member(grundig->message, "unitcode", json_mknumber(unitcode, 0));

        if(state == 0) {
                json_append_member(grundig->message, "state", json_mkstring("on"));
        } else {
                json_append_member(grundig->message, "state", json_mkstring("off"));
        }
}

static int createCode(JsonNode *code) {
    double itmp = -1;
    int unitcode = -1, systemcode = -1, state = -1;

    if(json_find_number(code, "unitcode", &itmp) == 0)
        unitcode = (int)round(itmp);
    if(json_find_number(code, "systemcode", &itmp) == 0)
        systemcode = (int)round(itmp);
    if(json_find_number(code, "off", &itmp) == 0)
        state=1;
    else if(json_find_number(code, "on", &itmp) == 0)
        state=0;
// If any parameter is still having its initial value no json element was found
    if(systemcode==-1 || (unitcode==-1) || state==-1) {
        logprintf(LOG_ERR, "grundig: insufficient number of arguments");
        return EXIT_FAILURE;
    } else if(systemcode < 0) {
        logprintf(LOG_ERR, "grundig: invalid systemcode");
        return EXIT_FAILURE;
    } else if(unitcode < 0) {
        logprintf(LOG_ERR, "grundig: invalid unit code");
        return EXIT_FAILURE;
    } else {
// Add if required any logic in accordance to individual needs (this is just an example)
        if(unitcode > 99) {
            unitcode = 100;
        }
// End of adding any logic
// Start building raw array, initialize length of raw array
        grundig->rawlen = RAW_LENGTH;
// Check validity of entered user data
        createMessage(systemcode, unitcode, state);
// Start populating raw data array (e.q. convert user input into raw pulse values)
// ..
// End populating raw data array
    }
    return EXIT_SUCCESS;
}

static void parseCode(void) {
    int binary[RAW_LENGTH], x = 0;

    if(grundig->rawlen>RAW_LENGTH) {
        logprintf(LOG_ERR, "grundig: parsecode - invalid protocol length %d", grundig->rawlen);
        return;
    }
// Dummy decoder logic
    for(x=0; x<grundig->rawlen-1; x++) {
        if(grundig->raw[x] > AVG_PULSE_LENGTH) {
            binary[x] = 1;
		} else {
			binary[x] = 0;
		}
    }
// End Dummy decoder logic
    int unitcode = binToDecRev(binary, 0, 9);
    int systemcode = binToDecRev(binary, 10,19);
    int state = binToDecRev(binary, 20, 20);

    createMessage(systemcode, unitcode, state);
}

static void printHelp(void) {
    printf("\t Dummy Help file\n");
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif

void grundigInit(void) {
    //----------------------------------
    printf("\n\n-----GRUNDIG grundigInit-----\n\n");
    protocol_register(&grundig);
    protocol_set_id(grundig, "grundig");
    protocol_device_add(grundig, "grundig", "Grundig QH-831A Doorbell protocol");
    grundig->devtype = SWITCH;
    grundig->hwtype = RF433;
    grundig->minrawlen = RAW_LENGTH;
    grundig->maxrawlen = RAW_LENGTH;
    grundig->maxgaplen = (int)MAX_PULSE_LENGTH*PULSE_DIV;
    grundig->mingaplen = (int)MIN_PULSE_LENGTH*PULSE_DIV;

//    options_add(&grundig->options, 't', "on", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
//    options_add(&grundig->options, 'f', "off", OPTION_NO_VALUE, DEVICES_STATE, JSON_STRING, NULL, NULL);
//    options_add(&grundig->options, 'u', "unit", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-3])$");
//    options_add(&grundig->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "^([0-9]|[1-9][0-9])$");

    grundig->parseCode=&parseCode;
    grundig->createCode=&createCode;
    grundig->printHelp=&printHelp;
    grundig->validate=&validate;
}

#if defined(MODULE) && !defined(_WIN32)
void compatibility(struct module_t *module) {
    //----------------------------------
    printf("\n\n-----GRUNDIG compability-----\n\n");
    module->name = "grundig";
    module->version = "0.1";
    module->reqversion = "5.0";
    module->reqcommit = NULL;
}

void init(void) {
    //----------------------------------
    printf("\n\n-----GRUNDIG init-----\n\n");
    grundigInit();
}
#endif
