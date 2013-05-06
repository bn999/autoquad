/*
 * aq_mavlink_gnd.h
 *
 *  Created on: Dec 23, 2012
 *      Author: Max
 */

#ifndef AQ_MAVLINK_GND_H_
#define AQ_MAVLINK_GND_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "logger.h"
#include "../mavlink_types.h"

#define MAVLINK_HEARTBEAT_INTERVAL	    1e6	    //  1Hz
#define MAVLINK_PARAM_INTERVAL		    2e4	    // 50Hz
#define MAVLINK_WP_TIMEOUT		    1e6	    // 1 second
#define MAVLINK_NOTICE_DEPTH		    20
#define MAVLINK_PARAMID_LEN		    16
//#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

typedef struct {
    unsigned long nextHeartbeat;
    unsigned long nextParam;
    unsigned int currentParam;

    // TODO: pull actual count from header files
//    unsigned long streamInterval[13];
//    unsigned long streamNext[13];

    // this is a temporary implementation until we adopt mavlink completely
    int numParams;

    uint16_t packetDrops;
    uint16_t idlePercent;
    uint8_t mode;
    uint8_t nav_mode;
    uint8_t status;
    uint8_t wpTargetSysId;
    uint8_t wpTargetCompId;
    uint8_t wpCount;
    uint8_t wpCurrent;
    uint32_t wpNext;

    unsigned long lastCounter;

} mavlinkStruct_t;

static const int mavBufLen = MAVLINK_MAX_PACKET_LEN + sizeof(uint64_t);

extern mavlinkStruct_t mavlinkData;
extern mavlink_system_t mavlink_system;

extern void mavlinkInit(void);
extern void mavlinkWpReached(uint16_t seqId);
extern void mavlinkWpAnnounceCurrent(uint16_t seqId);
extern void mavlinkDo(loggerRecord_t *l);
//static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {}


#ifdef __cplusplus
}
#endif

#endif /* AQ_MAVLINK_GND_H_ */
