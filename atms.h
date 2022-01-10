#ifndef INC_ATMS_H
#define INC_ATMS_H

#define PKT_TYPE_TEST           0
#define PKT_TYPE_POSITION       1
#define PKT_TYPE_STATUS         2
#define PKT_TYPE_OBJECT         3
#define PKT_TYPE_MESSAGE        4
#define PKT_TYPE_MSGACK         5
#define PKT_TYPE_TELEMETRY_DATA 6
#define PKT_TYPE_TELEMETRY_DEF  7
#define PKT_TYPE_WEATHER        8
#define PKT_TYPE_SUBSCRIPTION   9
#define PKT_TYPE_LOCALINFO      10
#define PKT_TYPE_COMMENT        11
#define PKT_TYPE_USER_DEFINED   127
#define PKT_TYPE_ROUTING_ANN    128
#define PKT_TYPE_TRACEROUTE     129
#define PKT_TYPE_ACK            255

#define FLAG_LAT_SOUTH          0x10
#define FLAG_HAS_ALTITUDE       0x20
#define FLAG_HAS_COURSE         0x40
#define FLAG_HAS_SPEED          0x80
#define FLAG_LON_WEST           0x10
#define FLAG_HAS_PHGD           0x20
#define FLAG_HAS_TIMESTAMP      0x40
#define FLAG_MSG_SUPPORT        0x80

#endif
