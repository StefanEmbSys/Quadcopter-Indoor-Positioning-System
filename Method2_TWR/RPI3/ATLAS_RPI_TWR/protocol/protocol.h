//
//  protocol.h
//  atlas
//
//  Created by Janis on 26.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__protocol__
#define __atlas__protocol__

#include <stdio.h>
#include <string>
#include <iostream>

#include "stdint.h"
#include "protocol_structures.h"

#include "../serial/serial.h"
#include "../ethernet/ethernet.h"

#include "../atlas_comm_conf.h"

namespace protocol
{

const uint16_t maxpayloadSize = 0x004C; // max. payload size defined from payload "msgDistance_t" (==dez76)
const uint16_t maxMsgSize = 0x0050; // max. payload size defined from payload "msgDistance_t" + 4 (==dez80)

#pragma pack(1)
typedef struct
{
    uint16_t messageId;
    uint16_t payloadLength;
    uint8_t payload[maxpayloadSize];
} packet_t;


class Protocol
{
public:
    Protocol(std::string port, int zmq_connection_type);
    ~Protocol();
    bool poll(packet_t *pkt);
    template<typename T>
    bool sendReplyMsg(T msg);
    template<typename T>
    bool sendMsg(T msg);

private:
    packet_t m_packet;
    Ethernet m_port;
    bool fillPacketFromBuffer(packet_t *pkt, uint8_t* buffer);
    bool getPacket(packet_t *packet, uint32_t get_length);
    bool sendMessage(packet_t *packet);
    void populateHeader(packet_t *packet, const uint16_t messageId, const uint16_t payloadLength) const;
};

}


#endif /* defined(__atlas__protocol__) */
