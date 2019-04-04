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

#if CONNECTION == ETH_ZMQ
const uint16_t maxpayloadSize = 0x0031; // max. payload size defined from payload "msg_id_toad" (==dez49)
const uint16_t maxMsgSize = 0x0039; // max. payload size defined from payload "msg_id_toad" + 8 (==dez57)
#elif CONNECTION == SER_USB
const uint16_t payloadSize = 0x5FF;
#endif // CONNECTION

#pragma pack(1)
typedef struct
{
    uint16_t preamble;
    uint16_t messageId;
    uint16_t payloadLength;
    uint8_t payload[maxpayloadSize];
    uint16_t checksum;
} packet_t;

typedef enum
{
    DECODER_STATE_PREAMBLE_0 = 0,
    DECODER_STATE_PREAMBLE_1,
    DECODER_STATE_CLASS_ID,
    DECODER_STATE_MSG_ID,
    DECODER_STATE_LENGTH_0,
    DECODER_STATE_LENGTH_1,
    DECODER_STATE_PAYLOAD,
    DECODER_STATE_CHECKSUM_0,
    DECODER_STATE_CHECKSUM_1
} decoderState_t;

class Protocol
{
public:
#if CONNECTION == ETH_ZMQ
    Protocol(std::string port, int zmq_connection_type);
    template<typename T>
    bool sendReplyMsg(T msg);
#elif CONNECTION == SER_USB
    Protocol(std::string port);
    template<typename T>
    bool sendMsgPoll(T *msg);
#endif // CONNECTION

    ~Protocol();

    bool poll(packet_t *pkt);
    uint16_t calculateCheckSum(const packet_t *packet) const;

    template<typename T>
    bool sendMsg(T msg);

private:
    #if CONNECTION == ETH_ZMQ
    Ethernet m_port;
    bool fillPacketFromBuffer(packet_t *pkt, uint8_t* buffer);
    bool getPacket(packet_t *packet, uint32_t get_length);
    #elif CONNECTION == SER_USB
    Serial m_port;
    uint16_t m_payloadCounter = 0;
    decoderState_t m_state = DECODER_STATE_PREAMBLE_0;
    bool getPacket(packet_t *packet, uint32_t timeout);
    #endif
    packet_t m_packet;

    bool sendMessage(packet_t *packet);

    bool checkPacket(const packet_t *packet) const;
    bool evaluatePacket(packet_t *packet);
    void populateHeader(packet_t *packet, const uint16_t messageId, const uint16_t payloadLength) const;
};

}


#endif /* defined(__atlas__protocol__) */
