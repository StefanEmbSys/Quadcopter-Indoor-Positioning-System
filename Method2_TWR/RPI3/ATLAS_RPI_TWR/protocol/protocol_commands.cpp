//
//  protocol_commands.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "protocol.h"
#include "protocol_structures.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

using namespace protocol;

const uint16_t msg_id_distance = 0x0105;

template<typename T> inline uint16_t getMsgId(T t)
{
    return 0;
}

template<> inline uint16_t getMsgId<msgDistance_t>(msgDistance_t t)
{
    return msg_id_distance;
}


template<typename T>
bool Protocol::sendMsg(T msg)
{
    uint16_t msg_id = getMsgId(msg);

    packet_t packet;
    populateHeader(&packet, msg_id, sizeof(T));
    memcpy(packet.payload, &msg, sizeof(T));
    sendMessage(&packet);

    if(!getPacket(&packet, 9))
    {
        std::cout << "Invalid Response received!" <<  std::endl;
        return false;
    }

    return true;
}

template<typename T>
bool Protocol::sendReplyMsg(T msg)
{
    uint16_t msg_id = getMsgId(msg);

    packet_t packet;

    if(!getPacket(&packet, 9))
    {
        std::cout << "Invalid Request from BaseStation received!" <<  std::endl;
        return false;
    }
    else
    {
        populateHeader(&packet, msg_id, sizeof(T));
        memcpy(packet.payload, &msg, sizeof(T));
        sendMessage(&packet);
    }
    return true;
}

template bool Protocol::sendReplyMsg<msgDistance_t>(msgDistance_t msg);
template bool Protocol::sendMsg<msgDistance_t>(msgDistance_t msg);

