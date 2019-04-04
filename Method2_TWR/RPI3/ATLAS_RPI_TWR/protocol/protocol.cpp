//
//  protocol.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "protocol.h"
#include "protocol_structures.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "../serial/serial.h"
#include "../ethernet/ethernet.h"

#include <chrono>

using namespace protocol;

Protocol::Protocol(std::string port, int zmq_connection_type)
    : m_port(port, zmq_connection_type)
{

}

Protocol::~Protocol()
{

}

#if CONNECTION == ETH_ZMQ
bool Protocol::fillPacketFromBuffer(packet_t *pkt, uint8_t* buffer)
{
    int i = 0;
    // Split the received buffer to the packet structure used by ATLAS
    pkt->messageId =   buffer[1];
    pkt->messageId += (buffer[0] << 8);
    pkt->payloadLength =   buffer[3];
    pkt->payloadLength += (buffer[2] << 8);

    if(pkt->payloadLength > maxpayloadSize)
    {
        return false;
    }

    for(i = 0; i < pkt->payloadLength; i++)
    {
        pkt->payload[i] = buffer[i + 4];
    }

    return true;
}
#endif // CONNECTION

bool Protocol::poll(packet_t *pkt)
{
    bool isWholePacket = false;

    // Receive non blocking because of the continous polling.
    // If available, receive the entire packet and split it to *pkt
    uint8_t buffer[maxMsgSize];

    if (m_port.receive_noblock((void*)&buffer, (sizeof(msgDistance_t) + 4)) == 0)
    {
        /*
        int x = 0;
        // print received message
        for(x = 0; x < 5; x++)
        {
            fprintf(stdout, "%c", buffer[x]);
            buffer[x] = 0;
        }
        fprintf(stdout, "\r\n");
        */

        // send an answer (necessary because of the "Reqest-Reply" pattern of ZMQ)
        m_port.send("0", 1);

        isWholePacket = fillPacketFromBuffer(&m_packet, buffer);
        *pkt = m_packet;
    }
    else
    {
        // Do nothing and continue
    }

    usleep(100);
    return isWholePacket;
}

void Protocol::populateHeader(packet_t *packet,
                              const uint16_t messageId,
                              const uint16_t payloadLength) const
{
    packet->messageId = messageId;
    packet->payloadLength = payloadLength;
}

bool Protocol::getPacket(packet_t *packet, uint32_t get_length)
{
    bool isWholePacket = false;
    // Receive blocking because of the awaited answer for the REQ-REP pattern of ZMQ.
    // Once the answer is, receive the entire packet and split it to *pkt

    uint8_t buffer[maxMsgSize];
    //auto ts = std::chrono::high_resolution_clock::now();

    if (m_port.receive((void*)buffer, get_length) != -1)
    {
        // print received message
        int x = 0;
        for(x = 0; x < get_length; x++)
        {
            fprintf(stdout, "%x ", buffer[x]);
        }
        fprintf(stdout, "\r\n");

        isWholePacket = fillPacketFromBuffer(packet, buffer);
        /*
        auto t = std::chrono::high_resolution_clock::now();
        auto d = t - ts;
        if (d > std::chrono::milliseconds(timeout))
        {
            std::cerr << "Timeout occurred after " << timeout << "ms" << std::endl;
            return false;
        }
        */
    }

    return isWholePacket;
}

bool Protocol::sendMessage(packet_t *packet)
{
    uint8_t msg[maxMsgSize];

    memcpy(msg, packet, 4);
    memcpy(msg + 4, packet->payload, packet->payloadLength);

    /*
    std::cout << "RAW: ";
    for(uint16_t i = 0; i < 6 + packet->payloadLength + 2; ++i)
    {
        int n = msg[i];
        //std::cout << "x" << std::hex << n << " ";
    }
    std::cout << std::endl;
    */

    m_port.send(msg, 4 + packet->payloadLength + 2);
    return true;
}

