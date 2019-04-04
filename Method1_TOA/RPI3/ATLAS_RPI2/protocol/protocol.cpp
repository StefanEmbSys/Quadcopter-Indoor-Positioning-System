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


const uint16_t preamble = 0x62B5;

#if CONNECTION == ETH_ZMQ
Protocol::Protocol(std::string port, int zmq_connection_type)
    : m_port(port, zmq_connection_type)
#elif CONNECTION == SER_USB
Protocol::Protocol(std::string port)
    : m_port(port, 115200)
#endif // CONNECTION
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
    pkt->preamble =   buffer[0];
    pkt->preamble += (buffer[1] << 8);
    pkt->messageId =   buffer[2];
    pkt->messageId += (buffer[3] << 8);
    pkt->payloadLength =   buffer[4];
    pkt->payloadLength += (buffer[5] << 8);

    if(pkt->payloadLength > maxpayloadSize)
    {
        return false;
    }

/*
    for(i = pkt->payloadLength; i <= 0; i--)
    {
        pkt->payload[pkt->payloadLength - i] = buffer[6+i];
    }
*/
    for(i = 0; i < pkt->payloadLength; i++)
    {
        pkt->payload[i] = buffer[i+6];
    }

    pkt->checksum = buffer[pkt->payloadLength + 6];
    pkt->checksum += (buffer[pkt->payloadLength + 7] << 8);
    return true;
}
#endif // CONNECTION

bool Protocol::poll(packet_t *pkt)
{
    bool isWholePacket = false;

    #if CONNECTION == ETH_ZMQ
    // Receive non blocking because of the continous polling.
    // If available, receive the entire packet and split it to *pkt
    uint8_t buffer[maxMsgSize];

    if (m_port.receive_noblock((void*)&buffer, (sizeof(msgToa_t) + 8)) == 0) //real length to be done!!
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

    #elif CONNECTION == SER_USB

    uint8_t c;
    while(true)
    {
        int32_t ret = m_port.receive(&c);
        if(ret == 0)
        {
            usleep(200);
            break;
        }
        if(ret < 0)
        {
            usleep(200);
            break;
        }

        //std::cout << std::hex << "x" << (int)c << " ";
        //std::cout << std::flush;
        switch(m_state)
        {
        case DECODER_STATE_PREAMBLE_0:
            //std::cout << "DECODER_STATE_SYNC_0" << std::endl;
            m_state = (c == (preamble & 0xFF)) ? DECODER_STATE_PREAMBLE_1 : DECODER_STATE_PREAMBLE_0;
            break;

        case DECODER_STATE_PREAMBLE_1:
            //std::cout << "DECODER_STATE_SYNC_1" << std::endl;
            m_state = (c == (preamble >> 8)) ? DECODER_STATE_CLASS_ID : DECODER_STATE_PREAMBLE_0;
            break;

        case DECODER_STATE_CLASS_ID:
            //std::cout << "DECODER_STATE_CLASS_ID" << std::endl;
            m_packet.messageId = c;
            m_state = DECODER_STATE_MSG_ID;
            break;

        case DECODER_STATE_MSG_ID:
            //std::cout << "DECODER_STATE_MSG_ID" << std::endl;
            m_packet.messageId += (c << 8);
            m_state = DECODER_STATE_LENGTH_0;
            break;

        case DECODER_STATE_LENGTH_0:
            //std::cout << "DECODER_STATE_LENGTH_0" << std::endl;
            m_packet.payloadLength = c;
            m_state = DECODER_STATE_LENGTH_1;
            break;

        case DECODER_STATE_LENGTH_1:
            //std::cout << "DECODER_STATE_LENGTH_1" << std::endl;
            m_packet.payloadLength += (c << 8);

            if(m_packet.payloadLength > payloadSize)
            {
                return false;
            }

            m_payloadCounter = m_packet.payloadLength;
            if(m_packet.payloadLength > 0)
            {
                m_state = DECODER_STATE_PAYLOAD;
            }
            else
            {
                m_state = DECODER_STATE_CHECKSUM_0;
            }
            break;

        case DECODER_STATE_PAYLOAD:
            //std::cout << "DECODER_STATE_PAYLOAD" << std::endl;
            m_packet.payload[m_packet.payloadLength - m_payloadCounter--] = c;
            if(m_payloadCounter == 0)
            {
                m_state = DECODER_STATE_CHECKSUM_0;
            }
            break;

        case DECODER_STATE_CHECKSUM_0:
            //std::cout << "DECODER_STATE_CHECKSUM_0" << std::endl;
            m_packet.checksum = c;
            m_state = DECODER_STATE_CHECKSUM_1;
            break;

        case DECODER_STATE_CHECKSUM_1:
            //std::cout << "DECODER_STATE_CHECKSUM_1" << std::endl;
            m_packet.checksum += (c << 8);
            m_state = DECODER_STATE_PREAMBLE_0;
            isWholePacket = true;
            *pkt = m_packet;
            //std::cout << "Took " << std::dec << std::chrono::duration_cast<std::chrono::milliseconds>(d).count() << "ms" << std::endl;
            return isWholePacket;
            break;
        }
        //usleep(10);
    }
    #endif // CONNECTION
    usleep(100);
    return isWholePacket;
}

void Protocol::populateHeader(packet_t *packet,
                              const uint16_t messageId,
                              const uint16_t payloadLength) const
{
    packet->preamble = preamble;
    packet->messageId = messageId;
    packet->payloadLength = payloadLength;
}

#if CONNECTION == ETH_ZMQ
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

#elif CONNECTION == SER_USB
bool Protocol::getPacket(packet_t *packet, uint32_t timeout)
{
    bool isWholePacket = false;
    decoderState_t state = DECODER_STATE_PREAMBLE_0;
    uint8_t payloadCounter = 0;

    auto ts = std::chrono::high_resolution_clock::now();

    while (!isWholePacket)
    {
        auto t = std::chrono::high_resolution_clock::now();
        auto d = t - ts;

        if (d > std::chrono::milliseconds(timeout))
        {
            std::cerr << "Timeout occurred after " << timeout << "ms" << std::endl;
            return false;
        }

        uint8_t c;
        while(true)
        {
            int ret = m_port.receive(&c);
            if(ret == 0)
            {
                usleep(10);
                break;
            }
            if(ret < 0)
            {
                usleep(10);
                break;
            }

            //std::cout << std::hex << "x" << (int)c << " ";
            //std::cout << std::flush;
            switch(state)
            {
            case DECODER_STATE_PREAMBLE_0:
                //std::cout << "DECODER_STATE_SYNC_0" << std::endl;
                state = (c == (preamble & 0xFF)) ? DECODER_STATE_PREAMBLE_1 : DECODER_STATE_PREAMBLE_0;
                break;

            case DECODER_STATE_PREAMBLE_1:
                //std::cout << "DECODER_STATE_SYNC_1" << std::endl;
                state = (c == (preamble >> 8)) ? DECODER_STATE_CLASS_ID : DECODER_STATE_PREAMBLE_0;
                break;

            case DECODER_STATE_CLASS_ID:
                //std::cout << "DECODER_STATE_CLASS_ID" << std::endl;
                packet->messageId = c;
                state = DECODER_STATE_MSG_ID;
                break;

            case DECODER_STATE_MSG_ID:
                //std::cout << "DECODER_STATE_MSG_ID" << std::endl;
                packet->messageId += (c << 8);
                state = DECODER_STATE_LENGTH_0;
                break;

            case DECODER_STATE_LENGTH_0:
                //std::cout << "DECODER_STATE_LENGTH_0" << std::endl;
                packet->payloadLength = c;
                state = DECODER_STATE_LENGTH_1;
                break;

            case DECODER_STATE_LENGTH_1:
                //std::cout << "DECODER_STATE_LENGTH_1" << std::endl;
                packet->payloadLength += (c << 8);

                if(packet->payloadLength > payloadSize)
                {
                    return false;
                }

                payloadCounter = packet->payloadLength;
                if(packet->payloadLength > 0)
                {
                    state = DECODER_STATE_PAYLOAD;
                }
                else
                {
                    state = DECODER_STATE_CHECKSUM_0;
                }
                break;

            case DECODER_STATE_PAYLOAD:
                //std::cout << "DECODER_STATE_PAYLOAD" << std::endl;
                packet->payload[packet->payloadLength - payloadCounter--] = c;
                if(payloadCounter == 0)
                {
                    state = DECODER_STATE_CHECKSUM_0;
                }
                break;

            case DECODER_STATE_CHECKSUM_0:
                //std::cout << "DECODER_STATE_CHECKSUM_0" << std::endl;
                packet->checksum = c;
                state = DECODER_STATE_CHECKSUM_1;
                break;

            case DECODER_STATE_CHECKSUM_1:
                //std::cout << "DECODER_STATE_CHECKSUM_1" << std::endl;
                packet->checksum += (c << 8);
                state = DECODER_STATE_PREAMBLE_0;
                isWholePacket = true;
                //std::cout << std::endl << "Received Packet, Took " << std::dec << std::chrono::duration_cast<std::chrono::milliseconds>(d).count() << "ms" << std::endl;
                break;
            }

            //usleep(10);
        }
        //usleep(10);
    }
#endif // CONNECTION
    return isWholePacket;
}

bool Protocol::sendMessage(packet_t *packet)
{
    uint8_t msg[maxMsgSize];

    memcpy(msg, packet, 6);
    memcpy(msg + 6, packet->payload, packet->payloadLength);
    memcpy(msg + 6 + packet->payloadLength, &packet->checksum, 2);

    /*
    std::cout << "RAW: ";
    for(uint16_t i = 0; i < 6 + packet->payloadLength + 2; ++i)
    {
        int n = msg[i];
        //std::cout << "x" << std::hex << n << " ";
    }
    std::cout << std::endl;
    */

    m_port.send(msg, 6 + packet->payloadLength + 2);
    return true;
}

bool Protocol::checkPacket(const packet_t *packet) const
{
    uint16_t cs = calculateCheckSum(packet);
    return (cs == packet->checksum);
}

uint16_t Protocol::calculateCheckSum(const packet_t *packet) const
{
    uint8_t a = 0;
    uint8_t b = 0;

    a = a + (packet->messageId & 0xFF);
    b = b + a;

    a = a + (packet->messageId >> 8);
    b = b + a;

    a = a + (packet->payloadLength & 0xFF);
    b = b + a;

    a = a + (packet->payloadLength >> 8);
    b = b + a;

    for(uint16_t i = 0; i < packet->payloadLength; ++i)
    {
        a = a + packet->payload[i];
        b = b + a;
    }

    return (b << 8) + a;
}
