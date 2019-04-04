//
//  parser.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "parser.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
#include <map>
#include <string>
#include "zmq.h"

#include "atlas_types.h"
#include "config.h"
#include "../protocol/protocol.h"
#include "../protocol/protocol_structures.h"

typedef enum
{
    CFG_ROLE_LISTENER = 0,
    CFG_ROLE_TAG,
    CFG_ROLE_ANCHOR,
    CFG_ROLE_TAG_TDOA,
    CFG_ROLE_NUM_MODES,
    CFG_ROLE_SYNC_ANCHOR
} cfgRole_t;

Parser::Parser()
{

}

Parser::~Parser()
{

}

void Parser::initialize(const configParser_t &p, configTags_t &t)
{
    std::cout << "Initializing Parser..." << std::endl;
    std::cout << "Whitelist: ";
    for (auto it = p.tagWhitelist.begin(); it != p.tagWhitelist.end(); it++)
    {
        std::cout << std::hex << it->first << ",";
    }
    std::cout << std::endl;

    #if ALGORITHM == TOA
    std::cout << "Tag Configuration:" << std::endl;

    // for now, manually written.
    t.ports.insert(std::pair<uint64_t, std::string>(0xdeca010000000001, "tcp://192.168.0.27:6050"));

    for (auto it = t.ports.begin(); it != t.ports.end(); ++it)
    {
        m_tags.insert(std::make_pair(it->first, new protocol::Protocol(it->second, ZMQ_REP)));
    }

    /* No init exchange coded so far
    for (auto it = m_tags.begin(); it != m_tags.end(); it++)
    {
        std::cout << "Waiting for initialization message from eui: " << it->first << std::endl;
        //as config will be overwritten below again and sent to the sync node, the sendMsgPoll does not make sense and is not conducted so far
        //protocol::msgConfig_t config;
        //it->second->sendMsgPoll(&config);
        //std::cout << it->first << " Config: " << std::hex << (int)config.role << ", " << (uint64_t)config.eui << ", " << (int)config.channel << std::endl;

        //unused in ATLAS, hence not conducted so far
        //protocol::msgPeriod_t period;
        //it->second->sendMsgPoll(&period);
        //std::cout << it->first << " Periods: " << std::hex << (int)period.syncPeriod << ", " << (int)period.tagPeriod << std::endl;

        config.role = CFG_ROLE_ANCHOR;
        //std::string::size_type sz = 0;
        //config.eui = std::stoull(it->first, &sz, 16);
        config.eui = it->first;
        config.channel = 0x20;
        //std::cout << it->first << " configured to role ANCHOR and channel" << (int)config.channel << std::endl;

        //as all the BaseStations are connected as Replyer, the BaseStations shall be turned on one after each other. The ATLAS OSS shall wait here for the
        //corresponding connection configured in the whitelist of the configuration and respond with the above written configuration.
        it->second->sendReplyMsg(config);

        usleep(10000);
    }
    */

    #endif // ALGORITHM

    m_tagWhitelist = p.tagWhitelist;
}

void Parser::fillUintFromBuffer(uint64_t *var, uint8_t *buff, uint8_t length)
{
    if(length == 64)
    {
        uint32_t temp = 0;
        temp =   buff[3];
        temp += (buff[2] << 8);
        temp += (buff[1] << 16);
        temp += (buff[0] << 24);
        *var = temp;
        *var = (*var << 32);
        *var += buff[7];
        *var += (buff[6] << 8);
        *var += (buff[5] << 16);
        *var += (buff[4] << 24);
    }
    else if(length == 32)
    {
        *var = (buff[0] << 24);
        *var += (buff[1] << 16);
        *var += (buff[2] << 8);
        *var += buff[3];
    }
    else if(length == 16)
    {
        *var = (buff[0] << 8);
        *var += buff[1];
    }
    else if (length == 8)
    {
        *var = buff[0];
    }
    else
    {
        // no valid length
    }
}

void Parser::poll()
{
    for (auto it = m_tags.begin(); it != m_tags.end(); ++it)
    {
        protocol::packet_t packet;
        if(it->second->poll(&packet))
        {
            protocol::msgDistance_t msg;
            fillUintFromBuffer(&msg.txId, &packet.payload[POS_TARGETEUI], 64);
            fillUintFromBuffer((uint64_t *)&msg.seqNr, &packet.payload[POS_SEQNUM], 32);
            for(int i = 0; i < NUM_ANCHORS; i++)
            {
                fillUintFromBuffer(&msg.rxId[i], &packet.payload[POS_ANCHOREUIS + (i*8)], 64);
            }
            memcpy(&msg.distance, &packet.payload[POS_DISTANCES], (8 * NUM_ANCHORS));

            if(m_tagWhitelist.find(msg.txId) != m_tagWhitelist.end())
            {
                measurement_t meas;
                meas.hts = std::chrono::system_clock::now();
                meas.txeui = msg.txId;
                meas.seq = msg.seqNr;
                for(int i = 0; i < NUM_ANCHORS; i++)
                {
                    meas.rxeui = msg.rxId[i];
                    meas.distance = msg.distance[i];
                    m_measurements[meas.txeui][meas.seq].insert(std::make_pair(meas.rxeui, meas));
                }
            }
            else
            {
                std::cout << "Warning: " << msg.txId << " not in Whitelist " << std::endl;
            }
        }
    }

    usleep(100);
}

void Parser::extractSample(sample_t *sample)
{
    // go through measurements and schedule latest
    auto now = std::chrono::system_clock::now();

    // go through txeuis
    for (auto it_txeui = m_measurements.begin(); it_txeui != m_measurements.end(); ++it_txeui)
    {
        // go through sequence numbers
        for (auto it_seq = it_txeui->second.begin(); it_seq != it_txeui->second.end(); ++it_seq)
        {
            // go through rxeuis
            for (auto it_rxeui = it_seq->second.begin(); it_rxeui != it_seq->second.end(); ++it_rxeui)
            {
                sample->hts = now;
                sample->txeui = it_txeui->first;
                sample->seq = it_seq->first;
                sample->meas = it_seq->second;

                // storage to sampe is finished. Data structure can be erased.
                //m_measurements.erase(it_txeui);
                it_txeui->second.erase(it_seq);
                // Break to not iterate with an invalid iterator
                break;
            }
        }
    }
}



/*
// calculate measurement offset for 0-Point reference at {0, 0, 0}
void ClockModel::measure_ZeroRefOffset(double measured_distance)
{
    arma::mat BSposxyz_quad(1,3);
    arma::mat BSposxyz_sqrt(1,1);
    double ref_distance = 0;

    //Calculate the distance between 0-Point and BaseStation which is the reference distance for the measurement
    BSposxyz_quad(0,0) = positionXYZ(0);
    BSposxyz_quad(0,1) = positionXYZ(1);
    BSposxyz_quad(0,2) = positionXYZ(2);
    BSposxyz_quad = arma::square(BSposxyz_quad);
    BSposxyz_sqrt(0,0) = (BSposxyz_quad(0,0) + BSposxyz_quad(0,1));
    BSposxyz_sqrt = arma::sqrt(BSposxyz_sqrt);
    ref_distance = BSposxyz_sqrt(0,0);

    // If the measurement is unplausible, skip it for the offset calculation as it would falsify it
    if((measured_distance > (ref_distance + MAX_OFFSETERROR)) || (measured_distance < (ref_distance - MAX_OFFSETERROR)))
    {
        std::cout << "Mesurement skipped for Offset calculation. " << std::endl;
    }
    else
    {

        errors[error_cntr] = (measured_distance - ref_distance);
        std::cout << "errors[" << error_cntr << "] = " << errors[error_cntr] << std::endl;
        error_cntr++;
    }
}

void ClockModel::calculate_ZeroRefOffset(void)
{
    int cntrneg = 0;
    int devidecntr = 0;

    for(int i = 0; i < error_cntr; i++)
    {
        if(errors[i] < 0)
        {
            cntrneg++;
        }
    }


    for(int i = 0; i < error_cntr; i++)
    {
        // if more negative offsets are existing
        if(cntrneg > (error_cntr - cntrneg))
        {
            devidecntr = cntrneg;
            if(errors[i] < 0)
            {
                ZeroRefOffset = (ZeroRefOffset + errors[i]);
            }
            else
            {
                // Skip, as >= 0
            }
        }
        else
        {
            devidecntr = (error_cntr - cntrneg);
            if(errors[i] > 0)
            {
                ZeroRefOffset = (ZeroRefOffset + errors[i]);
            }
            else
            {
                // Skip, as <= 0
            }
        }
    }

    for(int i = 0; i < error_cntr; i++)
    {
        ZeroRefOffset = (ZeroRefOffset + errors[i]);
    }
    ZeroRefOffset = (ZeroRefOffset / error_cntr);

    ZeroRefOffset = (ZeroRefOffset / devidecntr);
    std::cout << "ZeroRefOffset = " << ZeroRefOffset << std::endl;
}


void ClockModel::init(arma::vec pos)
{
    positionXYZ = pos;
}
*/
