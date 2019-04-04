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

#if POSITIONER == TOA
void Parser::initialize(const configParser_t &p, const configAnchors_t &a, configTags_t &t, const configSync_t &s)
#elif POSITIONER == TDOA
void Parser::initialize(const configParser_t &p, const configAnchors_t &a, const configSync_t &s)
#endif // POSITIONER
{
    std::cout << "Initializing Parser..." << std::endl;
    std::cout << "Whitelist: ";
    for (auto it = p.tagWhitelist.begin(); it != p.tagWhitelist.end(); it++)
    {
        std::cout << std::hex << it->first << ",";
    }
    std::cout << std::endl;
    //std::cout << "Sync: " << s.port << "," << std::dec << s.baudrate << "," << std::hex << s.eui << "," << std::dec << s.interval << std::endl;

    std::cout << "Sync Configuration:" << std::endl;
    #if CONNECTION == ETH_ZMQ
    protocol::Protocol *sync = new protocol::Protocol(s.port, s.zmq_connection_type);

    protocol::msgPeriod_t period;
    period.syncPeriod = 100; //every 100ms a Sync Frame shall be sent to the BaseStations
    period.tagPeriod = 200; //not used for SyncNode
    //std::cout << std::hex << s.eui << " Periods: " << (int)period.syncPeriod << ", " << (int)period.tagPeriod << std::endl;

    //send config to SyncNode, wait for response from SyncNode and validate the response as soon as received
    sync->sendMsg(period);

    //usleep(1000000);

    protocol::msgConfig_t config;
    config.role = CFG_ROLE_SYNC_ANCHOR;
    config.eui = s.eui;
    config.channel = 0x20; //actually unused
    //std::cout << s.eui << " configured to role SYNC_ANCHOR and channel" << (int)config.channel << std::endl;

    //send config to SyncNode, wait for response from SyncNode and validate the response as soon as received
    sync->sendMsg(config);

    #elif CONNECTION == SER_USB
    protocol::Protocol *sync = new protocol::Protocol(s.port);
    protocol::msgConfig_t config;
    sync->sendMsgPoll(&config);
    //std::cout << std::hex << s.eui << " Config: " << (int)config.role << ", " << (uint64_t)config.eui << ", " << (int)config.channel << std::endl;

    protocol::msgPeriod_t period;
    sync->sendMsgPoll(&period);
    //std::cout << std::hex << s.eui << " Periods: " << (int)period.syncPeriod << ", " << (int)period.tagPeriod << std::endl;

    config.role = CFG_ROLE_SYNC_ANCHOR;
    config.eui = s.eui;
    config.channel = 0x20;
    sync->sendMsg(config);

    //std::cout << s.eui << " configured to role SYNC_ANCHOR and channel" << (int)config.channel << std::endl;

    #endif // CONNECTION
    usleep(10000);

    std::cout << "Anchor Configuration:" << std::endl;
    #if CONNECTION == ETH_ZMQ
    for (auto it = a.ports.begin(); it != a.ports.end(); ++it)
    {
        m_decoders.insert(std::make_pair(it->first, new protocol::Protocol(it->second, ZMQ_REP)));
    }
    for (auto it = m_decoders.begin(); it != m_decoders.end(); it++)
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

    #elif CONNECTION == SER_USB
    for (auto it = a.ports.begin(); it != a.ports.end(); ++it)
    {
        m_decoders.insert(std::make_pair(it->first, new protocol::Protocol(it->second)));
    }
    for (auto it = m_decoders.begin(); it != m_decoders.end(); it++)
    {
        protocol::msgConfig_t config;
        it->second->sendMsgPoll(&config);
        //std::cout << it->first << " Config: " << std::hex << (int)config.role << ", " << (uint64_t)config.eui << ", " << (int)config.channel << std::endl;

        protocol::msgPeriod_t period;
        it->second->sendMsgPoll(&period);
        //std::cout << it->first << " Periods: " << std::hex << (int)period.syncPeriod << ", " << (int)period.tagPeriod << std::endl;

        config.role = CFG_ROLE_ANCHOR;
        std::string::size_type sz = 0;
        //config.eui = std::stoull(it->first, &sz, 16);
        config.eui = it->first;
        config.channel = 0x20;
        it->second->sendMsg(config);

        //std::cout << it->first << " configured to role ANCHOR and channel" << (int)config.channel << std::endl;
        usleep(10000);
    }

    #endif // CONNECTION

    #if ALGORITHM == TOA
    std::cout << "Tag Configuration:" << std::endl;
    #if CONNECTION == ETH_ZMQ
    // for now, manually written.
    t.ports.insert(std::pair<uint64_t, std::string>(0xdeca010000000001, "tcp://192.168.0.27:6050"));
    //m_tags.insert(std::pair<uint64_t, protocol::Protocol>(0xdeca010000000001, new protocol::Protocol("tcp://192.168.0.27:6050", ZMQ_REP)));
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

    #elif CONNECTION == SER_USB
    // not supported, not tested!
    assert(FALSE);
    for (auto it = a.ports.begin(); it != a.ports.end(); ++it)
    {
        m_decoders.insert(std::make_pair(it->first, new protocol::Protocol(it->second)));
    }
    for (auto it = m_decoders.begin(); it != m_decoders.end(); it++)
    {
        protocol::msgConfig_t config;
        it->second->sendMsgPoll(&config);
        //std::cout << it->first << " Config: " << std::hex << (int)config.role << ", " << (uint64_t)config.eui << ", " << (int)config.channel << std::endl;

        protocol::msgPeriod_t period;
        it->second->sendMsgPoll(&period);
        //std::cout << it->first << " Periods: " << std::hex << (int)period.syncPeriod << ", " << (int)period.tagPeriod << std::endl;

        config.role = CFG_ROLE_ANCHOR;
        std::string::size_type sz = 0;
        //config.eui = std::stoull(it->first, &sz, 16);
        config.eui = it->first;
        config.channel = 0x20;
        it->second->sendMsg(config);

        //std::cout << it->first << " configured to role ANCHOR and channel" << (int)config.channel << std::endl;
        usleep(10000);
    }

    #endif // CONNECTION
    #endif // ALGORITHM

    m_tagWhitelist = p.tagWhitelist;
}


void Parser::poll()
{
    for (auto it = m_decoders.begin(); it != m_decoders.end(); ++it)
    {
        protocol::packet_t packet;
        if(it->second->poll(&packet))
        {
            if (it->second->calculateCheckSum(&packet) != packet.checksum)
            {
                std::cout << it->first << " CHECKSUM ERROR!!! ";
                std::cout << std::hex << "x" << packet.messageId << ", x" << packet.payloadLength << ", x" << packet.checksum << ", x" << it->second->calculateCheckSum(&packet);
                std::cout << " payload:";
                for(uint16_t i = 0; i < packet.payloadLength; ++i)
                {
                    std::cout << std::hex << " x" << (int)packet.payload[i];
                }
                std::cout << std::endl;
            }
            else
            {
                protocol::msgToa_t msg;
                memcpy(&msg, packet.payload, sizeof(protocol::msgToa_t));

                //std::cout << it->first << " Received packet ";
                //std::cout << std::hex << msg.txId << ", " << msg.rxId << ", x" << msg.rxTs << ", x" << (int)msg.seqNr << std::endl;

                // Linearize receive Timestamps
                uint64_t ts = msg.rxTs;
                uint64_t rxeui = msg.rxId;

                if(m_lastTimestamp.find(rxeui) == m_lastTimestamp.end())
                {
                    m_lastTimestamp.insert(std::make_pair(rxeui, ts));
                }
                else
                {
/*
                    int64_t lastts = m_lastTimestamp[rxeui];
                    int64_t res = lastts % ticksPerRevolution;

                    if((int64_t)ts - (int64_t)res > 0)
                    {
                        m_lastTimestamp[rxeui] = lastts + (ts - res);
                        //std::cout << "+ ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                    if((int64_t)ts - (int64_t)res < 0)
                    {
                        m_lastTimestamp[rxeui] = lastts + (ts - res) + ticksPerRevolution;
                        //std::cout << "- ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
*/
                    m_lastTimestamp[rxeui] = ts;
                }

                if(m_tagWhitelist.find(msg.txId) != m_tagWhitelist.end())
                {
                    #if POSITIONER == TOA
                    // Differentiate between the first position-frame from the target and the second with valid time stamp
                    // first ts from target will have 0xFFFFFFFFFFFFFFFF inserted.
                    // ts from SyncNode will always have 0x0000000000000000 inserted.
                    #if CLOCKCORR == ATLASEXT
                    if(msg.txTS == 0xFFFFFFFFFFFFFFFF)
                    #elif CLOCKCORR == ATLAS
                    if((msg.txTS == 0xFFFFFFFFFFFFFFFF) || (msg.txTS == 0x0000000000000000))
                    #endif // CLOCKCORR
                    {
                        measurement_t meas;
                        meas.hts = std::chrono::system_clock::now();
                        meas.ts = m_lastTimestamp[rxeui];
                        meas.ts1 = msg.txTS;
                        meas.txeui = msg.txId;
                        meas.rxeui = msg.rxId;
                        meas.seq = msg.seqNr;
                        meas.fpPower = 0;
                        meas.rxPower = 0;
                        meas.fpRatio = 0;

                        m_measurements[meas.txeui][meas.seq].insert(std::make_pair(meas.rxeui, meas));
                    }
                    else
                    {
                        try
                        {
                            m_measurements[msg.txId][msg.seqNr].at(msg.rxId).ts1 = msg.txTS;
                        }
                        catch(std::out_of_range & e)
                        {
                            std::cout << "Sequence Number " << std::hex << msg.seqNr << " not found. Skipping measurement." << e.what() << std::endl;
                        }
                        catch(std::exception & e)
                        {
                            std::cout << "Sequence Number " << std::hex << msg.seqNr << " not found. Skipping measurement." << e.what() << std::endl;
                        }
                    }

                    #elif POSITIONER == TDOA
                    measurement_t meas;
                    meas.hts = std::chrono::system_clock::now();
                    meas.ts = m_lastTimestamp[rxeui];
                    meas.ts1 = msg.txTS;
                    meas.txeui = msg.txId;
                    meas.rxeui = msg.rxId;
                    meas.seq = msg.seqNr;
                    meas.fpPower = 0;
                    meas.rxPower = 0;
                    meas.fpRatio = 0;

                    m_measurements[meas.txeui][meas.seq].insert(std::make_pair(meas.rxeui, meas));

                    #endif // POSITIONER

                }
                else
                {
                    std::cout << "Warning: " << msg.txId << " not in Whitelist " << std::endl;
                }
            }
        }
    }

    for (auto it = m_tags.begin(); it != m_tags.end(); ++it)
    {
        protocol::packet_t packet;
        if(it->second->poll(&packet))
        {
            if (it->second->calculateCheckSum(&packet) != packet.checksum)
            {
                std::cout << it->first << " CHECKSUM ERROR!!! ";
                std::cout << std::hex << "x" << packet.messageId << ", x" << packet.payloadLength << ", x" << packet.checksum << ", x" << it->second->calculateCheckSum(&packet);
                std::cout << " payload:";
                for(uint16_t i = 0; i < packet.payloadLength; ++i)
                {
                    std::cout << std::hex << " x" << (int)packet.payload[i];
                }
                std::cout << std::endl;
            }
            else
            {
                protocol::msgToa_t msg;
                memcpy(&msg, packet.payload, sizeof(protocol::msgToa_t));

                //std::cout << it->first << " Received packet ";
                //std::cout << std::hex << msg.txId << ", " << msg.rxId << ", x" << msg.rxTs << ", x" << (int)msg.seqNr << std::endl;

                // Linearize receive Timestamps
                uint64_t ts = msg.rxTs;
                uint64_t rxeui = msg.rxId;

                if(m_lastTimestamp.find(rxeui) == m_lastTimestamp.end())
                {
                    m_lastTimestamp.insert(std::make_pair(rxeui, ts));
                }
                else
                {

                    int64_t lastts = m_lastTimestamp[rxeui];
                    int64_t res = lastts % ticksPerRevolution;

                    if((int64_t)ts - (int64_t)res > 0)
                    {
                        m_lastTimestamp[rxeui] = lastts + (ts - res);
                        //std::cout << "+ ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }
                    if((int64_t)ts - (int64_t)res < 0)
                    {
                        m_lastTimestamp[rxeui] = lastts + (ts - res) + ticksPerRevolution;
                        //std::cout << "- ts:" << ts << " lastts:" << lastts << " res:" << res << std::endl;
                    }

                    m_lastTimestamp[rxeui] = ts;
                }

                if(m_tagWhitelist.find(msg.txId) != m_tagWhitelist.end())
                {
                    #if POSITIONER == TOA
                    // Differentiate between the first position-frame from the target and the second with valid time stamp
                    // first ts from target will have 0xFFFFFFFFFFFFFFFF inserted.
                    // ts from SyncNode will always have 0x0000000000000000 inserted.
                    #if CLOCKCORR == ATLASEXT
                    if(msg.txTS == 0xFFFFFFFFFFFFFFFF)
                    #elif CLOCKCORR == ATLAS
                    if((msg.txTS == 0xFFFFFFFFFFFFFFFF) || (msg.txTS == 0x0000000000000000))
                    #endif // CLOCKCORR
                    {
                        measurement_t meas;
                        meas.hts = std::chrono::system_clock::now();
                        meas.ts = m_lastTimestamp[rxeui];
                        meas.ts1 = msg.txTS;
                        meas.txeui = msg.txId;
                        meas.rxeui = msg.rxId;
                        meas.seq = msg.seqNr;
                        meas.fpPower = 0;
                        meas.rxPower = 0;
                        meas.fpRatio = 0;

                        m_measurements[meas.txeui][meas.seq].insert(std::make_pair(meas.rxeui, meas));
                    }
                    else
                    {
                        try
                        {
                            m_measurements[msg.txId][msg.seqNr].at(msg.rxId).ts1 = msg.txTS;
                        }
                        catch(std::out_of_range & e)
                        {
                            std::cout << "Sequence Number " << std::hex << msg.seqNr << " not found. Skipping measurement." << e.what() << std::endl;
                        }
                        catch(std::exception & e)
                        {
                            std::cout << "Sequence Number " << std::hex << msg.seqNr << " not found. Skipping measurement." << e.what() << std::endl;
                        }
                    }

                    #elif POSITIONER == TDOA
                    measurement_t meas;
                    meas.hts = std::chrono::system_clock::now();
                    meas.ts = m_lastTimestamp[rxeui];
                    meas.ts1 = msg.txTS;
                    meas.txeui = msg.txId;
                    meas.rxeui = msg.rxId;
                    meas.seq = msg.seqNr;
                    meas.fpPower = 0;
                    meas.rxPower = 0;
                    meas.fpRatio = 0;

                    m_measurements[meas.txeui][meas.seq].insert(std::make_pair(meas.rxeui, meas));

                    #endif // POSITIONER

                }
                else
                {
                    std::cout << "Warning: " << msg.txId << " not in Whitelist " << std::endl;
                }
            }
        }
    }

    usleep(100);
}

void Parser::extractSamples(std::vector<sample_t> *samples)
{
    // go through measurements and schedule latest
    auto now = std::chrono::system_clock::now();

    // go through txeuis
    for (auto it_txeui = m_measurements.begin(); it_txeui != m_measurements.end(); ++it_txeui)
    {
        #if POSITIONER == TOA
        uint16_t num_msgs = 0;
        if(it_txeui->first == 0xdeca030000000001)
        {
            // if the sender equals the SyncNode, all Targets and all BaseStations need to receive the brodcast
            num_msgs = (m_decoders.size() + m_tags.size());
        }
        else
        {
            // if the sender doesnt equal the SyncNode, only the BaseStations need to receive the brodcast
            num_msgs = m_decoders.size();
        }
        #endif // POSITIONER

        // go through sequence numbers
        for (auto it_seq = it_txeui->second.begin(); it_seq != it_txeui->second.end(); ++it_seq)
        {
            bool dispatch = false;
            #if CONNECTION == ETH_ZMQ
            int i = 0;
            #endif // CONNECTION

            // go through rxeuis
            for (auto it_rxeui = it_seq->second.begin(); it_rxeui != it_seq->second.end(); ++it_rxeui)
            {

                #if CONNECTION == ETH_ZMQ
                // If the sequence number is filled out for the cooresponding rxeui, it is assumed, that
                // the message from the txeui and this seqence number is received
                if((it_rxeui->second.seq == it_seq->first) && (it_rxeui->second.ts1 != 0xFFFFFFFFFFFFFFFF))
                {
                    // Increment i so that it can be validated, that all rxeui were received for the sequence
                    // number of the txeui
                    i++;
                    #if POSITIONER == TOA
                    if(i >= num_msgs)
                    #elif POSITIONER == TDOA
                    if(i >= m_decoders.size())
                    #endif // POSITIONER
                    {
                        dispatch = true;
                    }
                }
                else
                {
                    // Do nothing and leave i at current value
                    dispatch = false;
                }
                #elif CONNECTION == SER_USB
                // go through
                auto d = now - it_rxeui->second.hts;
                if(d > std::chrono::milliseconds(10))
                {
                    dispatch = true;
                }
                #endif // CONNECTION
            }

            if(dispatch)
            {
                // Linearize sequence number
                int64_t seq = it_seq->first;
                uint64_t txeui = it_txeui->first;

                if(m_lastSequence.find(txeui) == m_lastSequence.end())
                {
                    m_lastSequence.insert(std::make_pair(txeui, seq));
                }
                else
                {
                    int64_t lastseq = m_lastSequence[txeui];
                    int64_t res = lastseq % 256;

                    if(seq - res > 0)
                    {
                        m_lastSequence[txeui] = lastseq + (seq - res);
                        //std::cout << "+ seq:" << seq << " lastseq:" << lastseq << " res:" << res << std::endl;
                    }
                    if(seq - res < 0)
                    {
                        m_lastSequence[txeui] = lastseq + (seq - res) + 256;
                        //std::cout << "- seq:" << seq << " lastseq:" << lastseq << " res:" << res << std::endl;
                    }
                }

                //std::cout << "Dispatching sample " << txeui << " seq:" << m_lastSequence[txeui] << " size:" << it_seq->second.size() << std::endl;

                sample_t s;
                s.hts = now;
                s.txeui = txeui;
                s.seq = m_lastSequence[txeui];
                s.meas = it_seq->second;

                it_txeui->second.erase(it_seq);
                samples->push_back(s);

                #if CONNECTION == ETH_ZMQ
                // Break to not iterate with an invalid iterator (it_seq)
                break;

                /*
                // erase() responds with the iterator to the next element after the erased one
                it_seq = it_txeui->second.erase(it_seq);

                it_seq = it_txeui->second.begin();

                //std::cout << it_txeui->second.end() << " --- " << it_temp << " --- " << it_seq << std::endl;

                if(it_seq != it_temp)
                {
                    break;
                }
                else
                {
                    // As the for loop will ++it_seq and hence might skip the next element after the erased one (as already set on it_seq)
                    --it_seq;
                }
*/

                #elif CONNECTION == SER_USB
                it_txeui->second.erase(it_seq);

                #endif // CONNECTION
            }
        }
    }
}


ClockModel::ClockModel ()
{
    m_lastSync = 0;
    m_lastOffset = 0;
    m_lastDrift = 0;
}

ClockModel::~ClockModel()
{

}

#if CLOCKCORR == ATLASEXT
void ClockModel::processSynchronizationFrame(arma::vec pos_tx, arma::vec pos_rx, uint64_t txts, uint64_t rxts)
{
    arma::mat t_delta_xyz(1,3);
    arma::mat t_delta_square_xyz(1,3);
    arma::mat t_root(1,1);

    double rxts_sec = (double)rxts / ((double)ticksPerSecond);
    double txts_sec = (double)txts / ((double)ticksPerSecond);
    double distance_txrx = 0;
    //std::cout << "rxts_sec: " << rxts_sec << std::endl;
    //std::cout << "txts_sec: " << txts_sec << std::endl;


    t_delta_xyz(0,0) = pos_rx(0) - pos_tx(0);
    t_delta_xyz(0,1) = pos_rx(1) - pos_tx(1);
    t_delta_xyz(0,2) = pos_rx(2) - pos_tx(2);

    t_delta_square_xyz = arma::square(t_delta_xyz);

    // differenciate between 2- and 3-dimensional space for calculation
    // if x between Sender (tx) and Receiver (rx) equal to 0
    if(t_delta_xyz(0,0) == 0)
    {
        // take y and z for calculation
        t_root(0,0) = t_delta_square_xyz(0,1) + t_delta_square_xyz(0,2);
        t_root = arma::sqrt(t_root);
        distance_txrx = t_root(0,0);
    }

    // if y between Sender (tx) and Receiver (rx) equal to 0
    else if(t_delta_xyz(0,1) == 0)
    {
        // take x and z for calculation
        t_root(0,0) = t_delta_square_xyz(0,0) + t_delta_square_xyz(0,2);
        t_root = arma::sqrt(t_root);
        distance_txrx = t_root(0,0);
    }

    // if z between Sender (tx) and Receiver (rx) equal to 0
    else if(t_delta_xyz(0,2) == 0)
    {
        // take x and y for calculation
        t_root(0,0) = t_delta_square_xyz(0,0) + t_delta_square_xyz(0,1);
        t_root = arma::sqrt(t_root);
        distance_txrx = t_root(0,0);
    }

    // 3-dimensional space for calculation as x, y and z are not equal to 0
    else
    {
        // calculate diagonal of x and y
        t_root(0,0) = t_delta_square_xyz(0,0) + t_delta_square_xyz(0,1);
        t_root = arma::sqrt(t_root);
        t_root = arma::square(t_root);

        // take calculated diagonal and z for calculation
        t_root(0,0) = t_root(0,0) + t_delta_square_xyz(0,2);
        t_root = arma::sqrt(t_root);
        distance_txrx = t_root(0,0);
    }
    //std::cout << "distance_txrx: " << distance_txrx << std::endl;
    //double temp = (distance_txrx / SPEED_OF_LIGHT);
    //temp = (txts_sec + temp);
    double rxts_corr = rxts_sec - (txts_sec + (distance_txrx / SPEED_OF_LIGHT));
    //std::cout << "rxts_corr: " << rxts_corr << std::endl;

    //double drift = (rxts_corr - m_lastOffset) * syncFrequency;

    //std::cout <<"Offset: " << rxts_corr*1000 << std::setprecision(2) << std::fixed << "ms" << std::endl;
    //, drift: " << drift*1000000000 << "ns/s" << std::endl;

    m_lastSync = rxts_sec - rxts_corr;
    m_lastOffset = rxts_corr;
    //m_lastDrift = drift;
}

double ClockModel::getCorrectedTOA(uint64_t ts)
{
    double ts_sec = (double)ts / ((double)ticksPerSecond);
    //double td = toa - m_lastSync;
    //double correction = m_lastOffset + m_lastDrift * td;
    double correction = m_lastOffset;
    double corrected = ts_sec - correction;

    // if the correction would result in a negative time, the systemcolock of one participant was overflowing.
    // in this case, the negative value needs to be substracted from the maximum system time to get the correct global time.
    if(corrected < 0)
    {
        corrected = (double)MAXSYSTIME_DW1000 + corrected;
    }
    else if(corrected >= (double)MAXSYSTIME_DW1000)
    {
        corrected = corrected - (double)MAXSYSTIME_DW1000;
    }

    //std::cout << "Loc - toa: " << std::setprecision(3) << std::fixed << toa;
    //std::cout << std::setprecision(2) << std::fixed << "s, td: " << td*1000;
    //std::cout << std::setprecision(2) << std::fixed << "ms, drift*td: " << m_lastDrift*td*1000000000;
    //std::cout << std::setprecision(9) << std::fixed << "ns, corrected: " << corrected << std::endl;

    return corrected;
}


#elif CLOCKCORR == ATLAS
void ClockModel::processSynchronizationFrame(uint64_t seq, uint64_t ts)
{
    double syncFrequency = (double)ticksPerSecond / ticksPerInterval; //equals 10Hz
    #if POSITIONER == TOA
    double ref = (double)seq * ((1 / syncFrequency));// - 0.00015); //Compensate the error of the syncFrame-Timer
    #elif POSITIONER == TDOA
    double ref = (double)seq * (1 / syncFrequency);
    #endif // POSITIONER
    // ref will equal seq*100ms. Hence, Sync Node will periodically send 100ms syncs to all BaseStations.
    double toa = (double)ts / ticksPerSecond;
    double offset = toa - ref;
    #if POSITIONER == TOA
    double drift = (offset - m_lastOffset) * syncFrequency;//(1 / ref);//Compensate the error of the of the syncFrame-Timer
    #elif POSITIONER == TDOA
    double drift = (offset - m_lastOffset) * syncFrequency;
    #endif // POSITIONER

    //std::cout << "Sync - reference: " << std::setprecision(2) << std::fixed << ref;
    //std::cout << std::setprecision(2) << std::fixed << "s, offset: " << offset*1000;
    //std::cout << std::setprecision(2) << std::fixed << "ms, drift: " << drift*1000000000 << "ns/s" << std::endl;

    m_lastSync = toa;
    m_lastOffset = offset;
    m_lastDrift = drift;
}

double ClockModel::getCorrectedTOA(uint64_t ts)
{
    double toa = (double)ts / ticksPerSecond;
    double td = toa - m_lastSync;
    double correction = m_lastOffset + m_lastDrift * td;
    double corrected = toa - correction;

    //std::cout << "Loc - toa: " << std::setprecision(3) << std::fixed << toa;
    //std::cout << std::setprecision(2) << std::fixed << "s, td: " << td*1000;
    //std::cout << std::setprecision(2) << std::fixed << "ms, drift*td: " << m_lastDrift*td*1000000000;
    //std::cout << std::setprecision(9) << std::fixed << "ns, corrected: " << corrected << std::endl;

    return corrected;
}

#endif // CLOCKCORR


ClockCorrection::ClockCorrection ()
{

}

ClockCorrection::~ClockCorrection()
{

}

#if ((POSITIONER == TOA) || (CLOCKCORR == ATLASEXT))
void ClockCorrection::initialize(const configAnchors_t &a, configTags_t &t, const configSync_t &s)
{
    // copy all anchor position information
    m_RxPositions = a.positions;

    // copy all tag initial position information
    for(auto it = t.positions.begin(); it != t.positions.end(); ++it)
    {
        m_RxPositions.insert(std::make_pair(it->first, it->second));
    }

    #if CLOCKCORR == ATLASEXT
    // copy syncNode position information
    // Runtime Abort here... TBD
    m_TxPosition = s.position;
    #endif // CLOCKCORR
}
#endif

#if CLOCKCORR == ATLASEXT
void ClockCorrection::updateTagPosition(uint64_t tageui, arma::vec tagPosition)
{
    m_RxPositions.at(tageui) = tagPosition;
}

#endif // CLOCKCORR


void ClockCorrection::processSample(sample_t sample)
{
    static int config_finished = 0;

    if(sample.txeui == syncEUI)
    {
        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            if(m_anchorClocks.find(it->first) == m_anchorClocks.end())
            {
                ClockModel clock;
                m_anchorClocks.insert(std::make_pair(it->first, clock));
                #if POSITIONER == TOA
                if((it->first) != 0xdeca010000000001)
                {
                    m_anchorClocks.at(it->first).init(m_RxPositions.at(it->first));
                }
                else
                {
                    // Do nothing
                    // For the target with eui 0xdeca010000000001 position [0, 0, 0] is assumed
                }
                #endif // POSITIONER
            }
            #if CLOCKCORR == ATLASEXT
            m_anchorClocks[it->first].processSynchronizationFrame(m_TxPosition, m_RxPositions.at(it->second.rxeui), it->second.ts1, it->second.ts);
            #elif CLOCKCORR == ATLAS
            m_anchorClocks[it->first].processSynchronizationFrame(sample.seq, it->second.ts);
            #endif // CLOCKCORR
        }
        m_lastSync = sample.hts;
        m_lastSequence = sample.seq;
        if((int64_t)sample.seq - (int64_t)m_lastSequence > 1)
        {
            std::cout << "Warning: Missed sync packet " << sample.seq << std::endl;
        }
    }
    else
    {
        for (auto it = sample.meas.begin(); it != sample.meas.end(); ++it)
        {
            #if POSITIONER == TOA
            // Already ensured as sync frames will be sent before positioning frame.
            #elif POSITIONER == TDOA
            if(m_anchorClocks.find(it->first) == m_anchorClocks.end())
            {
                ClockModel clock;
                m_anchorClocks.insert(std::make_pair(it->first, clock));
            }
            #endif // POSITIONER
            it->second.toa = m_anchorClocks[it->first].getCorrectedTOA(it->second.ts);
            #if POSITIONER == TOA
            // Correct the timestamp from the sender tag with the clock model of the corresponding sender
            it->second.tos = m_anchorClocks[sample.txeui].getCorrectedTOA(it->second.ts1);
            it->second.tof = (it->second.toa - it->second.tos);
            it->second.distance = (it->second.tof * SPEED_OF_LIGHT);

            // Dynamic calibration of a reference offset for the first 20 samples
            // Until there, the Target shall be at the reference position without movement
            if((it->second.seq <= NUMOFFSETMEAS) && (m_anchorClocks[it->first].calib_st == 0))
            {
                m_anchorClocks[it->first].measure_ZeroRefOffset(it->second.distance);
            }
            else
            {
                if(it->second.seq == (NUMOFFSETMEAS + 1) && (m_anchorClocks[it->first].calib_st == 0))
                {
                    m_anchorClocks[it->first].calculate_ZeroRefOffset();
                    m_anchorClocks[it->first].calib_st = 1;
                    config_finished++;
                }
                //std::cout << "Raw - Distance of " << it->first << " to " << sample.txeui << " equals " << it->second.distance << " m"<< std::endl;
                it->second.distance = (it->second.distance - m_anchorClocks[it->first].ZeroRefOffset);
                std::cout << "Corr - Distance of " << it->first << " to " << sample.txeui << " equals " << it->second.distance << " m"<< std::endl;
            }

            #endif // POSITIONER
        }
        #if POSITIONER == TOA
        std::cout << "\r\n" << std::endl;
        #endif // POSITIONER

        /* Check time since last sync */
        int64_t d = std::chrono::duration_cast<std::chrono::milliseconds>(sample.hts - m_lastSync).count();
        #if POSITIONER == TOA
        if(config_finished == 0)
        {
            // Config not yet finished. Hence, do nothing.
        }
        else if(d < 200)
        #else
        if(d < 200)
        #endif // POSITIONER
        {
            m_samples.push_back(sample);
        }
        else
        {
            std::cout << "Warning: Last sync older than 200ms!!!" << std::endl;
        }
    }
}

void ClockCorrection::processSamples(std::vector<sample_t> *samples)
{
    for (auto it = samples->begin(); it != samples->end(); ++it)
    {
        processSample(*it);
    }
}

std::vector<sample_t> ClockCorrection::getCorrectedSamples()
{
    std::vector<sample_t> s = m_samples;
    m_samples.clear();
    return s;
}

#if POSITIONER == TOA
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


/*
    for(int i = 0; i < error_cntr; i++)
    {
        ZeroRefOffset = (ZeroRefOffset + errors[i]);
    }
    ZeroRefOffset = (ZeroRefOffset / error_cntr);
*/
    ZeroRefOffset = (ZeroRefOffset / devidecntr);
    std::cout << "ZeroRefOffset = " << ZeroRefOffset << std::endl;
}

void ClockModel::init(arma::vec pos)
{
    positionXYZ = pos;
}
#endif // POSITIONER
