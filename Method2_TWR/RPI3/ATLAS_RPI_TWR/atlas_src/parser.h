//
//  parser.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__parser_tdoa__
#define __atlas__parser_tdoa__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <armadillo>
#include <stdexcept>

#include "../protocol/protocol.h"
#include "atlas_types.h"

#define MAXSYSTIME_DW1000   17.20735697 // 0xFFFFFFE000 * (Conversion to seconds)
#define MAX_OFFSETERROR      2
#define NUMOFFSETMEAS       10

#define POS_TARGETEUI       0
#define POS_SEQNUM          8
#define POS_ANCHOREUIS     12
#define POS_DISTANCES      (POS_ANCHOREUIS + (8 * NUM_ANCHORS))

class Parser
{
private:
    std::map<uint64_t, protocol::Protocol*> m_tags;
    std::map<uint64_t, std::map<uint64_t, std::map<uint64_t, measurement_t> > > m_measurements;
    std::map<uint64_t, uint64_t> m_lastSequence;
    std::map<uint64_t, uint64_t> m_lastTimestamp;
    std::map<uint64_t, uint64_t> m_tagWhitelist;

public:
    Parser ();
    ~Parser();
    void poll();
    void initialize(const configParser_t &p, configTags_t &t);
    void extractSample(sample_t *sample);
    void fillUintFromBuffer(uint64_t *var, uint8_t *buff, uint8_t length);
    //void measure_ZeroRefOffset(double measured_distance);
    //void init(arma::vec pos)
    //void calculate_ZeroRefOffset(void);
};

#endif /* defined(__atlas__parser_tdoa__) */
