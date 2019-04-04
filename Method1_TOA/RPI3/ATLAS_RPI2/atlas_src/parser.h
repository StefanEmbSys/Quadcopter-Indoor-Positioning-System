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

class Parser
{
private:
    const uint64_t ticksPerRevolution = 0x10000000000;
    std::map<uint64_t, protocol::Protocol*> m_decoders;
    std::map<uint64_t, protocol::Protocol*> m_tags;
    std::map<uint64_t, std::map<uint64_t, std::map<uint64_t, measurement_t> > > m_measurements;
    std::map<uint64_t, uint64_t> m_lastSequence;
    std::map<uint64_t, uint64_t> m_lastTimestamp;
    std::map<uint64_t, uint64_t> m_tagWhitelist;

public:
    Parser ();
    ~Parser();

    #if POSITIONER == TOA
    void initialize(const configParser_t &p, const configAnchors_t &a, configTags_t &t, const configSync_t &s);
    #elif POSITIONER == TDOA
    void initialize(const configParser_t &p, const configAnchors_t &a, const configSync_t &s);
    #endif // POSITIONER
    void poll();
    void extractSamples(std::vector<sample_t> *s);
};

class ClockModel
{
private:
    const uint64_t ticksPerSecond = 128 * 499.2e6;
    const uint64_t ticksPerInterval = 128 * 499.2e5;
    double m_lastSync;
    double m_lastOffset;
    double m_lastDrift;
    #if POSITIONER == TOA
    double errors[NUMOFFSETMEAS] = {0};
    int error_cntr = 0;
    arma::vec positionXYZ;
    #endif // POSITIONER

public:
    ClockModel();
    ~ClockModel();
    #if CLOCKCORR == ATLASEXT
    void processSynchronizationFrame(arma::vec pos_tx, arma::vec pos_rx, uint64_t txts, uint64_t rxts);
    #elif CLOCKCORR == ATLAS
    void processSynchronizationFrame(uint64_t seq, uint64_t ts);
    #endif // CLOCKCORR
    double getCorrectedTOA(uint64_t ts);
    #if POSITIONER == TOA
    void init(arma::vec pos);
    void measure_ZeroRefOffset(double measured_distance);
    void calculate_ZeroRefOffset(void);
    double ZeroRefOffset = 0;
    int calib_st = 0;
    #endif // POSITIONER
};

class ClockCorrection
{
private:
    const uint64_t syncEUI = 0xdeca030000000001;
    std::map<uint64_t, ClockModel> m_anchorClocks;
    std::vector<sample_t> m_samples;
    std::chrono::time_point<std::chrono::system_clock> m_lastSync;
    #if ((POSITIONER == TOA) || (CLOCKCORR == ATLASEXT))
    std::map<uint64_t, arma::vec> m_RxPositions;
    #endif
    #if CLOCKCORR == ATLASEXT
    arma::vec m_TxPosition;
    #endif // CLOCKCORR
    uint64_t m_lastSequence;

public:
    ClockCorrection();
    ~ClockCorrection();

    void processSample(sample_t sample);
    void processSamples(std::vector<sample_t> *samples);
    std::vector<sample_t> getCorrectedSamples();
    #if ((POSITIONER == TOA) || (CLOCKCORR == ATLASEXT))
    void initialize(const configAnchors_t &a, configTags_t &t, const configSync_t &s);
    #endif // POSITIONER
    #if CLOCKCORR == ATLASEXT
    void updateTagPosition(uint64_t tageui, arma::vec tagPosition);
    #endif // CLOCKCOR
};


#endif /* defined(__atlas__parser_tdoa__) */
