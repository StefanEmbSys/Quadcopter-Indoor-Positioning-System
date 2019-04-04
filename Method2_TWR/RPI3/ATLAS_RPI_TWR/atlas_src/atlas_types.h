//
//  atlas_types.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__atlas_types__
#define __atlas__atlas_types__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include <armadillo>

typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> hts;
    uint64_t txeui;
    uint64_t seq;
    uint64_t rxeui;
    double distance;
} measurement_t;

typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> hts;
    uint64_t txeui;
    uint64_t seq;
    std::map<uint64_t, measurement_t> meas;
} sample_t;


typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> hts;
    uint64_t eui;
    arma::vec pos;
} position_t;


typedef struct
{
    std::map<uint64_t, arma::vec> positions;
    std::map<uint64_t, std::string> ports;
} configAnchors_t;


typedef struct
{
    std::map<uint64_t, std::string> ports;
} configTags_t;


typedef struct
{
    std::map<uint64_t, uint64_t> tagWhitelist;
} configParser_t;

typedef struct
{
    std::string port;
    int zmq_connection_type;
} configOutput_t;

#endif /* defined(__atlas__atlas_types__) */
