//
//  main.cpp
//  atlas
//
//  Created by Janis on 26.03.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <csignal>
#include <exception>
#include <map>

#include <armadillo>
#include "../docopt/docopt.h"

#include "atlas_types.h"
#include "config.h"
#include "parser.h"

#if POSITIONER == TOA
#include "toa.h"
#elif POSITIONER == TDOA
#include "tdoa.h"
#endif // POSITIONER

#include "logger.h"
#include "reporter.h"


#if CONNECTION == ETH_ZMQ
static const char USAGE[] =
    R"(atlas.

    Usage:
        atlas [-c CFILE] [--quiet | --verbose]
        atlas list [-c CFILE]
        atlas (-h | --help)
        atlas --version

        Options:
        -c CFILE            Specify config file [default: config.yaml]
        -v --verbose        Verbose output
        -q --quiet          Do not output
        -h --help           Show this usage screen.
        --version           Show version.

    Additioanl Info:
        To execute over comand line:
            cd to folder where the executable is and then "./atlas [comand]"

        [comand] can be one or more of the options sepcified above.

        The config file can be specified in two ways:
              1) ./atlas -c <absolute path>
                 <absolute path> can be e.g. /home/pi/Documents/.../config.yaml
              2) ./atlas -c <relative path>
                 <relative path> starts at the folder where ./atlas executed
                 e.g. ../../config.yaml
)";
#elif CONNECTION == SER_USB
static const char USAGE[] =
    R"(atlas.

    Usage:
        atlas [-c CFILE] [--quiet | --verbose]
        atlas list [-c CFILE]
        atlas (-h | --help)
        atlas --version

        Options:
        -c CFILE            Specify config file [default: config.yaml]
        -v --verbose        Verbose output
        -q --quiet          Do not output
        -h --help           Show this usage screen.
        --version           Show version.
)";
#endif // CONNECTION


void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    exit (EXIT_SUCCESS);
}

int main(int argc, const char* argv[])
{
    std::signal(SIGINT, signal_handler);

    #if CONNECTION == ETH_ZMQ
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true,
    "atlas 3.0\r\nExtended atlas 2.0 with TOA positioner instead of TDOA positioner.");
    #elif CONNECTION == SER_USB
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "atlas 1.0");
    #endif // CONNECTION


    std::string config_file = args.find("-c")->second.asString();
    Config config(config_file);

    if(args.find("list")->second.asBool())
    {
        std::cout << "Listing config file: " << config_file << std::endl;
        config.list();
    }

    // against ATLAS_1.0, the init of logger and reporter was shifted here as it is necessary to
    // open the publisher port for the reporter before anyone connects to it.
    Logger log;
    #if CONNECTION == ETH_ZMQ
    Reporter rep(config.m_output.port, config.m_output.zmq_connection_type);
    #elif CONNECTION == SER_USB
    Reporter rep(config.m_output.port);
    #endif // CONNECTION

    Parser parser;
    #if POSITIONER == TOA
    parser.initialize(config.m_parser, config.m_anchors, config.m_tagsconf, config.m_sync);
    #elif POSITIONER == TDOA
    parser.initialize(config.m_parser, config.m_anchors, config.m_sync);
    #endif // POSITIONER
    ClockCorrection cor;
    #if ((POSITIONER == TOA) || (CLOCKCORR == ATLASEXT))
    config.m_tagsconf.positions.insert(std::pair<uint64_t,arma::vec> (0xdeca010000000001, {0, 0, 0}));
    cor.initialize(config.m_anchors, config.m_tagsconf, config.m_sync);
    #endif // CLOCKCORR


    #if POSITIONER == TOA
    PositionerTOA pos;
    pos.initialize(config.m_anchors, config.m_sync, config.m_lrf, config.m_ekf, config.m_cal);
    #elif POSITIONER == TDOA
    PositionerTDOA pos;
    pos.initialize(config.m_anchors, config.m_sync, config.m_lrf, config.m_ekf, config.m_cal);
    #endif // POSITIONER

    std::cout << "Init complete" << std::endl;

    auto ts = std::chrono::high_resolution_clock::now();

    // Send on the publisher that init is finished and normal operation shall be started
    rep.m_port.send("GO", 2);
    std::cout << "Polling started" << std::endl;
    //usleep(100000);

    while (true)
    {
        parser.poll();

        std::vector<sample_t> s;
        parser.extractSamples(&s);

        cor.processSamples(&s);

        std::vector<sample_t> cs;
        cs = cor.getCorrectedSamples();

        for (auto it = cs.begin(); it != cs.end(); ++it)
        {
            sample_t s = *it;

            position_t p;
            #if POSITIONER == TOA
            p.pos = {0, 0, 0};
            if(pos.calculatePosition(s, &p))
            #elif POSITIONER == TDOA
            if(pos.calculatePositionEKF(s, &p))
            #endif // POSITIONER
            {
                // Do nothing.
                //cor.updateTagPosition(p.eui, p.pos);
                //log.logSample(s);
                //log.logPosition(p);
                //rep.reportPosition(p);
            }

        }
    }
    return 0;
}

