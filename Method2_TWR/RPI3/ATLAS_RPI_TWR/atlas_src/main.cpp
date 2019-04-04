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

#include "toa.h"

#include "logger.h"
#include "reporter.h"


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


void signal_handler(int signal)
{
    std::cerr << " Quitting..." << std::endl;
    exit (EXIT_SUCCESS);
}

int main(int argc, const char* argv[])
{
    std::signal(SIGINT, signal_handler);

    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true,
    "Host-SW for IndoorPOS\r\nA Master Thesis from Stefan Koller BsC");


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
    Reporter rep(config.m_output.port, config.m_output.zmq_connection_type);

    Parser parser;
    parser.initialize(config.m_parser, config.m_tagsconf);

    Positioner positioner;
    positioner.initialize(config.m_anchors);

    // initialize position
    position_t position;
    position.pos = {0, 0, 0};

    std::cout << "Init complete" << std::endl;

    //auto ts = std::chrono::high_resolution_clock::now();

    std::cout << "Polling started" << std::endl;

    while (true)
    {
        parser.poll();

        sample_t sample;
        parser.extractSample(&sample);

        if(positioner.calculatePosition(&sample, &position))
        {
            rep.reportPosition(0xdeca010000000001, &position);
        }
        usleep(1000);
    }
    return 0;
}

