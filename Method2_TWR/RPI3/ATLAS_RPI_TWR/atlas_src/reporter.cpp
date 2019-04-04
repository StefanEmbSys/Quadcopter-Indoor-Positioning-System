//
//  reporter.cpp
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "reporter.h"

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>

#include <chrono>
#include <string>

#include "atlas_types.h"

Reporter::Reporter (std::string port, int zmq_connection_type)
    : m_port(port, zmq_connection_type)
{

}

Reporter::~Reporter()
{

}

void Reporter::reportPosition(uint64_t target_eui, position_t *p)
{
    uint8_t buffer[32];
    int i = 0;

    memcpy(&buffer[0], &target_eui, 8);
    memcpy(&buffer[8], &p->pos(0), 8);
    memcpy(&buffer[16], &p->pos(1), 8);
    memcpy(&buffer[24], &p->pos(2), 8);

    /*
    fprintf(stdout, "Buffer to be sent to Reporter: \r\n");
    for(i = 0; i < 32; i++)
    {
        fprintf(stdout, "%x ", buffer[i]);
    }
    fprintf(stdout, "\r\n");
    */

    m_port.send((const uint8_t*)buffer, 32);

    if (m_port.receive((void*)buffer, 1) != -1)
    {
        /*
        // print received message
        for(i = 0; i < 1; i++)
        {
            fprintf(stdout, "%x ", buffer[i]);
        }
        fprintf(stdout, "\r\n");
        */
    }

}
