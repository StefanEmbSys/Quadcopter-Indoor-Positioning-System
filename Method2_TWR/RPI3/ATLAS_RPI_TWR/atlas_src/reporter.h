//
//  reporter.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__reporter__
#define __atlas__reporter__

#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <chrono>

#include "atlas_types.h"
#include "../serial/serial.h"
#include "../ethernet/ethernet.h"


class Reporter
{
private:

public:
    Ethernet m_port;
    Reporter (std::string port, int zmq_connection_type);
    ~Reporter();

    void reportPosition(uint64_t target_eui, position_t *p);
};


#endif /* defined(__atlas__reporter__) */
