//
//  atlas_types.h
//  atlas
//
//  Created by Janis on 22.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__atlas_comm_conf__
#define __atlas__atlas_comm_conf__

#define SER_USB     0
#define ETH_ZMQ     1
#define CONNECTION  ETH_ZMQ

#define TOA         0
#define TDOA        1
#define POSITIONER  TOA

#define ATLASEXT    0
#define ATLAS       1
#define CLOCKCORR   ATLAS

// Speed of light in air in m/s
#define SPEED_OF_LIGHT 299702547

#endif /* defined(__atlas__atlas_comm_conf__) */
