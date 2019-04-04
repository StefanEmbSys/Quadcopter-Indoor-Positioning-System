//
//  protocol_structures.h
//  atlas
//
//  Created by Janis on 26.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#ifndef __atlas__protocol_structures__
#define __atlas__protocol_structures__

namespace protocol
{

#pragma pack(1)

typedef struct
{
    uint64_t    txId;
    uint32_t    seqNr;
    uint64_t    rxId[4];
    double      distance[4];
} msgDistance_t;

}

#endif /* defined(__atlas__protocol_structures__) */
