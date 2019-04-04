//
//  TBD
//

#ifndef __atlas__toa__
#define __atlas__toa__

#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>

#include <armadillo>

#include "atlas_types.h"

#define POSx    0
#define POSy    1
#define POSz    2
#define ANCHOR1  0xdeca020000000001
#define ANCHOR2  0xdeca020000000002
#define ANCHOR3  0xdeca020000000003
#define ANCHOR4  0xdeca020000000004


class Positioner
{
private:
    // system parameters
    std::map<uint64_t, arma::vec> m_anchorPositions;
    arma::vec m_syncPosition;

    // validation
    std::map<uint64_t, position_t> m_lastPosition;
    std::map<uint64_t, sample_t> m_lastSample;
    std::map<uint64_t, uint64_t> m_updateCount;

public:
    Positioner ();
    void initialize(const configAnchors_t &anchors);
    bool calculatePosition (sample_t *s, position_t *p);
    arma::vec calcPosXYZ(double a, double b, double s1, double s2, double s3, int x_reverse, int y_reverse);
    bool vote(position_t *p, double pos1, double pos2, int coord);
};

#endif /* defined(__atlas__TOA__) */
