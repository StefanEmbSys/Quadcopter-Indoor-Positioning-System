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
#define POINTA  0xdeca020000000001
#define POINTB  0xdeca020000000002
#define POINTC  0xdeca020000000003
#define POINTD  0xdeca020000000004

typedef struct
{
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;
    arma::vec state;
    arma::mat stateCovariance;
    arma::mat stateTransition;
    arma::mat processNoiseCovariance;
} ekf_t;

class PositionerTOA
{
private:
    // system parameters
    std::map<uint64_t, arma::vec> m_anchorPositions;
    arma::vec m_syncPosition;

    // local reference frame
    arma::vec m_ecefRef;
    double m_lrfAngle;

    // validation
    std::map<uint64_t, position_t> m_lastPosition;
    std::map<uint64_t, sample_t> m_lastSample;
    std::map<uint64_t, uint64_t> m_updateCount;

    // calibration
    std::map<uint64_t, double> m_calOffset;
    uint64_t m_calNode;
    arma::vec m_calPosition;
    double m_calIncrement;

    // ekf initialization and state
    std::map<uint64_t, ekf_t> m_ekf;
    double m_processNoise;
    double m_measurementNoise;
    double m_initialStateVariance;
    double m_initialStateVarianceDelta;
    double m_initialInterval;
    arma::vec m_initialState;

    // outlier detection
    double m_outlierThreshold;
    double m_outlierThresholdDelta;

public:
    PositionerTOA ();

    void initialize(const configAnchors_t &anchors, const configSync_t &sync, const configLRF_t &lrf, const configEKF_t &ekf, const configCal_t &cal);
    void createNewEKF(uint64_t eui, std::chrono::time_point<std::chrono::system_clock> ts);

    bool calculatePositionEKFInner (const sample_t &s, position_t *p);
    bool calculatePosition (const sample_t &s, position_t *p);
    arma::vec calcPosXYZ(double a, double b, double s1, double s2, double s3, int x_reverse, int y_reverse);
    bool vote(position_t *p, double pos1, double pos2, int coord);
};

#endif /* defined(__atlas__TOA__) */
