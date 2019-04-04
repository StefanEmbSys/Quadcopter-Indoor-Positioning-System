//
//  TOA.cpp
//  atlas
//
//  Created by Janis on 18.05.15.
//  Copyright (c) 2015 Janis. All rights reserved.
//

#include "toa.h"

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <armadillo>

#include "coordinate_systems.h"


PositionerTOA::PositionerTOA ()
{

}

// Init is TBD
void PositionerTOA::initialize(const configAnchors_t &anchors, const configSync_t &sync, const configLRF_t &lrf, const configEKF_t &ekf, const configCal_t &cal)
{
    std::cout << "Initializing PositionerTOA..." << std::endl;
    std::cout << "Config ekf: " << ekf.processNoise << "," << ekf.measurementVariance << ","
              << ekf.initialInterval << "," << ekf.initialVariance << "," << ekf.initialVarianceDelta << std::endl;

    m_anchorPositions = anchors.positions;
    m_syncPosition = sync.position;

    m_ecefRef = lla2ecef(lrf.wgs84ref);
    m_lrfAngle = calculateAngle(lrf.wgs84ref, lrf.wgs84axis);
    std::cout << "LRF Angle: " << m_lrfAngle << std::endl;

    m_processNoise = ekf.processNoise;
    m_measurementNoise = ekf.measurementVariance;
    m_initialInterval = ekf.initialInterval;
    m_initialStateVariance = ekf.initialVariance;
    m_initialStateVarianceDelta = ekf.initialVarianceDelta;
    m_initialState = ekf.initialState;

    m_outlierThreshold = 15;
    m_outlierThresholdDelta = 0.5;

    m_calNode = cal.eui;
    m_calIncrement = 1.0/cal.window;
    m_calPosition = cal.position;
}

// TBD
void PositionerTOA::createNewEKF(uint64_t eui, std::chrono::time_point<std::chrono::system_clock> ts)
{
    ekf_t ekf;
    ekf.lastUpdate = ts;

    ekf.state = m_initialState;
    //std::cout << "state:" << std::endl;
    //std::cout << state << std::endl;

    arma::mat stateTransition = arma::eye<arma::mat>(6, 6);
    stateTransition.diag(3).fill(m_initialInterval);
    ekf.stateTransition = stateTransition;
    //std::cout << "stateTransition:" << std::endl;
    //std::cout << stateTransition << std::endl;

    arma::vec dia;
    dia << m_initialStateVariance << m_initialStateVariance << m_initialStateVariance
        << m_initialStateVarianceDelta << m_initialStateVarianceDelta << m_initialStateVarianceDelta << arma::endr;
    arma::mat stateCovariance = arma::zeros<arma::mat>(6, 6);
    stateCovariance.diag() = dia;
    ekf.stateCovariance = stateCovariance;
    //std::cout << "stateCovariance:" << std::endl;
    //std::cout << ekf.stateCovariance << std::endl;

    m_ekf.insert(std::make_pair(eui, ekf));
}

// TBD
bool PositionerTOA::calculatePositionEKFInner(const sample_t &s, position_t *p)
{
    int count = s.meas.size();
    arma::mat anchorPositions = arma::zeros<arma::mat>(count, 3);
    arma::vec anchorTOAs = arma::zeros<arma::vec>(count);

    if (s.txeui == m_calNode)
    {
        // distance of first anchor to calibration
        double firstAnchorDistance;
        double firstToa;

        int row = 0;
        for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
        {
            arma::vec vds = m_anchorPositions[it->first] - m_syncPosition;
            double syncAnchorDistance = sqrt(arma::accu(arma::square(vds)));
            double toa = it->second.toa * 299792458 + syncAnchorDistance;

            arma::vec vdc = m_anchorPositions[it->first] - m_calPosition;
            double anchorDistance = sqrt(arma::accu(arma::square(vdc)));

            if(row == 0)
            {
                arma::vec vd = m_anchorPositions[it->first] - m_calPosition;
                firstAnchorDistance = sqrt(arma::accu(arma::square(vd)));
                firstToa = toa;
            }

            double expectedToa = anchorDistance - firstAnchorDistance;
            double relativeToa = toa - firstToa;
            double diffToa = relativeToa - expectedToa;

            if(m_calOffset.find(s.txeui) == m_calOffset.end())
            {
                m_calOffset.insert(std::make_pair(s.txeui, 0.0));
            }

            m_calOffset[it->first] = m_calOffset[it->first] - (m_calOffset[it->first] - diffToa) * m_calIncrement;

            std::cout << std::hex << it->first << " expected: " << expectedToa << ", toa: " << relativeToa << ", diff: " << diffToa << ", off: " << m_calOffset[it->first] << std::endl;
            row++;
        }
    }

    int row = 0;
    for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
    {
        anchorPositions.row(row) = m_anchorPositions[it->first].t();

        arma::vec vd = m_anchorPositions[it->first] - m_syncPosition;
        double d = sqrt(arma::accu(arma::square(vd)));

        if(m_calOffset.find(it->first) != m_calOffset.end())
        {
            d = d - m_calOffset[it->first];
        }

        anchorTOAs(row) = it->second.toa * 299792458 + d;
        row++;
    }

    // --- get ekf state for specific participant ---
    ekf_t ekf = m_ekf[s.txeui];
    arma::vec xk = ekf.state;
    arma::mat Pk = ekf.stateCovariance;

    // --- create dynamic state transition matrix ---
    double interval = m_initialInterval;
    if(m_lastPosition.find(p->eui) != m_lastPosition.end())
    {
        interval = std::chrono::duration<double>(s.hts - m_lastPosition[p->eui].hts).count();
        //std::cout << "interval: " << interval << std::endl;
    }
    arma::mat Fk = arma::eye<arma::mat>(6, 6);
    Fk.diag(3).fill(interval);

    // --- create dynamic process noise covariance matrix ---
    arma::mat Q;
    arma::mat pt = arma::eye<arma::mat>(3, 3) * pow(interval, 2) / 2;
    arma::mat pb = arma::eye<arma::mat>(3, 3) * interval;
    arma::mat pcv = arma::join_vert(pt, pb);
    Q = pcv * (arma::eye<arma::mat>(3, 3) * m_processNoise) * pcv.t();
    //ekf.processNoiseCovariance = Q;
    //std::cout << "Q:" << std::endl << Q << std::endl;

    // --- prediction ---
    // estimate state
    arma::mat exk = Fk * xk;
    //std::cout << "exk:" << std::endl << exk << std::endl;

    // estimate covariance
    arma::mat ePk = Fk * Pk * Fk.t() + Q;
    //std::cout << "ePk:" << std::endl << ePk << std::endl;

    // --- correction ---
    arma::vec referenceAnchor = anchorPositions.row(0).t();

    // observation vector
    arma::vec td = anchorTOAs - anchorTOAs(0);
    arma::vec dv = td.subvec(1, count - 1);
    //std::cout << "dv:" << std::endl << dv << std::endl;

    arma::vec ep = xk.rows(0, 2);
    //std::cout << "ep:" << std::endl << ep << std::endl;

    arma::mat epmat = arma::repmat(ep.t(), count - 1, 1);
    //std::cout << "epmat:" << std::endl << epmat << std::endl;

    arma::mat refmat = arma::repmat(referenceAnchor.t(), count - 1, 1);
    //std::cout << "refmat:" << std::endl << refmat << std::endl;

    arma::mat anmat = anchorPositions.rows(1, count - 1);
    //std::cout << "anmat:" << std::endl << anmat << std::endl;

    arma::mat ta1 = epmat - anmat;
    arma::mat ta2 = epmat - refmat;
    //std::cout << "ta1:" << std::endl << ta1 << std::endl << "ta2:" << std::endl << ta2 << std::endl;

    arma::mat distanceToAnchors = arma::sqrt(arma::sum(arma::square(ta1), 1));
    arma::mat distanceToReference = arma::sqrt(arma::sum(arma::square(ta2), 1));
    arma::mat ta1dr = arma::repmat(distanceToAnchors, 1, 3);
    arma::mat ta2dr = arma::repmat(distanceToReference, 1, 3);
    arma::mat t1 = ta1 / ta1dr;
    arma::mat t2 = ta2 / ta2dr;
    arma::mat j = t1 - t2;
    //std::cout << "ta1dr:" << std::endl << ta1dr << std::endl << "ta2dr:" << std::endl << ta2dr << std::endl;
    //std::cout << "t1:" << std::endl << t1 << std::endl << "t2:" << std::endl << t2 << std::endl << "j:" << std::endl << j << std::endl;

    arma::mat Hk = arma::join_horiz( j, arma::zeros<arma::mat>(count - 1, 3) );
    //std::cout << "Hk:" << std::endl << Hk << std::endl;

    // innovation
    arma::vec expectedMeasurement = distanceToAnchors - distanceToReference;
    arma::vec observedMeasurement = dv;
    arma::vec innovation = observedMeasurement - expectedMeasurement;
    //std::cout << "innovation:" << std::endl << innovation << std::endl;

    arma::mat Rk;
    Rk = arma::zeros<arma::mat>(count - 1, count - 1);
    Rk.diag().fill(m_measurementNoise);
    //std::cout << "Rk:" << std::endl << Rk << std::endl;

    // covariance of the innovation
    arma::mat Sk = Hk * ePk * Hk.t() + Rk;
    //std::cout << "Sk:" << std::endl << Sk << std::endl;

    // kalman gain computation
    arma::mat Kk = ePk * Hk.t() * Sk.i();
    //std::cout << "Kk:" << std::endl << Kk << std::endl;

    // --- state update ---
    // a posteriori state estimate
    xk = exk + Kk * innovation;
    //std::cout << "xk:" << std::endl << xk << std::endl;

    // a posteriori state covariance
    Pk = (arma::eye<arma::mat>(6, 6) - Kk * Hk) * ePk;
    //std::cout << "Pk:" << std::endl << Pk << std::endl;

    ekf.state = xk;
    ekf.stateCovariance = Pk;

    m_ekf[s.txeui] = ekf;

    p->pos = xk.rows(0, 2);
    p->dpos = xk.rows(3, 5);

    return true;
}

bool PositionerTOA::vote(position_t *p, double pos1, double pos2, int coord)
{
    bool locreturn = false;
    static double maxcoord = 2.5;

    // Validate the input
    if(!(arma::is_finite(pos1)))
    {
        if(!(arma::is_finite(pos2)))
        {
            // Out of Range, hence invalid
            locreturn = false;
        }
        else if((pos2 > (-maxcoord)) && (pos2 < maxcoord))
        {
            p->pos(coord) = pos2;
            locreturn = true;
        }
        else
        {
            // Out of Range, hence invalid
            locreturn = false;
        }
    }
    else if(!(arma::is_finite(pos2)))
    {
        if(!(arma::is_finite(pos1)))
        {
            // Out of Range, hence invalid
            locreturn = false;
        }
        else if((pos1 > (-maxcoord)) && (pos1 < maxcoord))
        {
            p->pos(coord) = pos1;
            locreturn = true;
        }
        else
        {
            // Out of Range, hence invalid
            locreturn = false;
        }
    }
    else
    {
        if(((pos2 > (-maxcoord)) && (pos2 < maxcoord)) && ((pos1 > (-maxcoord)) && (pos1 < maxcoord)))
        {
            // Both are valid, hence take the average value
            p->pos(coord) = ((pos1 + pos2) / 2);
            locreturn = true;
        }
        else if((pos2 > (-maxcoord)) && (pos2 < maxcoord))
        {
            p->pos(coord) = pos2;
            locreturn = true;
        }
        else if((pos1 > (-maxcoord)) && (pos1 < maxcoord))
        {
            p->pos(coord) = pos1;
            locreturn = true;
        }
        else
        {
            // Out of Range, hence invalid
            locreturn = false;
        }
    }
    return locreturn;
}

bool PositionerTOA::calculatePosition(const sample_t &s, position_t *p)
{
    arma::vec pos[2];
    double a, b;
    bool locreturn = false;
    // Calculation is base on the assumption, that the 0-Point from coordinate system must be at a/2, b/2 and h = 0
    a = (m_anchorPositions.at(POINTA)[POSx] * (-1) + m_anchorPositions.at(POINTB)[POSx]);
    b = (m_anchorPositions.at(POINTB)[POSy] * (-1) + m_anchorPositions.at(POINTC)[POSy]);
    //std::cout << "a = " << a << std::endl;
    //std::cout << "b = " << b << std::endl;
    pos[0] = calcPosXYZ(a, b, s.meas.at(POINTA).distance, s.meas.at(POINTB).distance, s.meas.at(POINTC).distance, 0, 0);
    std::cout << "posTarget ABC = [" << pos[0](0) << ", " << pos[0](1) << ", " << pos[0](2) << "]" << std::endl;

    a = (m_anchorPositions.at(POINTC)[POSx] + m_anchorPositions.at(POINTD)[POSx] * (-1));
    b = (m_anchorPositions.at(POINTD)[POSy] + m_anchorPositions.at(POINTA)[POSy] * (-1));
    //std::cout << "a = " << a << std::endl;
    //std::cout << "b = " << b << std::endl;
    pos[1] = calcPosXYZ(a, b, s.meas.at(POINTC).distance, s.meas.at(POINTD).distance, s.meas.at(POINTA).distance, 0, 1);
    std::cout << "posTarget CDA = [" << pos[1](0) << ", " << pos[1](1) << ", " << pos[1](2) << "]" << std::endl;


    // Vote all the results together
    // Vote for x
    locreturn = vote(p, pos[0](0), pos[1](0), 0);
    if(locreturn == false)  return locreturn;

    // Vote for y
    locreturn = vote(p, pos[0](1), pos[1](1), 1);
    if(locreturn == false)  return locreturn;

    // Vote for z
    //locreturn = vote(p, pos[0](2), pos[1](2), 2);
    //if(locreturn == false)  return locreturn;

    std::cout << "posTarget FINAL = [" << p->pos(0) << ", " << p->pos(1) << ", " << p->pos(2) << "]" << std::endl;



    /*
    // set basic position result props
    p->hts = s.hts;
    p->eui = s.txeui;
    p->num_s = s.meas.size();
    p->pos = posTarget;

    m_lastPosition[p->eui] = *p;
    */


/*
    if(m_ekf.find(s.txeui) == m_ekf.end())
    {
        createNewEKF(s.txeui, s.hts);
        m_updateCount.insert(std::make_pair(s.txeui, 0));
        m_lastSample.insert(std::make_pair(s.txeui, s));
        return false;
    }

    m_updateCount[s.txeui] = m_updateCount[s.txeui] + 1;
    if(m_updateCount[s.txeui] < 2)
    {
        return false;
    }

    // outlier detection
    double lastToa = m_lastSample[s.txeui].meas.begin()->second.toa * 299792458;
    double toa = s.meas.begin()->second.toa * 299792458;

    bool discard = false;
    for (auto it = s.meas.begin(); it != s.meas.end(); ++it)
    {
        if(m_lastSample[s.txeui].meas.find(it->first) != m_lastSample[s.txeui].meas.end())
        {
            double lastTOA = m_lastSample[s.txeui].meas[it->first].toa * 299792458 - lastToa;
            double TOA = it->second.toa * 299792458 - toa;
            double diff = fabs(lastTOA - TOA);

            if(diff > m_outlierThresholdDelta)
            {
                std::cout << "Warning - Outlier: TOA diff " << s.txeui << "," << it->first << " exceeded d: " << diff << std::endl;
                discard = true;
            }

            if(TOA > m_outlierThreshold || TOA < -m_outlierThreshold)
            {
                std::cout << "Warning - Outlier: TOA large " << s.txeui << "," << it->first << " exceeded threshold: " << TOA << std::endl;
                discard = true;
            }

        }
    }
    m_lastSample[s.txeui] = s;

    if(!discard)
    {
        if(!calculatePositionEKFInner(s, p))
        {
            return false;
        }
    }

    if(discard)
    {
        return false;
    }

    // set basic position result props
    p->hts = s.hts;
    p->eui = s.txeui;
    p->num_s = s.meas.size();

    // coordinate conversions
    p->enu = lrf2enu(p->pos, m_lrfAngle);
    p->ecef = enu2ecef(m_ecefRef, p->enu);
    p->wgs84 = ecef2lla(p->ecef);

    // calculate heading and velocities
    arma::vec denu = lrf2enu(p->dpos, m_lrfAngle);
    double vG = sqrt( pow(denu.at(0), 2) + pow(denu.at(1), 2) );
    double v = sqrt( pow(denu.at(0), 2) + pow(denu.at(1), 2) + pow(denu.at(2), 2) );
    double heading = 0;
    if (v != 0)
    {
        double h = atan2(denu.at(0), denu.at(1));
        if(denu.at(0) < 0)
        {
            h = h + 2 * M_PI;
        }
        heading = ((h * 180) / M_PI);
    }

    std::chrono::duration<double> dtc = p->hts - m_lastPosition[p->eui].hts;
    double dt = dtc.count();
    p->wgs84_heading = heading;
    p->wgs84_speed = vG * dt;

    m_lastPosition[p->eui] = *p;
*/

    return locreturn;
}

arma::vec PositionerTOA::calcPosXYZ(double a, double b, double s1, double s2, double s3, int x_reverse, int y_reverse)
{
    // Calculation is base on the assumption, that the 0-Point from coordinate system must be at a/2, b/2 and h = 0
    double x, y, z;
    arma::mat temp(1,2);
    arma::mat beta(1,2);
    arma::mat hba(1,2);
    arma::mat y2x1(1,2);
    arma::mat y1x1(1,2);
    arma::mat h(1,2);
    arma::mat t_square(2,3);
    arma::vec posTarget; // pos(0) = x, pos(1) = y, pos(2) = h

    // Square calculation (numerator) of arccos
    t_square(0,0) = s3;
    t_square(0,1) = b;
    t_square(0,2) = s2;
    t_square(1,0) = s2;
    t_square(1,1) = a;
    t_square(1,2) = s1;
    t_square = arma::square(t_square);
    //std::cout << "t_square(0,0) = " << t_square(0,0) << std::endl;
    //std::cout << "t_square(0,1) = " << t_square(0,1) << std::endl;
    //std::cout << "t_square(0,2) = " << t_square(0,2) << std::endl;
    //std::cout << "t_square(1,0) = " << t_square(1,0) << std::endl;
    //std::cout << "t_square(1,1) = " << t_square(1,1) << std::endl;
    //std::cout << "t_square(1,2) = " << t_square(1,2) << std::endl;

    // Calculation of numerator and denumerator and devision of numerator and denumerator of arccos
    temp(0,0) = ((t_square(0,0) + t_square(0,1) - t_square(0,2)) / (2 * s3 * b));
    temp(0,1) = ((t_square(1,0) + t_square(1,1) - t_square(1,2)) / (2 * s2 * a));
    //std::cout << "temp(0,0) = " << temp(0,0) << std::endl;
    //std::cout << "temp(0,1) = " << temp(0,1) << std::endl;

    // Calculation of angle beta (angle between b and CS / a and BS)
    beta = arma::acos(temp);
    //std::cout << "beta(0,0) = " << beta(0,0) << std::endl;
    //std::cout << "beta(0,1) = " << beta(0,1) << std::endl;

    // Calculation of hb and ha
    temp(0,0) = s3;
    temp(0,1) = s2;
    //std::cout << "temp(0,0) = " << temp(0,0) << std::endl;
    //std::cout << "temp(0,1) = " << temp(0,1) << std::endl;

    hba = arma::sin(beta);
    hba(0,0) = hba(0,0) * temp(0,0);
    hba(0,1) = hba(0,1) * temp(0,1);
    //std::cout << "hba(0,0) = " << hba(0,0) << std::endl;
    //std::cout << "hba(0,1) = " << hba(0,1) << std::endl;

    // calculation of y2 and x1
    y2x1 = arma::cos(beta);
    y2x1(0,0) = y2x1(0,0) * temp(0,0);
    y2x1(0,1) = y2x1(0,1) * temp(0,1);
    //std::cout << "y2x1(0,0) = " << y2x1(0,0) << std::endl;
    //std::cout << "y2x1(0,1) = " << y2x1(0,1) << std::endl;

    // Calculation of y1
    y1x1(0,0) = b - y2x1(0,0);
    y1x1(0,1) = y2x1(0,1);
    //std::cout << "y1x1(0,0) = " << y1x1(0,0) << std::endl;
    //std::cout << "y1x1(0,1) = " << y1x1(0,1) << std::endl;

    // calculation of all squares used in calculation of the hight of the target (h)
    t_square(0,0) = hba(0,0);
    t_square(0,1) = y1x1(0,1);
    t_square(1,0) = hba(0,1);
    t_square(1,1) = y1x1(0,0);
    //std::cout << "t_square(0,0) = " << t_square(0,0) << std::endl;
    //std::cout << "t_square(0,1) = " << t_square(0,1) << std::endl;
    //std::cout << "t_square(1,0) = " << t_square(1,0) << std::endl;
    //std::cout << "t_square(1,1) = " << t_square(1,1) << std::endl;

    t_square = arma::square(t_square);
    //std::cout << "t_square(0,0) = " << t_square(0,0) << std::endl;
    //std::cout << "t_square(0,1) = " << t_square(0,1) << std::endl;
    //std::cout << "t_square(1,0) = " << t_square(1,0) << std::endl;
    //std::cout << "t_square(1,1) = " << t_square(1,1) << std::endl;


    // Calculation of the square differences used in calculation of the hight of the target (h)
    temp(0,0) = t_square(0,0) - t_square(0,1);
    temp(0,1) = t_square(1,0) - t_square(1,1);
    //std::cout << "temp(0,0) = " << temp(0,0) << std::endl;
    //std::cout << "temp(0,1) = " << temp(0,1) << std::endl;

    // Calculation of the targets hight (h)
    h = arma::sqrt(temp);
    //std::cout << "h(0,0) = " << h(0,0) << std::endl;
    //std::cout << "h(0,1) = " << h(0,1) << std::endl;

    // Calculate z of the targets position by calculating the average value of the calculated hights
    z = (h(0,0) + h(0,1)) / 2;
    //std::cout << "z = " << z << std::endl;

    if(x_reverse != 0)
    {
        // Calculate x of the targets position
        x = (y1x1(0,1) - (a/2));
        //std::cout << "x = " << x << std::endl;
    }
    else
    {
        // Calculate x of the targets position
        x = ((a/2) - y1x1(0,1));
        //std::cout << "x = " << x << std::endl;
    }

    if (y_reverse != 0)
    {
        // Calculate y of the targets position
        y = (y1x1(0,0) - (b/2));
        //std::cout << "y = " << y << std::endl;
    }
    else
    {
        // Calculate y of the targets position
        y = (b/2) - (y1x1(0,0));
        //std::cout << "y = " << y << std::endl;
    }


    posTarget = {x, y, z};
    //std::cout << "posTarget = [" << x << ", " << y << ", " << z << "]" << std::endl;

    return posTarget;
}

