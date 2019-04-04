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


Positioner::Positioner ()
{

}

// Init is TBD
void Positioner::initialize(const configAnchors_t &anchors)
{
    m_anchorPositions = anchors.positions;
}


bool Positioner::vote(position_t *p, double pos1, double pos2, int coord)
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

bool Positioner::calculatePosition(sample_t *s, position_t *p)
{
    arma::vec pos[2];
    double a, b;
    bool locreturn = false;

    // Check for validity of sample
    auto it = s->meas.find(ANCHOR1);
    if(it == s->meas.end())
    {
        return false;
    }

    // Calculation is base on the assumption, that the 0-Point from coordinate system must be at a/2, b/2 and h = 0
    a = (m_anchorPositions.at(ANCHOR1)[POSx] * (-1) + m_anchorPositions.at(ANCHOR2)[POSx]);
    b = (m_anchorPositions.at(ANCHOR2)[POSy] * (-1) + m_anchorPositions.at(ANCHOR3)[POSy]);
    //std::cout << "a = " << a << std::endl;
    //std::cout << "b = " << b << std::endl;
    pos[0] = calcPosXYZ(a, b, s->meas.at(ANCHOR1).distance, s->meas.at(ANCHOR2).distance, s->meas.at(ANCHOR3).distance, 0, 1);
    //std::cout << "posTarget ABC = [" << pos[0](0) << ", " << pos[0](1) << ", " << pos[0](2) << "]" << std::endl;

    a = (m_anchorPositions.at(ANCHOR3)[POSx] + m_anchorPositions.at(ANCHOR4)[POSx] * (-1));
    b = (m_anchorPositions.at(ANCHOR4)[POSy] + m_anchorPositions.at(ANCHOR1)[POSy] * (-1));
    //std::cout << "a = " << a << std::endl;
    //std::cout << "b = " << b << std::endl;
    pos[1] = calcPosXYZ(a, b, s->meas.at(ANCHOR3).distance, s->meas.at(ANCHOR4).distance, s->meas.at(ANCHOR1).distance, 1, 0);
    //std::cout << "posTarget CDA = [" << pos[1](0) << ", " << pos[1](1) << ", " << pos[1](2) << "]" << std::endl;


    // Vote all the results together
    // Vote for x
    locreturn = vote(p, pos[0](0), pos[1](0), 0);
    if(locreturn == false)  return locreturn;

    // Vote for y
    locreturn = vote(p, pos[0](1), pos[1](1), 1);
    if(locreturn == false)  return locreturn;

    // Vote for z
    locreturn = vote(p, pos[0](2), pos[1](2), 2);
    if(locreturn == false)  return locreturn;

    std::cout << "posTarget FINAL = [" << p->pos(0) << ", " << p->pos(1) << ", " << p->pos(2) << "]" << std::endl;
    std::cout << "\r\n" << std::endl;
    return locreturn;
}

arma::vec Positioner::calcPosXYZ(double a, double b, double s1, double s2, double s3, int x_reverse, int y_reverse)
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

