//
//  ForceSpring.cpp
//  Springy
//
//  Created by Uriana on 13-11-4.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "ForceSpring.h"
ForceSpring::ForceSpring(Point3D pi, Point3D pj, Vector3D vi, Vector3D vj, float sc, float dc, float d)
{
    kij = sc;  //spring constant
    dij = dc;  //damper constant
    lij0 = d;
    
    //Spring Force
    Vector3D sf;
    Vector3D xij(pj - pi);
    float lij = Magnitude(xij);
    Vector3D uij = xij;
    uij.Normalize();
    
    sf = kij * (lij - lij0) *uij;
    
    //Damper Force
    Vector3D df;
    df = dij * ((vj - vi) * uij) *uij;
    
    value = sf + df;
}

void ForceSpring::setRestD(float d)
{
    lij0 = d;
}
