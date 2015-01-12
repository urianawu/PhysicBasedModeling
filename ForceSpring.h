//
//  ForceSpring.h
//  Springy
//
//  Created by Uriana on 13-11-4.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#ifndef __Springy__ForceSpring__
#define __Springy__ForceSpring__

#include <iostream>
#include "Force.h"

class ForceSpring : public Force {
public:
    ForceSpring(Point3D pi, Point3D pj, Vector3D vi, Vector3D vj,float sc, float dc, float d);
    void setRestD(float d);
    
protected:
    float kij = 100;  //spring constant
    float dij = 1;  //damper constant
    float lij0 = 1;    //rest distance
};
#endif /* defined(__Springy__ForceSpring__) */
