//
//  RK4Integration.h
//  Springy
//
//  Created by Uriana on 13-11-5.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#ifndef __Springy__RK4Integration__
#define __Springy__RK4Integration__

#include <iostream>
#include "Vector3D.h"
#include "Integration.h"
# include "rk4.hpp"


class RK4Integration :public Integration{
public:
    void update(Particle &p, Particle &p_after, World w);
    void update(RigidBodyObject &b, RigidBodyObject &b_after, World w);
    
    RK4Integration(float theh);
    
protected:
    Point3D F1p, F2p, F3p, F4p;
    Vector3D F1v, F2v, F3v,F4v;
    
};

#endif /* defined(__Springy__RK4Integration__) */
