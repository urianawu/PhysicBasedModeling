//
//  Obstacle.h
//  Springy
//
//  Created by Uriana on 13-11-3.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#ifndef __Springy__Obstacle__
#define __Springy__Obstacle__

#include <iostream>
#include "Particle.h"
#include "RigidBodyObject.h"
class Obstacle {
public:
    virtual ~Obstacle(){}
    virtual bool CollisionDetection(Particle p, Particle &p_after) = 0;
    virtual void CollisionResponse(float d, Particle &p_after) = 0;
    
    virtual bool CollisionDetection(RigidBodyObject &b) = 0;

    bool collided;
};
#endif /* defined(__Springy__Obstacle__) */
