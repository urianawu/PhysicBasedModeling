//
//  EulerIntegration.cpp
//  Springy
//
//  Created by Uriana on 13-11-1.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "EulerIntegration.h"
EulerIntegration::EulerIntegration(float theh)
{
    h = theh;
    
}

void EulerIntegration::update(Particle &p, Particle &p_after, World w)
{
    w.applyForce(p);
    p.setAccel(p.calculateAccel());
    
    //printf("%f", p.getAccel().y);
    //integrate to get new state
    p_after.setVelocity(p.getVelocity() + p.getAccel() * h);
    p_after.setPosition(p.getPosition() + p.getVelocity() * h);
    p_after.setColor(p.getColor());
    p_after.setMass(p.getMass());
    
    //check for collision

    w.checkCollision(p, p_after);
    
    //update state
    p = p_after;
    setTime(time + h);
    
}
void EulerIntegration::update(RigidBodyObject &b, RigidBodyObject &b_after, World w)
{
    
}