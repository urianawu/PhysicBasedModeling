//
//  RK4Integration.cpp
//  Springy
//
//  Created by Uriana on 13-11-5.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "RK4Integration.h"

#define STATE_SIZE 18



RK4Integration::RK4Integration(float theh)
{
    h = theh;
}


void RK4Integration::update(Particle &p, Particle &p_after, World w) {
    
    Particle buffer;

    // make a copy of current particle
    buffer.setVelocity(p.getVelocity());
    buffer.setPosition(p.getPosition());
    buffer.setMass(p.getMass());
    buffer.setAccel(p.getAccel());
    buffer.setColor(p.getColor());
    buffer.setForce(p.getForce());
    
    //printf("%f\t",buffer.getPosition().x);
    w.applyForce(p);
    p.setAccel(p.calculateAccel());
    
    F1p = p.getVelocity() * h;
    F1v = p.getAccel() * h;
    buffer.setPosition(F1p * 0.5);
    buffer.setVelocity(F1v * 0.5);
    
    buffer.setPosition(buffer.getPosition() + p.getPosition());
    buffer.setVelocity(buffer.getVelocity() + p.getVelocity());
    
    w.applyForce(buffer);
    buffer.setAccel(buffer.calculateAccel());
    
    F2p = buffer.getVelocity() * h;
    F2v = p.getAccel() * h;
    buffer.setPosition(F2p * 0.5);
    buffer.setVelocity(F2v * 0.5);
    buffer.setPosition(buffer.getPosition() + p.getPosition());
    buffer.setVelocity(buffer.getVelocity() + p.getVelocity());
    
    w.applyForce(buffer);
    buffer.setAccel(buffer.calculateAccel());
    
    F3p = buffer.getVelocity() * h;
    F3v = p.getAccel() * h;
    buffer.setPosition(F3p * 0.5);
    buffer.setVelocity(F3v * 0.5);
    buffer.setPosition(buffer.getPosition() + p.getPosition());
    buffer.setVelocity(buffer.getVelocity() + p.getVelocity());
    
    w.applyForce(buffer);
    buffer.setAccel(buffer.calculateAccel());
    F4p = buffer.getVelocity() * h;
    F4v = p.getAccel() * h;
    buffer.setPosition(F2p * 2);
    buffer.setVelocity(F3p * 2);
    Point3D bp(buffer.getPosition().x,buffer.getPosition().y,buffer.getPosition().z);
    Point3D bv(buffer.getVelocity().x,buffer.getVelocity().y,buffer.getVelocity().z);
    
    buffer.setPosition((bp + bv + F1p + F4p)/6.0);
    p_after.setPosition(buffer.getPosition() + p.getPosition());
    
    buffer.setPosition(Point3D(F2v.x * 2, F2v.y * 2, F2v.z * 2));
    buffer.setVelocity(F3v * 2);
    buffer.setPosition((buffer.getPosition() + buffer.getVelocity() + F1v + F4v)/6.0);
    p_after.setVelocity(buffer.getPosition() + p.getVelocity());
    p_after.setMass(buffer.getMass());
    p_after.setColor(buffer.getColor());
    
    w.checkCollision(p, p_after);
    
    p = p_after;
    setTime(time + h);
    
}

void RK4Integration::update(RigidBodyObject &b, RigidBodyObject &b_after, World w)
{
//    double * (RigidBodyObject::*DerivFunc)(double,int,double[]) = NULL;
//    DerivFunc = &RigidBodyObject::Dxdt;
//    
//    int i;
//    double *x0 = nullptr;
//    double *xFinal = nullptr;
//    
//    b.StateToArray(x0);
//    b.StateToArray(xFinal);
//    
//    xFinal = rk4vec( time, STATE_SIZE, x0, h, (b.*DerivFunc) );
//    //
//    //  Shift the data to prepare for another step.
//    //
//    setTime(time + h);
//    for ( i = 0; i < STATE_SIZE; i++ )
//    {
//        x0[i] = xFinal[i];
//    }
//    delete [] xFinal;
//    
//    b_after.ArrayToState(xFinal);
    
}