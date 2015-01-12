//
//  RK4Integration.cpp
//  Springy
//
//  Created by Uriana on 13-11-5.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "RK4Integration.h"
#define SIZE 8
#define N 512

RK4Integration::RK4Integration(float theh)
{
    h = theh;
}

float RK4Integration::getTime() const
{
    return time;
}
void RK4Integration::setTime(float thetime)
{
    time = thetime;
}
void RK4Integration::update(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w) {
    
    
    // make a copy of current particle
    for (int i = 0; i < N; i++) {
        buffer.push_back(new Particle());
        size_t n = buffer.size();
        buffer[n-1]->setVelocity(p[i]->getVelocity());
        buffer[n-1]->setPosition(p[i]->getPosition());
        buffer[n-1]->setMass(p[i]->getMass());
        buffer[n-1]->setAccel(p[i]->getAccel());
        buffer[n-1]->setColor(p[i]->getColor());
        buffer[n-1]->setForce(p[i]->getForce());

    }
    //printf("%f\t",buffer[0]->getPosition().x);
    for (int n = 0; n < N; n++) {
        p[n]->setAccel(p[n]->calculateAccel());
    }
    
    for (int n = 0; n < N; n++) {
        F1p[n] = p[n]->getVelocity() * h;
        F1v[n] = p[n]->getAccel() * h;
        buffer[n]->setPosition(F1p[n] * 0.5);
        buffer[n]->setVelocity(F1v[n] * 0.5);
        
        buffer[n]->setPosition(buffer[n]->getPosition() + p[n]->getPosition());
        buffer[n]->setVelocity(buffer[n]->getVelocity() + p[n]->getVelocity());
    }
    
}
void RK4Integration::update2(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w) {
    for (int n = 0; n < N; n++) {
        buffer[n]->setAccel(buffer[n]->calculateAccel());
    }
    for (int n = 0; n < N; n++) {
        
        
        F2p[n] = buffer[n]->getVelocity() * h;
        F2v[n] = p[n]->getAccel() * h;
        buffer[n]->setPosition(F2p[n] * 0.5);
        buffer[n]->setVelocity(F2v[n] * 0.5);
        buffer[n]->setPosition(buffer[n]->getPosition() + p[n]->getPosition());
        buffer[n]->setVelocity(buffer[n]->getVelocity() + p[n]->getVelocity());
    }
}
void RK4Integration::update3(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w) {
    for (int n = 0; n < N; n++) {
        buffer[n]->setAccel(buffer[n]->calculateAccel());
    }
    for (int n = 0; n < N; n++) {
        
        F3p[n] = buffer[n]->getVelocity() * h;
        F3v[n] = p[n]->getAccel() * h;
        buffer[n]->setPosition(F3p[n] * 0.5);
        buffer[n]->setVelocity(F3v[n] * 0.5);
        buffer[n]->setPosition(buffer[n]->getPosition() + p[n]->getPosition());
        buffer[n]->setVelocity(buffer[n]->getVelocity() + p[n]->getVelocity());
    }
}
void RK4Integration::update4(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w) {
    for (int n = 0; n < N; n++) {
        buffer[n]->setAccel(buffer[n]->calculateAccel());
    }
    for (int n = 0; n < N; n++) {
        F4p[n] = buffer[n]->getVelocity() * h;
        F4v[n] = p[n]->getAccel() * h;
        buffer[n]->setPosition(F2p[n] * 2);
        buffer[n]->setVelocity(F3p[n] * 2);
        Point3D bp(buffer[n]->getPosition().x,buffer[n]->getPosition().y,buffer[n]->getPosition().z);
        Point3D bv(buffer[n]->getVelocity().x,buffer[n]->getVelocity().y,buffer[n]->getVelocity().z);
        
        buffer[n]->setPosition((bp + bv+F1p[n]+F4p[n])/6.0);
        p_after[n]->setPosition(buffer[n]->getPosition() + p[n]->getPosition());
        
        buffer[n]->setPosition(Point3D(F2v[n].x * 2, F2v[n].y * 2, F2v[n].z * 2));
        buffer[n]->setVelocity(F3v[n] * 2);
        buffer[n]->setPosition((buffer[n]->getPosition() + buffer[n]->getVelocity()+F1v[n]+F4v[n])/6.0);
        p_after[n]->setVelocity(buffer[n]->getPosition() + p[n]->getVelocity());
        
        
    }
    
    for (int n = 0; n < N; n++) {
        //w.checkCollision(*p[n], *p_after[n]);
    }
    
    p = p_after;
    setTime(time + h);

}