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
#define N 512

class RK4Integration {
public:
    void update(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w);
    void update2(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w);
    void update3(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w);
    void update4(std::vector<Particle *> &p, std::vector<Particle *> &p_after, World w);
    RK4Integration(float theh);
    float getTime() const;
    void setTime(float thetime);
    
protected:
    Point3D F1p[N], F2p[N], F3p[N], F4p[N];
    Vector3D F1v[N], F2v[N], F3v[N],F4v[N];
    
    std::vector<Particle *> buffer;
    float time = 0;
    float h;
};

#endif /* defined(__Springy__RK4Integration__) */
