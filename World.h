//
//  World.h
//  Springy
//
//  Created by Uriana on 13-11-4.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#ifndef __Springy__World__
#define __Springy__World__

#include <iostream>
#include <vector>
#include "Obstacle.h"
#include "ObstaclePlane.h"
#include "Force.h"
#include "ForceAir.h"

class World {
public:
    void addObstacle(Obstacle* o);
    void checkCollision(Particle p, Particle &p_after);
    void checkCollision(RigidBodyObject &b);
    void addForce(Force *f);
    void applyForce(Particle &p);

protected:
    std::vector<Obstacle *> Obstacles;
    std::vector<Force *> forceList;

    bool collided = false;
};
#endif /* defined(__Springy__World__) */
