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

class World {
public:
    void addObstacle(Obstacle* o);
    void checkCollision(Particle p, Particle &p_after);
    
protected:
    std::vector<Obstacle *> Obstacles;
    bool collided = false;
};
#endif /* defined(__Springy__World__) */
