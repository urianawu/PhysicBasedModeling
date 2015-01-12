//
//  World.cpp
//  Springy
//
//  Created by Uriana on 13-11-4.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "World.h"
void World::addObstacle(Obstacle* o)
{
    Obstacles.push_back(o);
}

void World::checkCollision(Particle p, Particle &p_after)
{
    for (int i = 0; i < Obstacles.size(); i++) {
        if(Obstacles[i]->CollisionDetection(p, p_after))
            collided = true;
    }
}