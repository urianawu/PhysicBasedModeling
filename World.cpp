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

void World::checkCollision(RigidBodyObject &b)
{
    for (int i = 0; i < Obstacles.size(); i++) {
        if(Obstacles[i]->CollisionDetection(b))
            collided = true;
    }

}
void World::addForce(Force *f)
{
    forceList.push_back(f);
}

void World::applyForce(Particle &p)
{

        //external forces
        float g = -1.0;
        Force gravity(Vector3D(0, p.getMass() * g, 0));
        ForceAir air(p.getVelocity());
        addForce(&gravity);
        addForce(&air);
        
        //edge forces
    

    for (int i = 0; i < forceList.size(); i++) {
        p.setForce( p.getForce() + forceList[i]->value);
    }
    

}
