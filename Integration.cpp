//
//  Integration.cpp
//  Springy
//
//  Created by Uriana on 13-11-1.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "Integration.h"
Integration::Integration()
{
    h = 0.1;
}
Integration::Integration(float theh)
{
    h = theh;
}
float Integration::getTime() const
{
    return time;
}
void Integration::setTime(float thetime)
{
    time = thetime;
}