//
//  ForceAir.cpp
//  Springy
//
//  Created by Uriana on 13-11-4.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "ForceAir.h"

ForceAir::ForceAir(Vector3D v)
{
    value = - ad * v;
}