//
//  ForceAir.h
//  Springy
//
//  Created by Uriana on 13-11-4.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#ifndef __Springy__ForceAir__
#define __Springy__ForceAir__

#include <iostream>
#include "Force.h"
class ForceAir : public Force {
public:
    ForceAir(Vector3D v);
    
    float ad = 0.1;
};
#endif /* defined(__Springy__ForceAir__) */
