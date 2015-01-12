//
//  definition.h
//  Springy
//
//  Created by Uriana on 13-11-5.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#ifndef __Springy__definition__
#define __Springy__definition__

#include <iostream>
#include "Vector3D.h"
struct RK4world
{
    double h = 0.1;              // timestep
    Point3D p[8][8][8];   // position of the 512 control points
    Vector3D v[8][8][8];   // velocities of the 512 control points
    bool   inBox[8][8][8];
    Point3D pCollission[8][8][8];
    double kCollision = 300;      // Hook's elasticity coefficient for collision springs
    double dCollision = 0.25;      // Damping coefficient collision springs
};

struct point
{
    double x;
    double y;
    double z;
};
extern struct RK4world jello;

// computes crossproduct of two vectors,
// struct point vector1, vector2, dest
// result goes into dest
#define CROSSPRODUCTp(vector1   ,  vector2   ,dest)                \
CROSSPRODUCT((vector1).x, (vector1).y, (vector1).z,\
             (vector2).x, (vector2).y, (vector2).z,\
             (dest).x   , (dest).y   , (dest).z    )

// computes crossproduct of two vectors
// double coordinates x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define  CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
x = (y1) * (z2) - (y2) * (z1);\
y = (x2) * (z1) - (x1) * (z2);\
z = (x1) * (y2) - (x2) * (y1)

// computes dotProduct of two vectors,
// struct point vector1, vector2     ,
// result goes into double result

#define DOTPRODUCT(vector1,vector2,result)\
\
result = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z);


// normalizes vector  dest, result returned in dest
// must declare a double variable called 'length' inside the scope of the NORMALIZE macro
// macro will change 'length'
#define pNORMALIZE(dest)\
\
length = sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z);\
(dest).x /= length;\
(dest).y /= length;\
(dest).z /= length;

// copies vector source to vector dest
// struct point source,dest
#define pCPY(source,dest)\
\
(dest).x = (source).x;\
(dest).y = (source).y;\
(dest).z = (source).z;

// assigns values x,y,z to point vector dest
// struct point dest
// double x,y,z
#define pMAKE(x0,y0,z0,dest)\
\
dest.x = x0;\
dest.y = y0;\
dest.z = z0;

// sums points src1 and src2 to dest
// struct point src1,src2,dest
#define pSUM(src1,src2,dest)\
\
(dest).x = (src1).x + (src2).x;\
(dest).y = (src1).y + (src2).y;\
(dest).z = (src1).z + (src2).z;

// dest = src1 - src2
// struct point src1,src2,dest
#define pDIFFERENCE(src1,src2,dest)\
\
(dest).x = (src1).x - (src2).x;\
(dest).y = (src1).y - (src2).y;\
(dest).z = (src1).z - (src2).z;

// mulitplies components of point src by scalar and returns the result in dest
// struct point src,dest
// double scalar
#define pMULTIPLY(src,scalar,dest)\
\
(dest).x = (src).x * (scalar);\
(dest).y = (src).y * (scalar);\
(dest).z = (src).z * (scalar);

#endif

