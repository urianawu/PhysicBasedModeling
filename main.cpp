//
//  main.cpp
//  Springy
//
//  Created by Uriana on 13-11-1.
//  Copyright (c) 2013 Uriana. All rights reserved.
//

#include <iostream>
#include <time.h>
#include <vector>
#include <GLUI/glui.h>

#include "Vector3D.h"
#include "EulerIntegration.h"
#include "RK4Integration.h"
#include "ForceAir.h"
#include "ForceSpring.h"
#include "definition.h"

#define WIDTH 800
#define HEIGHT 600

#define N 512
#define SIZE 8
#define pi 3.1415926

GLfloat posx = -1.0;    //camera position
GLfloat posy = 1.0;    //camera position
double frames_per_second = 30;   // display rate
double size = 2;        //the size of the box


EulerIntegration ei(0.01);
//RK4Integration rk(0.001);
struct RK4world jello;

bool isEuler = true;
float factor = 10;
static int collided=0;
float RK4time;

std::vector<Particle *> s(N);
std::vector<Particle *> s_next(N);
Point3D pos[SIZE][SIZE][SIZE];
Vector3D vel[SIZE][SIZE][SIZE];
World w;


// camera parameters tweaked for a good first look
double Theta = 0.3136 ;
double Phi   = 0.7136 ;
double R     = 4.6    ;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton  ,
g_iMiddleMouseButton,
g_iRightMouseButton ;
// these variables control what is displayed on screen
int shear            = 0,
bend             = 0,
structural       = 1,
doPause          = 0,
viewingMode      = 1,
saveScreenToFile = 0;


struct index
{
    int i;
    int j;
    int k;
};


using namespace std;

void initialize()
{
    float mass = 0.5;

    
    //for (int n = 0; n < N; n++) {
        for (int i = 0; i < SIZE; i++) {
            for (int j = 0; j < SIZE; j++) {
                for (int k = 0; k < SIZE; k++) {
                    int n = i*SIZE*SIZE + j*SIZE + k;
                    Particle *p = new Particle(Point3D((float)i/SIZE,(float)j/SIZE+1,(float)k/SIZE), Vector3D(0,1,0), mass, Point3D(0.5,0.5,0.5));
                    //printf("%f \t",p->getPosition().x);
                    s[n] = p;
                    pos[i][j][k] = p->getPosition();
                    vel[i][j][k] = p->getVelocity();
                    jello.p[i][j][k] = pos[i][j][k];
                    jello.v[i][j][k] = vel[i][j][k];

                    Particle *p_init = new Particle();
                    s_next[n] = p_init;
                }
            }
        }
    
        
    
    w.addObstacle(new ObstaclePlane(Vector3D(0, 1, 0), Point3D(0, -2, 0)));
    w.addObstacle(new ObstaclePlane(Vector3D(0, -1, 0), Point3D(0, 2, 0)));
    w.addObstacle(new ObstaclePlane(Vector3D(0, 0, 1), Point3D(0, 0, -2)));
    w.addObstacle(new ObstaclePlane(Vector3D(0, 0, -1), Point3D(0, 0, 2)));
    w.addObstacle(new ObstaclePlane(Vector3D(1, 0, 0), Point3D(-2, 0, 0)));
    w.addObstacle(new ObstaclePlane(Vector3D(-1, 0, 0), Point3D(2, 0, 0)));

}

void computeStructForces(int pti, int ptj, int ptk, Vector3D &Fstruct)
{
    //get the neighbour point indexes in neighbours list vector.
    
    vector<struct index> neighbours;
    struct index curIndex;
    
    if(pti!=0)
    {
        curIndex.i = pti-1;
        curIndex.j = ptj;
        curIndex.k = ptk;
        neighbours.push_back(curIndex);
    }
    if(pti!=7)
    {
        curIndex.i = pti+1;
        curIndex.j = ptj;
        curIndex.k = ptk;
        neighbours.push_back(curIndex);
    }
    if(ptj!=0)
    {
        curIndex.i = pti;
        curIndex.j = ptj-1;
        curIndex.k = ptk;
        neighbours.push_back(curIndex);
    }
    if(ptj!=7)
    {
        curIndex.i = pti;
        curIndex.j = ptj+1;
        curIndex.k = ptk;
        neighbours.push_back(curIndex);
    }
    if(ptk!=0)
    {
        curIndex.i = pti;
        curIndex.j = ptj;
        curIndex.k = ptk-1;
        neighbours.push_back(curIndex);
    }
    if(ptk!=7)
    {
        curIndex.i = pti;
        curIndex.j = ptj;
        curIndex.k = ptk+1;
        neighbours.push_back(curIndex);
    }
    
    vector<struct index>::iterator iter;
    Vector3D FtempStruct;      //force of 1 struct spring
    
    Fstruct.x = 0;
    Fstruct.y = 0;
    Fstruct.z = 0;
    
    FtempStruct.x = 0;
    FtempStruct.y = 0;
    FtempStruct.z = 0;
    
    //  iterate through neighbours
    //  find force exerted by neighbour(i,j,k) on p(i,j,k)
    for (iter = neighbours.begin(); iter!=neighbours.end(); iter++)
    {
        int ni,nj,nk; //neighbour i,j,k
        ni = iter->i;
        nj = iter->j;
        nk = iter->k;
        
        ForceSpring f(pos[pti][ptj][ptk], pos[ni][nj][nk],vel[pti][ptj][ptk], vel[ni][nj][nk],factor,0.5,1.0/SIZE);
        FtempStruct += f.value;
    }
    Fstruct = FtempStruct;

}

void computeCrossForces(int pti, int ptj, int ptk, Vector3D &Fshear)
{
    vector<struct index> Shneighbours;
    struct index curIndex;
    
    //i,j-1,k-1; i,j-1,k+1
    if(ptj-1>=0)
    {
        if(ptk-1>=0)
        {
            curIndex.i = pti;
            curIndex.j = ptj-1;
            curIndex.k = ptk-1;
            Shneighbours.push_back(curIndex);
        }
        if(ptk+1<=7)
        {
            curIndex.i = pti;
            curIndex.j = ptj-1;
            curIndex.k = ptk+1;
            Shneighbours.push_back(curIndex);
        }
    }
    //i,j+1,k-1;i,j+1,k+1
    if(ptj+1<=7)
    {
        if(ptk-1>=0)
        {
            curIndex.i = pti;
            curIndex.j = ptj+1;
            curIndex.k = ptk-1;
            Shneighbours.push_back(curIndex);
        }
        if(ptk+1<=7)
        {
            curIndex.i = pti;
            curIndex.j = ptj+1;
            curIndex.k = ptk+1;
            Shneighbours.push_back(curIndex);
        }
    }
    //i-1
    if(pti-1>=0)
    {
        if(ptk-1>=0)
        {
            curIndex.i = pti-1;
            curIndex.j = ptj;
            curIndex.k = ptk-1;
            Shneighbours.push_back(curIndex);
        }
        if(ptk+1<=7)
        {
            curIndex.i = pti-1;
            curIndex.j = ptj;
            curIndex.k = ptk+1;
            Shneighbours.push_back(curIndex);
        }
        if(ptj-1>=0)
        {
            {   //i-1,j-1,k
                curIndex.i = pti-1;
                curIndex.j = ptj-1;
                curIndex.k = ptk;
                Shneighbours.push_back(curIndex);
            }
            if(ptk-1>=0)
            {
                curIndex.i = pti-1;
                curIndex.j = ptj-1;
                curIndex.k = ptk-1;
                Shneighbours.push_back(curIndex);
            }
            if(ptk+1<=7)
            {
                curIndex.i = pti-1;
                curIndex.j = ptj-1;
                curIndex.k = ptk+1;
                Shneighbours.push_back(curIndex);
            }
        }
        if(ptj+1<=7)
        {
            {   //i-1,j+1,k
                curIndex.i = pti-1;
                curIndex.j = ptj+1;
                curIndex.k = ptk;
                Shneighbours.push_back(curIndex);
            }
            if(ptk-1>=0)
            {
                curIndex.i = pti-1;
                curIndex.j = ptj+1;
                curIndex.k = ptk-1;
                Shneighbours.push_back(curIndex);
            }
            if(ptk+1<=7)
            {
                curIndex.i = pti-1;
                curIndex.j = ptj+1;
                curIndex.k = ptk+1;
                Shneighbours.push_back(curIndex);
            }
        }
    }
    if(pti+1<=7)
    {
        if(ptk-1>=0)
        {
            curIndex.i = pti+1;
            curIndex.j = ptj;
            curIndex.k = ptk-1;
            Shneighbours.push_back(curIndex);
        }
        if(ptk+1<=7)
        {
            curIndex.i = pti+1;
            curIndex.j = ptj;
            curIndex.k = ptk+1;
            Shneighbours.push_back(curIndex);
        }
        if(ptj-1>=0)
        {
            {   //i+1,j-1,k
                curIndex.i = pti+1;
                curIndex.j = ptj-1;
                curIndex.k = ptk;
                Shneighbours.push_back(curIndex);
            }
            if(ptk-1>=0)
            {
                curIndex.i = pti+1;
                curIndex.j = ptj-1;
                curIndex.k = ptk-1;
                Shneighbours.push_back(curIndex);
            }
            if(ptk+1<=7)
            {
                curIndex.i = pti+1;
                curIndex.j = ptj-1;
                curIndex.k = ptk+1;
                Shneighbours.push_back(curIndex);
            }
        }
        if(ptj+1<=7)
        {
            {   //i+1,j+1,k
                curIndex.i = pti+1;
                curIndex.j = ptj+1;
                curIndex.k = ptk;
                Shneighbours.push_back(curIndex);
            }
            if(ptk-1>=0)
            {
                curIndex.i = pti+1;
                curIndex.j = ptj+1;
                curIndex.k = ptk-1;
                Shneighbours.push_back(curIndex);
            }
            if(ptk+1<=7)
            {
                curIndex.i = pti+1;
                curIndex.j = ptj+1;
                curIndex.k = ptk+1;
                Shneighbours.push_back(curIndex);
            }
        }
    }
    vector<struct index>::iterator iter;
    Vector3D FtempShear;
    
    Fshear.x = 0;
    Fshear.y = 0;
    Fshear.z = 0;
    
    FtempShear.x = 0;
    FtempShear.y = 0;
    FtempShear.z = 0;
    
    //iterate through shear neighbours
    for (iter = Shneighbours.begin(); iter!=Shneighbours.end(); iter++)
    {
        int ni,nj,nk;
        ni = iter->i;
        nj = iter->j;
        nk = iter->k;
        
        //Rest length of shear springs
        double Rshlength1,Rshlength2;
        Rshlength1 = sqrt(2) * 1/SIZE; //square diagonal
        Rshlength2 = sqrt(3) * 1/SIZE; //cube diagonal
        
        if((abs(pti-ni)==1)&&(abs(ptj-nj)==1)&&(abs(ptk-nk)==1)) //cube diagonal
        {
            ForceSpring f(pos[pti][ptj][ptk], pos[ni][nj][nk],vel[pti][ptj][ptk], vel[ni][nj][nk],factor,0.5,Rshlength2);
            FtempShear += f.value;
        }
        else {
            ForceSpring f(pos[pti][ptj][ptk], pos[ni][nj][nk],vel[pti][ptj][ptk], vel[ni][nj][nk],factor,0.5,Rshlength1);
            FtempShear += f.value;
        }
        Fshear = FtempShear;
    }
}

void computeBendForces(int pti, int ptj, int ptk, Vector3D &Fbend)
{
    vector<struct index> Bneighbours;
    struct index curIndex;
    
    if(pti>=2)
    {
        curIndex.i = pti-2;
        curIndex.j = ptj;
        curIndex.k = ptk;
        Bneighbours.push_back(curIndex);
    }
    if(pti<=5)
    {
        curIndex.i = pti+2;
        curIndex.j = ptj;
        curIndex.k = ptk;
        Bneighbours.push_back(curIndex);
    }
    if(ptj>=2)
    {
        curIndex.i = pti;
        curIndex.j = ptj-2;
        curIndex.k = ptk;
        Bneighbours.push_back(curIndex);
    }
    if(ptj<=5)
    {
        curIndex.i = pti;
        curIndex.j = ptj+2;
        curIndex.k = ptk;
        Bneighbours.push_back(curIndex);
    }
    if(ptk>=2)
    {
        curIndex.i = pti;
        curIndex.j = ptj;
        curIndex.k = ptk-2;
        Bneighbours.push_back(curIndex);
    }
    if(ptk<=5)
    {
        curIndex.i = pti;
        curIndex.j = ptj;
        curIndex.k = ptk+2;
        Bneighbours.push_back(curIndex);
    }
    
    vector<struct index>::iterator iter;
    Fbend.x = 0;
    Fbend.y = 0;
    Fbend.z = 0;
    
    Vector3D FtempBend;
    FtempBend.x = 0;
    FtempBend.y = 0;
    FtempBend.z = 0;
    
    //iterate through bend neighbours
    for (iter = Bneighbours.begin(); iter!=Bneighbours.end(); iter++)
    {
        //neighbour i,j,k
        int ni,nj,nk;
        ni = iter->i;
        nj = iter->j;
        nk = iter->k;
        
        double Rblength;
        Rblength = 2.0/SIZE;
        
        ForceSpring f(pos[pti][ptj][ptk], pos[ni][nj][nk],vel[pti][ptj][ptk], vel[ni][nj][nk],factor,0.5,Rblength);
        FtempBend += f.value;

    }
    Fbend = FtempBend;
}

Force applyForce()
{
    Force Ftotal;

    //external forces
    float gx = -0.0;
    float gy = -1.0;
    float gz = -0.3;

    for (int n = 0; n < N; n++){
        Force gravity(Vector3D(s[n]->getMass() * gx, s[n]->getMass() * gy, s[n]->getMass() * gz));
        ForceAir air(s[n]->getVelocity());

        gravity.addTo(*s[n]);
        air.addTo(*s[n]);
    }
    
    //edge forces
    for(int i =0; i<SIZE; i++){
        for(int j =0; j<SIZE; j++){
            for(int k =0; k<SIZE; k++){
                int n = i*SIZE*SIZE + j*SIZE + k;
                Force Fstruct;
                computeStructForces( i, j, k, Fstruct.value);
                
                Force Fshear;
                computeCrossForces(i, j, k, Fshear.value);
                
                Force Fbend;
                computeBendForces(i, j, k, Fbend.value);
                
                Ftotal.value = Fstruct.value + Fshear.value + Fbend.value;
                // Keep the system stable by cutting the forces in half when it goes beyond control
//                if((Ftotal.value.x>70)||(Ftotal.value.x<-70))
//                    Ftotal.value.x/=2;
//                
//                if((Ftotal.value.y>70)||(Ftotal.value.y<-70))
//                    Ftotal.value.y/=2;
//                
//                if((Ftotal.value.z>70)||(Ftotal.value.z<-70))
//                    Ftotal.value.z/=2;
                Ftotal.addTo(*s[n]);
                
            }
        }
    }
    //face forces
    return Ftotal;
}
void computeAcceleration(struct RK4world *jello, Vector3D a[SIZE][SIZE][SIZE])
{
    Force Ftotal = applyForce();
    
    for(int i =0; i<8; i++)
        for(int j =0; j<8; j++)
            for(int k =0; k<8; k++)
            {
                
//                // check for collission with the box
//                // if point was inside the box
//                if(jello->inBox[i][j][k])
//                {
//                    collided = 0;
//                    //check collision
//                    if  ((jello->p[i][j][k].x<=-2)||(jello->p[i][j][k].x>=2) ||
//                         (jello->p[i][j][k].y<=-2)||(jello->p[i][j][k].y>=2) ||
//                         (jello->p[i][j][k].z<=-2)||(jello->p[i][j][k].z>=2)  )
//                    {
//                        //collided - point outside the box now
//                        jello->inBox[i][j][k] = false;
//                        collided++;
//                        
//                        //first collision for collision point
//                        if (collided==1)
//                        {
//                            jello->pCollission[i][j][k].x =jello->p[i][j][k].x;
//                            jello->pCollission[i][j][k].y =jello->p[i][j][k].y;
//                            jello->pCollission[i][j][k].z =jello->p[i][j][k].z;
//                        }
//                    }
//                }
//                else // point was outside the box
//                {
//                    // check if point inside the box now
//                    if((jello->p[i][j][k].x>=-2) && (jello->p[i][j][k].x<=2)&&
//                       (jello->p[i][j][k].y>=-2) && (jello->p[i][j][k].y<=2)&&
//                       (jello->p[i][j][k].z>=-2) && (jello->p[i][j][k].z<=2) )
//                    {
//                        jello->inBox[i][j][k] = true;
//                    }
//                    else
//                        //point still outside, find force due to collission
//                        //calculate elastic force between surface and current position
//                        //create spring between pCollission[0][0][0] and current p[0][0][0]
//                    {
//                        Point3D L;
//                        pDIFFERENCE(jello->p[i][j][k],jello->pCollission[i][j][k],L );
//                        
//                        //velocity difference for damping
//                        Point3D       diffVel, vpCollision;
//                        pMAKE      (0, 0, 0, vpCollision);
//                        pDIFFERENCE(jello->v[i][j][k],vpCollision,diffVel );
//                        
//                        //length of L
//                        double absL;
//                        absL = sqrt((L).x * (L).x + (L).y * (L).y + (L).z * (L).z);
//                        
//                        //normal vector
//                        Point3D Nfloor; //normal to floor
//                        
//                        //update Nfloor according to the colliding wall
//                        if(jello->p[i][j][k].x<=-2)
//                        {
//                            pMAKE(1,0,0,Nfloor);
//                        }
//                        
//                        else if(jello->p[i][j][k].x>=2)
//                        {
//                            pMAKE(-1,0,0,Nfloor);
//                        }
//                        
//                        else if(jello->p[i][j][k].y<=-2)
//                        {
//                            pMAKE(0,1,0,Nfloor);
//                        }
//                        
//                        else if(jello->p[i][j][k].y>=2)
//                        {
//                            pMAKE(0,-1,0,Nfloor);
//                        }
//                        
//                        else if(jello->p[i][j][k].z<=-2)
//                        {
//                            pMAKE(0,0,1,Nfloor);
//                        }
//                        else if(jello->p[i][j][k].z>=2)
//                        {
//                            pMAKE(0,0,-1,Nfloor);
//                        }
//                        
//                        //dot product for damping
//                        double vDotL;
//                        DOTPRODUCT(diffVel, L, vDotL);
//                        
//                        //collission force magnitude
//                        double FcolMag;
//                        FcolMag = jello->kCollision * absL;
//                        
//                        //collision force vector
//                        Point3D FcolV;
//                        pMULTIPLY(Nfloor, FcolMag, FcolV);
//                        
//                        //damping force magnitude
//                        double FcolDamp;
//                        FcolDamp = ( jello->dCollision * vDotL) / absL;
//                        
//                        //direction of Fdamp = Fdamp1 * normal of floor
//                        Point3D FcolDampV;
//                        pMULTIPLY(Nfloor, FcolDamp , FcolDampV);
//                        pSUM     (FcolV , FcolDampV, FcolV    );
//                        
//                        //add collision force to total force
//                        pSUM(Ftotal.value, FcolV, Ftotal.value);
//                    }
//                }
                
                
                Point3D accel;
                pMULTIPLY( Ftotal.value, (1/s[0]->getMass()), accel);
                
                // Keep the system stable by cutting the forces in half when it goes beyond control
                if((Ftotal.value.x>70)||(Ftotal.value.x<-70))
                    Ftotal.value.x/=2;
                
                if((Ftotal.value.y>70)||(Ftotal.value.y<-70))
                    Ftotal.value.y/=2;
                
                if((Ftotal.value.z>70)||(Ftotal.value.z<-70))
                    Ftotal.value.z/=2;
                
                //Update a[][][] matrix
                a[i][j][k] = accel;
            }
}

void RK4(struct RK4world *jello)
{
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            for (int k = 0; k < SIZE; k++) {
                int n = i*SIZE*SIZE + j*SIZE + k;

                jello->p[i][j][k] = s[n]->getPosition();
                jello->v[i][j][k] = s[n]->getVelocity();
            }
        }
    }

    Vector3D F1p[8][8][8], F1v[8][8][8],
    F2p[8][8][8], F2v[8][8][8],
    F3p[8][8][8], F3v[8][8][8],
    F4p[8][8][8], F4v[8][8][8];
    
    Vector3D a[8][8][8];
    
    struct RK4world buffer;
    int    i, j, k     ;
    
    // make a copy of jello
    buffer = *jello;
    
    computeAcceleration(jello, a);
    for(i=0; i<=7; i++)
        for(j=0; j<=7; j++)
            for(k=0; k<=7; k++)
            {
                pMULTIPLY(jello->v[i][j][k], jello->h, F1p[i][j][k]     );
                pMULTIPLY(a[i][j][k]       , jello->h, F1v[i][j][k]     );
                pMULTIPLY(F1p[i][j][k]     , 0.5      , buffer.p[i][j][k]);
                pMULTIPLY(F1v[i][j][k]     , 0.5      , buffer.v[i][j][k]);
                
                pSUM (jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM (jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }
    
    
    computeAcceleration(&buffer, a);
    for(i=0; i<=7; i++)
        for(j=0; j<=7; j++)
            for(k=0; k<=7; k++)
            {
                //    F2p = dt * buffer.v;
                pMULTIPLY ( buffer.v[i][j][k], jello->h, F2p[i][j][k]     );
                //    F2v = dt * a(buffer.p,buffer.v);
                pMULTIPLY ( a[i][j][k]       , jello->h, F2v[i][j][k]     );
                pMULTIPLY ( F2p[i][j][k]     , 0.5      , buffer.p[i][j][k]);
                pMULTIPLY ( F2v[i][j][k]     , 0.5      , buffer.v[i][j][k]);
                
                pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }
    
    computeAcceleration(&buffer, a);
    for(i=0; i<=7; i++)
        for(j=0; j<=7; j++)
            for(k=0; k<=7; k++)
            {
                //   F3p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k], jello->h, F3p[i][j][k]     );
                //   F3v = dt * a(buffer.p,buffer.v);
                pMULTIPLY(a[i][j][k]       , jello->h, F3v[i][j][k]     );
                pMULTIPLY(F3p[i][j][k]     , 0.5      , buffer.p[i][j][k]);
                pMULTIPLY(F3v[i][j][k]     , 0.5      , buffer.v[i][j][k]);
                
                pSUM (jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
                pSUM (jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
            }
    
    computeAcceleration(&buffer, a);
    for(i=0; i<=7; i++)
        for(j=0; j<=7; j++)
            for(k=0; k<=7; k++)
            {
                // F3p = dt * buffer.v;
                pMULTIPLY (buffer.v[i][j][k], jello->h, F4p[i][j][k]);
                // F3v = dt * a(buffer.p,buffer.v);
                pMULTIPLY (a[i][j][k]       , jello->h, F4v[i][j][k]);
                
                pMULTIPLY (F2p[i][j][k], 2, buffer.p[i][j][k]        );
                pMULTIPLY (F3p[i][j][k], 2, buffer.v[i][j][k]        );
                
                pSUM      (buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM      (buffer.p[i][j][k], F1p[i][j][k]     , buffer.p[i][j][k]);
                pSUM      (buffer.p[i][j][k], F4p[i][j][k]     , buffer.p[i][j][k]);
                pMULTIPLY (buffer.p[i][j][k], 1.0 / 6          , buffer.p[i][j][k]);
                pSUM      (buffer.p[i][j][k], jello->p[i][j][k], jello->p[i][j][k]);
                
                pMULTIPLY (F2v[i][j][k]     , 2                , buffer.p[i][j][k]);
                pMULTIPLY (F3v[i][j][k]     , 2                , buffer.v[i][j][k]);
                pSUM      (buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
                pSUM      (buffer.p[i][j][k], F1v[i][j][k]     , buffer.p[i][j][k]);
                pSUM      (buffer.p[i][j][k], F4v[i][j][k]     , buffer.p[i][j][k]);
                pMULTIPLY (buffer.p[i][j][k], 1.0 / 6          , buffer.p[i][j][k]);
                pSUM      (buffer.p[i][j][k], jello->v[i][j][k], jello->v[i][j][k]);
                
                int n = i*SIZE*SIZE+j*SIZE+k;
                s_next[n]->setVelocity(jello->v[i][j][k]);
                s_next[n]->setPosition(jello->p[i][j][k]);
                s_next[n]->setColor(s[n]->getColor());
                s_next[n]->setMass(s[n]->getMass());

            }
    for (int n = 0; n <N; n++) {
        w.checkCollision(*s[n], *s_next[n]);
    }
    s = s_next;
    RK4time = RK4time + jello->h;
    return;
}

void mouseMotionDrag(int x, int y)
{
    int vMouseDelta[2] = {x-g_vMousePos[0], y-g_vMousePos[1]};
    if (g_iRightMouseButton) // handle camera rotations
    {
        Phi   += vMouseDelta[0] * 0.01;
        Theta += vMouseDelta[1] * 0.01;
        
        if (Phi  > 2*pi)
            Phi -= 2*pi;
        
        if (Phi < 0)
            Phi += 2*pi;
        
        if (Theta > pi / 2 - 0.01)
            Theta = pi / 2 - 0.01;
        
        if (Theta <-pi / 2 + 0.01)
            Theta =-pi / 2 + 0.01;
        
        g_vMousePos[0] = x;
        g_vMousePos[1] = y;
    }
}

void mouseMotion (int x, int y)
{
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}

void mouseButton(int button, int state, int x, int y)
{
    switch (button)
    {
        case GLUT_LEFT_BUTTON:
            g_iLeftMouseButton = (state==GLUT_DOWN);
            break;
            
        case GLUT_MIDDLE_BUTTON:
            g_iMiddleMouseButton = (state==GLUT_DOWN);
            break;
            
        case GLUT_RIGHT_BUTTON:
            g_iRightMouseButton = (state==GLUT_DOWN);
            break;
    }
    
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}

// gets called whenever a key is pressed
void keyboardFunc (unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27:
            exit(0);
            break;
            
        case 'e':
            Theta = pi / 6;
            Phi = pi / 6;
            viewingMode = 0;
            break;
            
        case 'v':
            viewingMode = 1 - viewingMode;
            break;
            
        case 'h':
            shear = 1 - shear;
            break;
            
        case 's':
            structural = 1 - structural;
            break;
            
        case 'b':
            bend = 1 - bend;
            break;
            
        case 'p':
            doPause = 1 - doPause;
            break;
            
        case 'z':
            R -= 0.2;
            if (R < 0.2)
                R = 0.2;
            break;
            
        case 'x':
            R += 0.2;
            break;
            
        case ' ':
            saveScreenToFile = 1 - saveScreenToFile;
            break;
    }
}
void idle() {
    double start;
    clock_t start_time;
    if (isEuler) {

    start = ei.getTime();
    start_time = clock();
        applyForce();
        for (int n = 0; n<N; n++) {
            ei.update(*s[n], *s_next[n], w);
        }
    }else{
        start = RK4time;
        start_time = clock();

        RK4(&jello);
    }
    
    for(int i =0; i<SIZE; i++){
        for(int j =0; j<SIZE; j++){
            for(int k =0; k<SIZE; k++){
                int n = i*SIZE*SIZE + j*SIZE + k;
                pos[i][j][k] = s[n]->getPosition();
                vel[i][j][k] = s[n]->getVelocity();
            }
        }
    }
    double tau;
if (isEuler) {
    tau = 1.0 / frames_per_second;
    while ((ei.getTime()) - start < tau){
        
            applyForce();
            for (int n = 0; n<N; n++) {
                ei.update(*s[n], *s_next[n], w);
                
            }
    }
        }else{
            tau = 1.0 / frames_per_second;
            while ((RK4time) - start < tau){

            RK4(&jello);
            }
        }
    
        for(int i =0; i<SIZE; i++){
            for(int j =0; j<SIZE; j++){
                for(int k =0; k<SIZE; k++){
                    int n = i*SIZE*SIZE + j*SIZE + k;
                    pos[i][j][k] = s[n]->getPosition();
                    vel[i][j][k] = s[n]->getVelocity();
                }
            }
        }
    
    while (((double)(clock()) - start_time) / CLOCKS_PER_SEC < tau)
        ;
    //        double start = rk.getTime();
//        clock_t start_time = clock();
//        applyForce();
//        rk.update(s, s_next, w);
//        for(int i =0; i<SIZE; i++)
//            for(int j =0; j<SIZE; j++)
//                for(int k =0; k<SIZE; k++){
//                    int n = i*SIZE*SIZE + j*SIZE + k;
//                    pos[i][j][k] = s[n]->getPosition();
//                    vel[i][j][k] = s[n]->getVelocity();
//                }
//        applyForce();
//        rk.update2(s, s_next, w);
//        for(int i =0; i<SIZE; i++)
//            for(int j =0; j<SIZE; j++)
//                for(int k =0; k<SIZE; k++){
//                    int n = i*SIZE*SIZE + j*SIZE + k;
//                    pos[i][j][k] = s[n]->getPosition();
//                    vel[i][j][k] = s[n]->getVelocity();
//                }
//        applyForce();
//        rk.update3(s, s_next, w);
//        for(int i =0; i<SIZE; i++)
//            for(int j =0; j<SIZE; j++)
//                for(int k =0; k<SIZE; k++){
//                    int n = i*SIZE*SIZE + j*SIZE + k;
//                    pos[i][j][k] = s[n]->getPosition();
//                    vel[i][j][k] = s[n]->getVelocity();
//                }
//        applyForce();
//        rk.update4(s, s_next, w);
//        for(int i =0; i<SIZE; i++)
//            for(int j =0; j<SIZE; j++)
//                for(int k =0; k<SIZE; k++){
//                    int n = i*SIZE*SIZE + j*SIZE + k;
//                    pos[i][j][k] = s[n]->getPosition();
//                    vel[i][j][k] = s[n]->getVelocity();
//                }
//        
//        
//        double tau = 1.0 / frames_per_second;
//        while ((rk.getTime()) - start < tau){
//            applyForce();
//            rk.update(s, s_next, w);
//            for(int i =0; i<SIZE; i++)
//                for(int j =0; j<SIZE; j++)
//                    for(int k =0; k<SIZE; k++){
//                        int n = i*SIZE*SIZE + j*SIZE + k;
//                        pos[i][j][k] = s[n]->getPosition();
//                        vel[i][j][k] = s[n]->getVelocity();
//                    }
//            applyForce();
//            rk.update2(s, s_next, w);
//            for(int i =0; i<SIZE; i++)
//                for(int j =0; j<SIZE; j++)
//                    for(int k =0; k<SIZE; k++){
//                        int n = i*SIZE*SIZE + j*SIZE + k;
//                        pos[i][j][k] = s[n]->getPosition();
//                        vel[i][j][k] = s[n]->getVelocity();
//                    }
//            applyForce();
//            rk.update3(s, s_next, w);
//            for(int i =0; i<SIZE; i++)
//                for(int j =0; j<SIZE; j++)
//                    for(int k =0; k<SIZE; k++){
//                        int n = i*SIZE*SIZE + j*SIZE + k;
//                        pos[i][j][k] = s[n]->getPosition();
//                        vel[i][j][k] = s[n]->getVelocity();
//                    }
//            applyForce();
//            rk.update4(s, s_next, w);
//            for(int i =0; i<SIZE; i++)
//                for(int j =0; j<SIZE; j++)
//                    for(int k =0; k<SIZE; k++){
//                        int n = i*SIZE*SIZE + j*SIZE + k;
//                        pos[i][j][k] = s[n]->getPosition();
//                        vel[i][j][k] = s[n]->getVelocity();
//                    }
//        }
//        while (((double)(clock()) - start_time) / CLOCKS_PER_SEC < tau)
//            ;
//
    
    glutPostRedisplay();
    
}
int pointMap(int side, int i, int j)
{
    int r;
    
    switch (side)
    {
        case 1: //[i][j][0] bottom face
            r = 64 * i + 8 * j;
            break;
        case 6: //[i][j][7] top face
            r = 64 * i + 8 * j + 7;
            break;
        case 2: //[i][0][j] front face
            r = 64 * i + j;
            break;
        case 5: //[i][7][j] back face
            r = 64 * i + 56 + j;
            break;
        case 3: //[0][i][j] left face
            r = 8 * i + j;
            break;
        case 4: //[7][i][j] right face
            r = 448 + 8 * i + j;
            break;
    }
    return r;
}

void showCube()
{
    int    i,  j,  k,
    ip, jp, kp;
    int n, n1;
    
    point r1, r2, r3; // aux variables
    
    /* normals buffer and counter for Gourad shading*/
    struct point normal[8][8];
    int         counter[8][8];
    
    int    face;
    double faceFactor, length;
    
    
    
#define NODE(face,i,j) (*((Point3D * )(pos) + pointMap((face),(i),(j))))
    
#define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);                        \
    jp=j+(dj);                        \
    kp=k+(dk);                        \
    if                                \
        (!  ((ip >7)|| (ip<0) || (jp>7) || (jp<0) || ( kp>7) || (kp<0)  )                  \
         &&  (( i==0)|| (i==7) || (j==0) || (j==7) || ( k==0) || (k==7)  )                  \
         &&  ((ip==0)|| (ip==7)|| (jp==0)|| (jp==7)|| (kp==0) || (kp==7) )                  \
         )                                                                                  \
    {                                                                                  \
        n = i*SIZE*SIZE+j*SIZE+k;\
        n1 = ip*SIZE*SIZE+jp*SIZE+kp;\
        glVertex3f(s[n]->getPosition().x, s[n]->getPosition().y, s[n]->getPosition().z);         \
        glVertex3f(s[n1]->getPosition().x, s[n1]->getPosition().y, s[n1]->getPosition().z);\
    }                                                                                  \
    
    if (viewingMode==0) // render wireframe
    {
        glLineWidth(1);
        glPointSize(5);
        glDisable(GL_LIGHTING);
        for (i=0; i<=7; i++)
            for (j=0; j<=7; j++)
                for (k=0; k<=7; k++)
                {
                    if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
                        continue;
                    
                    glBegin    (GL_POINTS ); // draw point
                    glColor4f  (1, 1, 1, 0);
                    int n = i*SIZE*SIZE+j*SIZE+k;
                    glVertex3f (s[n]->getPosition().x, s[n]->getPosition().y, s[n]->getPosition().z);
                    glEnd      ();
                    
                    glBegin(GL_LINES);
                    // structural springs
                    if (structural == 1)
                    {
                        glColor4f(0,0,1,1);
                        PROCESS_NEIGHBOUR( 1, 0, 0);
                        PROCESS_NEIGHBOUR( 0, 1, 0);
                        PROCESS_NEIGHBOUR( 0, 0, 1);
                        PROCESS_NEIGHBOUR(-1, 0, 0);
                        PROCESS_NEIGHBOUR( 0,-1, 0);
                        PROCESS_NEIGHBOUR( 0, 0,-1);
                    }
                    // shear springs
                    if (shear == 1)
                    {
                        glColor4f(0,1,0,1);
                        PROCESS_NEIGHBOUR( 1, 1, 0);
                        PROCESS_NEIGHBOUR(-1, 1, 0);
                        PROCESS_NEIGHBOUR(-1,-1, 0);
                        PROCESS_NEIGHBOUR( 1,-1, 0);
                        PROCESS_NEIGHBOUR( 0, 1, 1);
                        PROCESS_NEIGHBOUR( 0,-1, 1);
                        PROCESS_NEIGHBOUR( 0,-1,-1);
                        PROCESS_NEIGHBOUR( 0, 1,-1);
                        PROCESS_NEIGHBOUR( 1, 0, 1);
                        PROCESS_NEIGHBOUR(-1, 0, 1);
                        PROCESS_NEIGHBOUR(-1, 0,-1);
                        PROCESS_NEIGHBOUR( 1, 0,-1);
                        
                        PROCESS_NEIGHBOUR( 1, 1, 1);
                        PROCESS_NEIGHBOUR(-1, 1, 1);
                        PROCESS_NEIGHBOUR(-1,-1, 1);
                        PROCESS_NEIGHBOUR( 1,-1, 1);
                        PROCESS_NEIGHBOUR( 1, 1,-1);
                        PROCESS_NEIGHBOUR(-1, 1,-1);
                        PROCESS_NEIGHBOUR(-1,-1,-1);
                        PROCESS_NEIGHBOUR( 1,-1,-1);
                    }
                    
                    // bend
                    if (bend == 1)
                    {
                        glColor4f(1,0,0,1);
                        PROCESS_NEIGHBOUR(2 , 0, 0);
                        PROCESS_NEIGHBOUR(0 , 2, 0);
                        PROCESS_NEIGHBOUR(0 , 0, 2);
                        PROCESS_NEIGHBOUR(-2, 0, 0);
                        PROCESS_NEIGHBOUR(0 ,-2, 0);
                        PROCESS_NEIGHBOUR(0 , 0,-2);
                    }
                    glEnd();
                }
        glEnable(GL_LIGHTING);
    }
    
    else
    {
        glPolygonMode(GL_FRONT, GL_FILL);
        
        for (face=1; face <= 6; face++)
            // face == face of a cube
            //  1 = bottom, 2 = front ,
            //  3 = left  , 4 = right ,
            //  5 = far   , 6 = top
        {
            if((face==1) || (face==3) || (face==5))
                faceFactor =-1;    // flip orientation
            else
                faceFactor = 1;
            
            for (i=0; i <= 7; i++) // reset buffers
                for (j=0; j <= 7; j++)
                {
                    normal[i][j].x=0;
                    normal[i][j].y=0;
                    normal[i][j].z=0;
                    counter[i][j] =0;
                }
            
            // process triangles, accumulate normals for Gourad shading
            for (i=0; i <= 6; i++)
                for(j=0; j <= 6; j++) // process block (i,j)
                {
                    pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1)     ; // first triangle
                    pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2)     ;
                    CROSSPRODUCTp(r1,r2,r3) ;
                    pMULTIPLY(r3,faceFactor,r3);
                    pNORMALIZE(r3);
                    pSUM(normal[i+1][j],r3,normal[i+1][j]);
                    counter[i+1][j]++;
                    pSUM(normal[i][j+1],r3,normal[i][j+1]);
                    counter[i][j+1]++;
                    pSUM(normal[i][j],r3,normal[i][j]);
                    counter[i][j]++;
                    
                    pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
                    pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
                    CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
                    pNORMALIZE(r3);
                    pSUM(normal[i+1][j],r3,normal[i+1][j]);
                    counter[i+1][j]++;
                    pSUM(normal[i][j+1],r3,normal[i][j+1]);
                    counter[i][j+1]++;
                    pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
                    counter[i+1][j+1]++;
                }
            
            /* the actual rendering */
            for(j=1; j<=7; j++) 
            {
                if (faceFactor > 0)
                    glFrontFace(GL_CCW); // the usual definition of front face
                else
                    glFrontFace(GL_CW); // flip definition of orientation
                
                glBegin(GL_TRIANGLE_STRIP);
                for(i=0; i<=7; i++)
                {
                    glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
                               normal[i][j].z / counter[i][j]);
                    glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
                    glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
                               normal[i][j-1].z / counter[i][j-1]);
                    glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
                }
                glEnd();
            }
        } 
    } // end for loop over faces
    glFrontFace(GL_CCW);
}

void display()
{
    glClear       (GL_COLOR_BUFFER_BIT |
                   GL_DEPTH_BUFFER_BIT );
    glMatrixMode  (GL_MODELVIEW        );
    
    glLoadIdentity();
    
    // camera parameters - Phi, Theta, R
    gluLookAt(R * cos(Phi) * cos (Theta),
              R * sin(Phi) * cos (Theta),
              R * sin (Theta),
              0.0,0.0,0.0, 0.0,0.0,1.0 );
    
    //gluLookAt(-3, 3,  1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    // global ambient light
    GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };
    
    // light 's ambient, diffuse, specular components
    GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };
    
    GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
    GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };
    
    GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
    GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };
    
    GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
    GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };
    
    GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
    GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };
    
    GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
    GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };
    
    GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };
    
    GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
    GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };
    
    // light positions and directions
    GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
    GLfloat lP1[] = {  1.999, -1.999, -1.999, 1.0 };
    GLfloat lP2[] = {  1.999,  1.999, -1.999, 1.0 };
    GLfloat lP3[] = { -1.999,  1.999, -1.999, 1.0 };
    GLfloat lP4[] = { -1.999, -1.999,  1.999, 1.0 };
    GLfloat lP5[] = {  1.999, -1.999,  1.999, 1.0 };
    GLfloat lP6[] = {  1.999,  1.999,  1.999, 1.0 };
    GLfloat lP7[] = { -1.999,  1.999,  1.999, 1.0 };
    
    // jelly material color
    GLfloat mKa[] = { 0, 0.2, 0.4, 1.0 };
    GLfloat mKd[] = { 0, 0.0, 0.4, 1.0 };
    GLfloat mKs[] = { 0, 0.0, 0.4, 1.0 };
    GLfloat mKe[] = { 0, 0.0, 0.4, 1.0 };
    
    /* set up lighting */
    glLightModelfv (GL_LIGHT_MODEL_AMBIENT     , aGa     );
    glLightModelf  (GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );
    glLightModelf  (GL_LIGHT_MODEL_TWO_SIDE    , GL_FALSE);
    
    // set up cube color
    glMaterialfv (GL_FRONT, GL_AMBIENT  , mKa);
    glMaterialfv (GL_FRONT, GL_DIFFUSE  , mKd);
    glMaterialfv (GL_FRONT, GL_SPECULAR , mKs);
    glMaterialfv (GL_FRONT, GL_EMISSION , mKe);
    glMaterialf  (GL_FRONT, GL_SHININESS, 10 );
    
    // macro to set up light i
#define LIGHTSETUP(i)\
    glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
    glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
    glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
    glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
    glEnable(GL_LIGHT##i)
    
    LIGHTSETUP (0);
    LIGHTSETUP (1);
    LIGHTSETUP (2);
    LIGHTSETUP (3);
    LIGHTSETUP (4);
    LIGHTSETUP (5);
    LIGHTSETUP (6);
    LIGHTSETUP (7);
    
    // enable lighting
    glEnable(GL_LIGHTING   );
    glEnable(GL_DEPTH_TEST);
    
    glPushMatrix();
    glScaled((size)*2, (size)*2, (size)*2);
    glColor3f(1.0, 1.0, 0);
    glutWireCube(1.0);
    glPopMatrix();
    
    showCube();
    
	glutSwapBuffers();
}

void reshape(GLint w, GLint h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    GLfloat aspect = w/h;
    glLoadIdentity();
    if (w <= h) {
        // width is smaller, so stretch out the height
        glOrtho(-4, 4, -4/aspect, 4/aspect, -10.0, 10.0);
    } else {
        // height is smaller, so stretch out the width
        glOrtho(-4*aspect, 4*aspect, -4, 4, -10.0, 10.0);
    }
    glMatrixMode(GL_MODELVIEW);
    
}

void mymouse(int btn,int state,int x,int y)
{
    
}

void keyboard(int key, int xx, int yy) {
    switch (key) {
        case GLUT_KEY_LEFT:
            posx -= 0.1;
            break;
        case GLUT_KEY_RIGHT:
            posx += 0.1;
            break;
        case GLUT_KEY_UP:
            posy += 0.1;
            break;
        case GLUT_KEY_DOWN:
            posy -= 0.1;
            break;
        default:
            break;
    }
}

void init() {
    glMatrixMode   (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective (90.0,1.0,0.01,1000.0);
    
    // set background color to grey
    glClearColor   (0.7, 0.7, 0.7, 0.0);
    
    glCullFace     (GL_BACK);
    glEnable       (GL_CULL_FACE);
    
    glShadeModel   (GL_SMOOTH);
    glEnable       (GL_POLYGON_SMOOTH);
    glEnable       (GL_LINE_SMOOTH);
}


// The usual main function.
int main(int argc, char** argv) {
    initialize();

    
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Springy");
    glShadeModel(GL_FLAT);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mymouse);
    glutSpecialFunc(keyboard);
    glutIdleFunc(idle);
    // callback for mouse drags
    glutMotionFunc (mouseMotionDrag);
    
    // callback for mouse movement
    glutPassiveMotionFunc(mouseMotion);
    
    // callback for mouse button changes
    glutMouseFunc(mouseButton);
    
    // register for keyboard events
    glutKeyboardFunc(keyboardFunc);
    init();
    
    glutMainLoop();
}

