//
//  RigidBodyObject.h
//  rigidBody
//
//  Created by Uriana on 13-11-20.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#ifndef __rigidBody__RigidBodyObject__
#define __rigidBody__RigidBodyObject__

#include <iostream>
#include <vector>
#include "NTmatrix.h"
#include "Vector3D.h"
#include "Particle.h"

using namespace std;
class RigidBodyObject {
public:
    std::vector<Particle *> p;
    std::vector<Particle *> p_init;
    int number;
    
    void RigidBodyToParticle();
    void InitParticleToRigidBody();
    void InitialState(Vector3D linear_v, Vector3D angular_w);
    void RefreshState(Vector3D linear_v, Vector3D angular_w, Vector3D thex, NTmatrix ther);

    void StateToArray(double *y);
    void ArrayToState(double *y);
    
    void DdtStateToArray(double *xdot);
    void ComputeForceAndTorque(double t);
    //double * Dxdt ( double t, int n, double u[] );
    Vector3D getX();
    NTmatrix getR();
    Vector3D getP();
    Vector3D getL();

    Vector3D getOmega();
    Vector3D getV();
    Vector3D getCom();
    std::vector<Vector3D > getr();
    NTmatrix getIinv();
    double getMass();
    
    void setP(Vector3D thep);
    void setL(Vector3D thel);
    void setV(Vector3D thev);
    void setOmega(Vector3D thew);

protected:
    /* Constant quantities */
    
    double mass = 0; /* mass M */
    Vector3D com; /* center of mass */
    std::vector<Vector3D > r; /* relative position of particles to center of mass*/
    NTmatrix Ibody; /* Ibody */
    NTmatrix Ibodyinv; /* I−1  body (inverse of Ibody) */
    
    /* State variables */
    Vector3D x; /* x(t) center of mass*/
    NTmatrix R; /* R(t) */
    Vector3D P; /* P(t) */
    Vector3D L; /* L(t) */
    
    /* Derived quantities (auxiliary variables) */
    NTmatrix Iinv; /* I−1(t) */
    Vector3D v; /* v(t) */
    Vector3D omega; /* w(t) */
    
    /* Computed quantities */
    Vector3D force; /* F(t) */
    Vector3D torque; /* tau(t) */
};
#endif /* defined(__rigidBody__RigidBodyObject__) */
