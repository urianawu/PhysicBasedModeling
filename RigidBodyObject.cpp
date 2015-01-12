//
//  RigidBodyObject.cpp
//  rigidBody
//
//  Created by Uriana on 13-11-20.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "RigidBodyObject.h"
#define STATE_SIZE 18

//compute initial rigid body variables
void RigidBodyObject::InitParticleToRigidBody()
{

    //p_init.resize(8);
    number = (int)p_init.size();
    //mass, com
    for (int n = 0; n < number; n++) {
        mass += p_init[n]->getMass();
        com += p_init[n]->getMass() * p_init[n]->getPosition();
    }
    com = com / mass;
    //r
    for (int n = 0; n < number; n++) {
        r.push_back( p_init[n]->getPosition() - com);
    }
    //Ibody
    Ibody.m = 3;
    Ibody.n = 3;
    Ibody.allocate();
    Ibody.zero();
    for (int n = 0; n < number; n++) {
        NTmatrix identity(3, 3, 'I');
        NTmatrix r_temp(3, 1);
        NTmatrix r_temp_T(1, 3);
        NTmatrix r_outerProduct(3,3);
        NTmatrix r_n(3,3);
        double r_value[3] = {r[n].x,r[n].y,r[n].z};
        r_temp.assign(r_value);
        r_temp_T = r_temp.Transpose();
        r_outerProduct.zero();
        r_outerProduct = r_temp * r_temp_T;
        
        double r_innerProduct = Dot(r[n],r[n]);
        NTmatrix temp(3,3);
        temp.zero();
        temp = identity * r_innerProduct;
        r_n = (temp - r_outerProduct);
        r_n = r_n* p_init[n]->getMass();
        Ibody = Ibody + r_n;
    }
    //Ibodyinv
    Ibodyinv = Ibody.inverse();
    //Ibody.printMat();
    //Ibodyinv.printMat();
}

void RigidBodyObject::InitialState(Vector3D linear_v, Vector3D angular_w)
{
    v = linear_v;
    omega = angular_w;
    
    x = com;
    
    R.n = 3;
    R.m = 3;
    R.allocate();
    R.zero();
    double identity[9] ={1,0,0,0,1,0,0,0,1};
    
    R.assign(identity);
    //R.printMat();
    
    P = mass * v;
    
    NTmatrix w_temp(3, 1);
    NTmatrix L_temp(3, 1);
    double w_value[3] = {omega.x, omega.y, omega.z};
    w_temp.assign(w_value);
    L_temp = Ibody * w_temp;
    L.Set(L_temp.vect[0], L_temp.vect[1], L_temp.vect[2]);
    
}
void RigidBodyObject::RefreshState(Vector3D linear_v, Vector3D angular_w, Vector3D thex, NTmatrix ther)
{
    v = linear_v;
    omega = angular_w;
    
    x = thex;
    R.n = 3;
    R.m = 3;
    R.allocate();
    R.zero();
    R.assign(ther.mat);
    //double value[9] ={ther.vect[0],ther.vect[1],ther.vect[2],ther.vect[3],ther.vect[4],ther.vect[5],ther.vect[6],ther.vect[7],ther.vect[8]};
    
    //R.assign(value);
   
    P = mass * v;
    
    NTmatrix w_temp(3, 1);
    NTmatrix L_temp(3, 1);
    double w_value[3] = {omega.x, omega.y, omega.z};
    w_temp.assign(w_value);
    L_temp = Ibody * w_temp;
    L.Set(L_temp.vect[0], L_temp.vect[1], L_temp.vect[2]);

}
//export rigid body state to particle state
void RigidBodyObject::RigidBodyToParticle()
{
    //p.empty();
    for (int n = 0; n < number; n++) {
        NTmatrix r_mat(3, 1);
        double r_value[3] = {r[n].x,r[n].y,r[n].z};
        r_mat.assign(r_value);
        NTmatrix x_mat(3, 1);
        double x_value[3] = {x.x,x.y,x.z};
        x_mat.assign(x_value);
        NTmatrix pos_mat(3, 1);
        NTmatrix temp(3,1);
        
        temp = R * r_mat;
        pos_mat = x_mat + temp;
        Point3D pos;
        pos.Set(pos_mat.vect[0], pos_mat.vect[1], pos_mat.vect[2]);
        //printf("%d\t",p.size());
        //p.push_back(new Particle());
        p[n]->setPosition(pos);
        //printf("%f\t",p[n]->getPosition().y);
    }
}

/* Copy the state information into an array */
void RigidBodyObject::StateToArray(double *y)
{
    *y++ = x[0]; /* x component of position */
    *y++ = x[1]; /* etc. */
    *y++ = x[2];
    for(int i = 0; i < 3; i++) /* copy rotation matrix */
        for(int j = 0; j < 3; j++)
            *y++ = R.mat[i][j];
    
    *y++ = P[0];
    *y++ = P[1];
    *y++ = P[2];
    *y++ = L[0];
    *y++ = L[1];
    *y++ = L[2];
}

/* Copy information from an array into the state variables */
void RigidBodyObject::ArrayToState(double *y)
{
    x[0] = *y++;
    x[1] = *y++;
    x[2] = *y++;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            R.mat[i][j] = *y++;
    //printf("x %f\n", x.y);
    P[0] = *y++;
    P[1] = *y++;
    P[2] = *y++;
    L[0] = *y++;
    L[1] = *y++;
    L[2] = *y++;
    /* Compute auxiliary variables... */
    /* v(t) = P(t)/M */
    v = P / mass;
    //printf("v %f %f %f\n",v.x,v.y,v.z);
    /* I−1(t) = R(t)*I−1body*R(t)T */
    NTmatrix tmp(3,3);
    tmp = R * Ibodyinv;
    NTmatrix RT(3,3);
    NTmatrix RT_value = R.Transpose();
    RT.assign((RT_value).vect);

    Iinv = tmp * RT;

    /* w(t)= I−1(t)*L(t) */
    NTmatrix L_mat(3, 1);
    double L_value[3] = {L.x,L.y,L.z};
    L_mat.assign(L_value);
    NTmatrix w_mat(3, 1);
    w_mat = Iinv * L_mat;
    omega.Set(w_mat.vect[0], w_mat.vect[1], w_mat.vect[2]);
}


void RigidBodyObject::DdtStateToArray(double *xdot)
{
    /* copy dx(t)/dt = v(t) into xdot */
    *xdot++ = v[0];
    *xdot++ = v[1];
    *xdot++ = v[2];
    /* Compute RP(t) = w(t)R(t) */
    NTmatrix w_mat(3, 1);
    double w_value[3] = {omega.x,omega.y,omega.z};
    w_mat.assign(w_value);
    NTmatrix Rdot = w_mat.Vector3DStar() * R;
    //Rdot.printMat();
    /* copy RP(t) into array */
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            *xdot++ = Rdot.mat[i][j];
    *xdot++ = force[0]; /* d
                             dtP(t) = F(t) */
    *xdot++ = force[1];
    *xdot++ = force[2];
    *xdot++ = torque[0]; /* d
                              dtL(t) = tau(t) */
    *xdot++ = torque[1];
    *xdot++ = torque[2];
}

void RigidBodyObject::ComputeForceAndTorque(double t)
{
    force.Set(0, -1.5*((int)t%1), 0);
    force.Set(0, -0.1, 0);

    srand((unsigned)time(0));
    torque.Set(0.01*rand()/RAND_MAX, 0.01*rand()/RAND_MAX, 0.01*rand()/RAND_MAX);
}

NTmatrix RigidBodyObject::getIinv()
{
    return Iinv;
}

double RigidBodyObject::getMass()
{
    return mass;
}
std::vector<Vector3D > RigidBodyObject::getr()
{
    return r;
}
Vector3D RigidBodyObject::getX()
{
    return x;
}
NTmatrix RigidBodyObject::getR()
{
    return R;
}
Vector3D RigidBodyObject::getP()
{
    return P;
}
Vector3D RigidBodyObject::getL()
{
    return L;
}

Vector3D RigidBodyObject::getOmega()
{
    return omega;
}
Vector3D RigidBodyObject::getV()
{
    return v;
}
Vector3D RigidBodyObject::getCom()
{
    return com;
}

void RigidBodyObject::setP(Vector3D thep)
{
    P = thep;
}
void RigidBodyObject::setL(Vector3D thel)
{
    L = thel;
}
void RigidBodyObject::setV(Vector3D thev)
{
    v = thev;
}
void RigidBodyObject::setOmega(Vector3D thew)
{
    omega = thew;
}
//double * RigidBodyObject::Dxdt ( double t, int n, double u[] )
//{    
//    double *xdot;
//    xdot = new double[n];
//    /* put data in x[] into Body state */
//    ArrayToState(u);
//    ComputeForceAndTorque(t);
//    DdtStateToArray(xdot);
//    return xdot;
//}
//
