//
//  ObstaclePlane.cpp
//  Springy
//
//  Created by Uriana on 13-11-3.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//

#include "ObstaclePlane.h"

ObstaclePlane::ObstaclePlane(Vector3D thenormal, Point3D thepoint)
{
    normal = thenormal;
    point = thepoint;
    collided = false;
}

bool ObstaclePlane::CollisionDetection(Particle p, Particle &p_after)
{

    Vector3D s_vector_current(0,0,0);
    Vector3D s_vector_next(0,0,0);
    double DotProduct_current = 0;
    double DotProduct_next = 0;
    
    s_vector_current = p.getPosition() - point;
    s_vector_next = p_after.getPosition() - point;
    DotProduct_current = Dot(s_vector_current, normal);
    DotProduct_next = Dot(s_vector_next, normal);
    //printf("%f ",DotProduct_current);
    //printf("DotProduct_next %f\n",DotProduct_next);

    float d = 0;
    if (( DotProduct_current >= 0 && DotProduct_next >= 0) || (DotProduct_current <= 0 && DotProduct_next <= 0))
        d = 0;
    else
        d = fabsf(DotProduct_next);
    // distance of xi+1 to the plane

    if (d != 0) {

        //compute new velocity and position
        //CollisionResponse( d, p_after);
        return true;
    }
    return false;
}

void ObstaclePlane::CollisionResponse(float d, Particle &p_after)
{
    Vector3D vn;
    float vn_value = 0;
    Point3D p = p_after.getPosition();
    Vector3D v = p_after.getVelocity();
    p_after.setPosition( p + (1+e)*d*normal);
    vn_value = Dot(v, normal);
    vn = fabsf(vn_value) * normal;
    p_after.setVelocity((1+e) * vn + v);

}
bool ObstaclePlane::initial(RigidBodyObject b)
{
    bool flag;
    for (int n = 0; n < b.number; n++) {

    Particle *p = b.p_init[n];
    Vector3D s_vector_current(0,0,0);
    double DotProduct_current = 0;
    
    s_vector_current = p->getPosition() - point;
    DotProduct_current = Dot(s_vector_current, normal);

        if (DotProduct_current >= 0) {
            flag = true;
        }else
            flag = false;
    }
    return flag;

}

bool ObstaclePlane::CollisionDetection(RigidBodyObject &b)
{
    for (int n = 0; n < b.number; n++) {
        Vector3D s_vector_current(0,0,0);
        double DotProduct_current = 0;
        s_vector_current = b.p[n]->getPosition() - point;
        DotProduct_current = Dot(s_vector_current, normal);
        //printf("%f\t",DotProduct_current);
        if(DotProduct_current <= 0) {
//        Particle *p = b.p[n];
//        Particle *p_next = b_after.p[n];
//        printf("%d %f\t",n,p->getPosition().y);
//        printf("%d %f\n",n,p_next->getPosition().y);
//        if (CollisionDetection(*p, *p_next)) {
        
        
            //collision detected
            //compute v-
            std::vector<Vector3D > ra0;
            ra0 = b.getr();
            NTmatrix r(3,1);
            double r_value[3] = {ra0[n].x, ra0[n].y, ra0[n].z};
            r.assign(r_value);
            Vector3D ra;
            NTmatrix R_mat = b.getR();

            NTmatrix ra_value = R_mat * r;
            ra.Set(ra_value.vect[0], ra_value.vect[1], ra_value.vect[2]);
            Vector3D v_before = Dot(normal,(b.p[n]->getVelocity() + Cross(b.getOmega(),ra))) * normal;
            
            //compute j
            Vector3D j;
            NTmatrix tmp_mat(3,1);
            Vector3D tmp_v;
            tmp_v = Cross((Cross(ra,normal)),ra);
            double tmp_value[3] = {tmp_v.x,tmp_v.y,tmp_v.z};
            tmp_mat.assign(tmp_value);
            NTmatrix inv = b.getIinv();
            double Iinv_value[9] = {inv.vect[0],inv.vect[1],inv.vect[2],inv.vect[3],inv.vect[4],inv.vect[5],inv.vect[6],inv.vect[7],inv.vect[8]};
            NTmatrix tmp_inv(3,3);
            tmp_inv.assign( Iinv_value);
            
            tmp_mat = tmp_inv *tmp_mat;
            tmp_v.Set(tmp_mat.vect[0], tmp_mat.vect[1], tmp_mat.vect[2]);

            j = (-(1+e)*v_before)/(1/b.getMass() + Dot(normal, tmp_v));
            Vector3D impulse;
            impulse = j;
            printf("impulse %f %f %f\n",impulse.x,impulse.y,impulse.z);
            //b.getR().printMat();
            b.RefreshState(b.getV(), b.getOmega(), b.getX(),R_mat);

            b.setP(b.getP() - impulse);
            b.setL(Cross(ra0[n], b.getL()) - impulse);
//            b.setV(b.getP()/b.getMass());
//            double tmp[3] = {b.getL().x,b.getL().y,b.getL().z};
//            NTmatrix temp(3,1);
//            temp.assign(tmp);
//            
//            temp = tmp_inv * temp;
//            tmp_v.Set(temp.vect[0], temp.vect[1], temp.vect[2]);
//            b.setOmega(tmp_v);
//            b.RefreshState(b.getV(), b.getOmega(), b.getX(),R_mat);

            return true;
        }else
            return false;
        
    }
    return false;
}