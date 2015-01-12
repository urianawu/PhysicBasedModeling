//
//  main.c
//  Flocking
//
//  Created by Uriana on 10/13/13.
//  Copyright (c) 2013 Uriana. All rights reserved.
//

#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "Vector3D.h"

#ifdef __APPLE__
#  include <GLUI/glui.h>
#else
#  include <GL/glui.h>
#endif

#define WIDTH 800
#define HEIGHT 800

#define N 200     //number of particles
#define PI 3.141592654

//used colors
//GLfloat black[] = { 0.0, 0.0, 0.0, 1.0 };
//GLfloat yellow[] = { 1.0, 1.0, 0.0, 1.0 };
//GLfloat pink[] = { 0.9, 0.4, 0.4, 1.0 };
//GLfloat cyan[] = { 0.0, 1.0, 1.0, 1.0 };
//GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
//GLfloat direction[] = { 1.0, 1.0, 1.0, 0.0 };



//structures
typedef struct {
    Point3D position[N];
    Vector3D velocity[N];
    Vector3D color[N];
    int lifespan[N];
    float mass;
}State;

typedef struct{
    Point3D position;
    Vector3D velocity;
    Vector3D color;
    float mass;
}Lead;

typedef struct {
    Vector3D value;
    Vector3D accel;
}Force;

typedef struct {
    Vector3D normal;
    Point3D point;
    
}Plane;

typedef struct {
    float radius;
    Point3D center;
    
}Sphere;


//public variables
State s_initial, s_current, s_next;
Lead lead_current, lead_next;
Plane p_left, p_right, p_bot, p_top, p_back, p_front, p_polygen;
Sphere s_inner, s_outer;
Vector3D acceleration[N];
Vector3D lead_accel;

Vector3D tranPos[N];     //translating position
double t;             // simulation time

//public defined parameters
double size = 2;        //the size of the ball
Point3D vertices[3];   //vertices of polygen
float innerPos[3];
double v_max = 2;     // maximum initial speed of ball
double x_max = 1;       // maximum initial position of ball
double c_max = 0.5;

float g = 1;        //gravity
float ad = 0.01;      //air resistance factor
float e = 0.5;      //elasticity factor
float miu = 0.1;    //friction factor

float ka = 0.005;     //flocking avoidance factor
float kv = 0.01;     //flocking velocity matching factor
float kc = 0.02;     //flocking centering factor
float kl = 2;

double h = 0.01;     // time step for Euler Integration
double frames_per_second = 30;   // display rate

float tolerance = 0.01;    // position error tolerance
float vtolerance = 0.15;     // velocity tolerance

GLfloat posx = -1.0;    //camera position
GLfloat posy = 1.0;    //camera position
GLUquadricObj *quadratic;
GLfloat mousex;
GLfloat mousey;

double gaussrand()
{
	static double U, V;
	static int phase = 0;
	double Z;
    
	if(phase == 0) {
		U = (rand() + 1.) / (RAND_MAX + 2.);
		V = rand() / (RAND_MAX + 1.);
		Z = sqrt(-2 * log(U)) * sin(2 * PI * V);
	} else
		Z = sqrt(-2 * log(U)) * cos(2 * PI * V);
    
	phase = 1 - phase;
    
	return Z;
}


void box()
{
    glPushMatrix();
    glScaled((size)*2, (size)*2, (size)*2);
    //glRotatef(5.0, 0.0, 1.0, 0.0);
    
    //    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
    //    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    //    glMaterialf(GL_FRONT, GL_SHININESS, 100);
    //    glDepthMask(GL_TRUE);
    glColor3f(0, 0.3, 0.3);
    glutWireSphere(1.0, 16, 10);
    glPopMatrix();
    
    glPushMatrix();
    glColor3f(0.5, 0.5 ,0.5 );
    //
    glTranslatef(s_inner.center.x,s_inner.center.y,s_inner.center.z);
    //    glRotated(90, 1, 0, 0);
    //    gluCylinder(quadratic,1.0f,0.0f,2.0f,3,3);
    
    glutSolidSphere(s_inner.radius, 24, 24);
    glPopMatrix();
}

void initPlaneNormal()
{
    
    Vector3D v1, v2;
    Vector3D polygenNormal;
    float normalValue = 0;
    v1 = vertices[1] - vertices[0];
    v2 = vertices[2] - vertices[1];
    
    
    polygenNormal = Cross(v1, v2);
    normalValue = Magnitude(polygenNormal);
    
    p_polygen.normal = -polygenNormal/normalValue;
    p_polygen.point = vertices[1];
    
    //printf("polygen point %f %f %f\n",p_polygen.point[0],p_polygen.point[1],p_polygen.point[2]);
    
    //back plane
    p_back.normal.Set(0,0,1);
    p_back.point.Set(0,0,-2);
    //front plane
    p_front.normal.Set(0,0,-1);
    p_front.point.Set(0,0,2);
    
    //left plane
    p_left.normal.Set(1,0,0);
    p_left.point.Set(-2, 0, 0);
    
    //right plane
    p_right.normal.Set(-1,0,0);
    p_right.point.Set(2,0,0);
    
    //bottom plane
    p_bot.normal.Set(0,1,0);
    p_bot.point.Set(0,-2,0);
    
    //top plane
    p_top.normal.Set(0,-1,0);
    p_top.point.Set(0,2,0);
    
    //inner sphere
    s_inner.radius = 0.2;
    s_inner.center.Set(innerPos[0], innerPos[1], innerPos[2]);
    
    //outer sphere
    s_outer.radius = size*2;
    s_outer.center.Set(0, 0, 0);
}

void initialization()
{
    
    for (int n =0; n<N; n++) {
        Vector3D vn(0.5,0.5,0.5);
        Vector3D v(0,0,0);
        Vector3D random(0.2*gaussrand(),0.2*gaussrand(),0.2*gaussrand());
        //v = vn + random;
        v = vn;
        float a, b, c;
        do {
        a = fabsf(gaussrand());
        b = fabsf(gaussrand());
        c = 1-a-b;
        }while (c < 0);
        
        vertices[0].Set(0.5, -0.5, 0.5);
        vertices[1].Set(0.3, 0.5, 0.7);
        vertices[2].Set(0, -0.5, 0.3);

        s_current.position[n] = a*vertices[0]+b*vertices[1]+c* vertices[2];
        s_current.velocity[n] = v;
        s_current.color[n].Set(1,1,1);
        s_current.mass = 0.1;
    }
    
    lead_current.position.Set(0.5,0.5,0.5);
    lead_current.velocity.Set(0.5, 0.5, 0.5);
    lead_current.color.Set(0.5, 0.5, 0.5);
    lead_current.mass = 0.1;
    
    innerPos[0] = 0;
    innerPos[1] = 0;
    innerPos[2] = 0;
    
}

void CollisionResponse (float d, Vector3D& p,Vector3D& v, Vector3D &normal, Point3D &point)
{
    //printf("collision response function! %f,%f,%f\n",v[0],v[1],v[2]);
    //printf("RESPONSE distance %f\n",d);
    Vector3D vn;
    float vn_value = 0;
    
    
//    if (normal == p_polygen.normal && point == p_polygen.point)
//    {
//        Point3D pc, p0c, p1c, p2c;
//        Vector3D a0, a1, a2;
//        float sign_a0 = 0, sign_a1 = 0, sign_a2 = 0;
//        
//        pc = p + d*p_polygen.normal;
//        p0c = vertices[0] - pc;
//        p1c = vertices[1] - pc;
//        p2c = vertices[2] - pc;
//        
//        
//        //printf("pc %f %f %f",pc[0],pc[1],pc[2]);
//        a0.x = p1c.y*p2c.z - p1c.z*p2c.y;
//        a0.y = -p1c.x*p2c.y + p1c.z*p2c.x;
//        a0.z = p1c.x*p2c.y - p1c.y*p2c.x;
//        
//        a1.x = p2c.y*p0c.z - p2c.z*p0c.y;
//        a1.y = -p2c.x*p0c.z + p2c.z*p0c.x;
//        a1.z = p2c.x*p0c.y - p2c.y*p0c.x;
//        
//        a2.x = p0c.y*p1c.z - p0c.z*p1c.y;
//        a2.y = -p0c.x*p1c.z + p0c.z*p1c.x;
//        a2.z = p0c.x*p1c.y - p0c.y*p1c.x;
//        
//        
//        sign_a0 = Dot(a0, p_polygen.normal);
//        sign_a1 = Dot(a1, p_polygen.normal);
//        sign_a2 = Dot(a2, p_polygen.normal);
//        
//        
//        //printf("sign %f %f %f\n",sign_a0, sign_a1, sign_a2);
//        if ((sign_a0 >= 0 && sign_a1 >= 0 && sign_a2 >= 0)||(sign_a0 <= 0 && sign_a1 <= 0 && sign_a2 <= 0)) {
//            p = p + (1+e)*d*p_polygen.normal;
//            vn_value = Dot(v, p_polygen.normal);
//            vn = fabsf(vn_value) * p_polygen.normal;
//            v = (1+e) * vn + v;
//        }
//        
//        
//    }else{
    
        p = p + (1+e)*d*normal;
        vn_value = Dot(v, normal);
        vn = fabsf(vn_value) * normal;
        v = (1+e) * vn + v;
    //}
}

void PlaneCollision(int n, Plane p)
{
    //printf("plan collision function!");
    Vector3D s_vector_current(0,0,0);
    Vector3D s_vector_next(0,0,0);
    double DotProduct_current = 0 ;
    double DotProduct_next = 0;
    
    s_vector_current = s_current.position[n] - p.point;
    s_vector_next = s_next.position[n] - p.point;
    DotProduct_current = Dot(s_vector_current, p.normal);
    DotProduct_next = Dot(s_vector_next, p.normal);
    //printf("vector next in plane collision %f \n",s_next.position[n][i]);
    
    //printf("dotproduct in plane collision %f \n",DotProduct_next);
    
    // Determine if it didnt collide
    float d = 0;
    if (( DotProduct_current >= 0 && DotProduct_next >= 0) || (DotProduct_current <= 0 && DotProduct_next <= 0))
        d = 0;
    else
        d = fabsf(DotProduct_next);
    // distance of xi+1 to the plane
    
    //printf("distance in plane collision %f \n",d);
    if (d != 0) {
        //compute new velocity and position
        CollisionResponse( d, s_next.position[n], s_next.velocity[n],p.normal,p.point );
        
    }
    
}

void SphereCollision(int n, Sphere s, int outward)
{
    Vector3D s_vector_current(0,0,0);
    Vector3D s_vector_next(0,0,0);
    Point3D point;
    Vector3D normal;
    point= s.radius * ((s_current.position[n]-s.center).Normalize());
    if (outward == 1)
        normal = (s_current.position[n]-s.center).Normalize();
    else
        normal = -(s_current.position[n]-s.center).Normalize();
    
    float dist_current;
    float dist_next;
    s_vector_current = (s_current.position[n] - point);
    s_vector_next = (s_next.position[n] - point);
    dist_current = Dot(s_vector_current, normal);
    dist_next = Dot(s_vector_next, normal);
    float d = 0;
    if (( dist_current >= 0 && dist_next >= 0) || (dist_current <= 0 && dist_next <= 0))
        d = 0;
    else
        d = fabsf(dist_next);
    
    if (d != 0) {
        CollisionResponse( d, s_next.position[n], s_next.velocity[n], normal, point);
        s_next.color[n].Set(0.5, 0.5, 0.5);

    }
}

void CollisionDetection(int n)
{
    
    //printf("collision detect function!");
    
    //PlaneCollision(n,p_polygen);
    //PlaneCollision(n,p_front);
    //PlaneCollision(n,p_back);
    //PlaneCollision(n,p_right);
    //PlaneCollision(n,p_left);
    //PlaneCollision(n,p_top);
    //PlaneCollision(n,p_bot);
    SphereCollision(n, s_inner,1);
    SphereCollision(n, s_outer,0);
    
}

Vector3D steering(int n, Sphere s)
{
    Vector3D xc;
    Vector3D vi;
    Vector3D vt;
    Vector3D accel(0,0,0);
    float distance;
    float T;
    
    xc = s.center - s_current.position[n];
    distance = fabsf(Magnitude(xc) - s.radius);
    vi = Dot(s_current.velocity[n], xc.Normalize()) * xc.Normalize();
    T = distance/Magnitude(vi);
    vt = s_current.velocity[n] - vi;
    if (T * Magnitude(vt) < s.radius) {
        accel = 2*(s.radius - T*Magnitude(vt))/(T*T) * vt.Normalize();
    }
    return accel;
}

Vector3D computeAcceleration(int n)
{
    Force gravity;
    Force airResist;
    Force wind;
    Force fAvoidance;
    Force fVmatch;
    Force fCentering;
    Vector3D acceleration( 0, 0, 0 );
    
    //gravity towards center
    Point3D center;
    Vector3D ag;
    Vector3D u;
    float r;
    center.Set(s_inner.center.x, s_inner.center.y, s_inner.center.z);
    if (n != -1) {
        u = (s_current.position[n] - center).Normalize();
        //printf("%f\t",s_current.position[n].x);
        //printf("%f %f %f\n",u.x,u.y,u.z);
        
        r = SquaredMag((s_current.position[n] - center));
        ag = - g* (s_current.mass/(r))*u;
        
        gravity.accel = ag;
        
        wind.value.Set(0, 0, 0);
        airResist.value = - ad * s_current.velocity[n];
        acceleration = gravity.accel + (airResist.value+wind.value) / s_current.mass;
        
        //flocking
        fAvoidance.accel.Set(0, 0, 0);
        fVmatch.accel.Set(0, 0, 0);
        fCentering.accel.Set(0, 0, 0);
        
        
        for (int i = 0; i<N; i++) {
            Vector3D xij;
            float dij;
            Vector3D uij;
            xij = s_current.position[i] - s_current.position[n];
            dij = Magnitude(xij);
            //printf("%f\t",dij);
            
            uij = xij/dij;
            //printf("uij %f\n",uij.x);
            
            if ( (dij != 0) && (dij<0.6) ) {
                fAvoidance.accel += (- ka * (1/(dij)) *uij);
                
                fVmatch.accel += kv *exp(-dij)* (s_current.velocity[i] - s_current.velocity[n]);
                fCentering.accel += kc *exp(-dij)* xij;
            }
        }
        //lead
        fCentering.accel += kl * (lead_current.position - s_current.position[n]);
        //STEERING
        acceleration += steering(n, s_inner);
        acceleration += fAvoidance.accel + fVmatch.accel + fCentering.accel;
    }else{
        u = (lead_current.position - center).Normalize();
        
        r = SquaredMag((lead_current.position - center));
        ag = - g* (lead_current.mass/(r))*u;
        
        gravity.accel = ag;
        
        wind.value.Set(0, 0, 0);
        airResist.value = - ad * lead_current.velocity;
        acceleration = gravity.accel + (airResist.value + wind.value) / lead_current.mass;

    }
    
    return acceleration;
    
    
    
}

int restingContact( Vector3D position, Vector3D velocity, Vector3D accel)
{
    
    
    float v = SquaredMag(velocity);
    
    //front
    if ((abs (position.z - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = accel *p_front.normal;
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //back
    if ((abs (- position.z - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = accel *p_back.normal;
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //right
    if ((abs (position.x - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = accel *p_right.normal;
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //left
    if ((abs (-position.x - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = accel *p_left.normal;
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //bot
    if ((abs (- position.y - size)) < tolerance) {
        //printf("checking bottom plane");
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = accel *p_bot.normal;
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //top
    if ((abs (position.y - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = accel *p_top.normal;
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    return 0;

}

void update()
{
    lead_accel = computeAcceleration(-1);
    //printf("here accel %f\t",lead_accel[0]);

    //lead_next.velocity = lead_current.velocity + lead_accel * h;
    //lead_next.position = lead_current.position + lead_current.velocity * h;
    lead_next.position.Set(size*4*(mousex-WIDTH/2)/WIDTH, size*4*(HEIGHT/2- mousey)/HEIGHT, 0);
    lead_next.color = lead_current.color;
    lead_next.mass = lead_current.mass;
    if (((int)round(t) % 6) < 3) {
        kc += 0.0001;     //flocking centering factor
    }else {
        kc -= 0.0001;
    }
    for (int n=0; n<N; n++) {
        
        //compute acceleration
        acceleration[n] = computeAcceleration(n);
        
        //integrate to get new state
        s_next.velocity[n] = s_current.velocity[n] + acceleration[n] * h;
        s_next.position[n] = s_current.position[n] + s_current.velocity[n] * h;
        if (((int)round(t) % 10) < 5) {
            s_next.color[n].y = s_current.color[n].y + Magnitude(s_current.velocity[n]) *h/10.0;
            s_next.color[n].x = s_current.color[n].x - Magnitude(s_current.velocity[n]) *h/5.0;
            s_next.color[n].z = s_current.color[n].z + Magnitude(s_current.velocity[n]) *h/10.0;
        }else {
            s_next.color[n].y = s_current.color[n].y - Magnitude(s_current.velocity[n])*h/10.0;
            s_next.color[n].x = s_current.color[n].x + Magnitude(s_current.velocity[n])*h/5.0;
            s_next.color[n].z = s_current.color[n].z - Magnitude(s_current.velocity[n])*h/10.0;
        }
        s_next.mass = s_current.mass;
        //printf("next position for %d is %f", n, s_next.position[n][0]);
        
        //check for collision
        CollisionDetection(n);
        
    }
    
    //update state
    lead_current = lead_next;
    s_current = s_next;
    t += h;
    for (int n=0; n<N; n++) {
        tranPos[n] = s_current.position[n];
        
        int outOfBox = 0;
        if (fabsf(s_current.position[n].x) > 2 || fabsf(s_current.position[n].y) > 2 || fabsf(s_current.position[n].z) > 2)
            outOfBox = 1;
//        int stopped = restingContact(s_current.position[n],s_current.velocity[n],acceleration[n]);
//        if (stopped == 1 || outOfBox == 1) {
//            initialization(n);
//        }
    }
   
    
}

void idle() {
    double start = t;
    clock_t start_time = clock();
    update();
    
    double tau = 1.0 / frames_per_second;
    while (t - start < tau)
        update();
    while (((double)(clock()) - start_time) / CLOCKS_PER_SEC < tau)
        ;
    glutPostRedisplay();
    
}


void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//clear buffer
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
 	gluLookAt(posx, posy, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//isometric view
    
    // BOX
    //glPushMatrix();
    //glDepthMask(GL_FALSE);
    box();
    
    //glPopMatrix();
    
    //lead
    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(lead_current.color.x, lead_current.color.y, lead_current.color.z);
    glVertex3f(lead_current.position.x, lead_current.position.y, lead_current.position.z);
    glEnd();
    
    // particles
    // Add particles to the scene.
    glPointSize(2);
    glBegin(GL_POINTS);
    for (int n=0; n<N; n++) {
        glColor3f(s_current.color[n].x,s_current.color[n].y,s_current.color[n].z);
        glVertex3f(tranPos[n].x, tranPos[n].y, tranPos[n].z);
        //printf("POSITION OF %d is %f/%f/%f",n,tranPos[n][0], tranPos[n][1], tranPos[n][2]);
    }
    glEnd();
    
    
    
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
void passive(int x1,int y1) {
    mousex=x1; mousey=y1;
    //printf("x %f y %f\n",mousex,mousey);
}

void init() {
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.1, 0.1, 0.1, 1.0);
    quadratic = gluNewQuadric();
    
    //    glLightfv(GL_LIGHT0, GL_AMBIENT, black);
    //    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    //    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    //    glLightfv(GL_LIGHT0, GL_POSITION, direction);
    //    glEnable(GL_NORMALIZE);
    //
    //    glMatrixMode(GL_MODELVIEW);
    //    glLoadIdentity();
    //
    //    glEnable(GL_LIGHTING);                // so the renderer considers light
    //    glEnable(GL_LIGHT0);                  // turn LIGHT0 on
    glEnable(GL_DEPTH_TEST);              // so the renderer considers depth
    
    initPlaneNormal();
    //s_current = s_initial;
    
}




// The usual main function.
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    
    //set initial state
    srand ((unsigned int)time(0));
    initialization();

    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Particle Systems");
    glShadeModel(GL_FLAT);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mymouse);
    glutSpecialFunc(keyboard);
    glutIdleFunc(idle);
    glutPassiveMotionFunc(passive);

    init();
    
    glutMainLoop();
}
