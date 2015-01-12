//
//  main.cpp
//  rigidBody
//
//  Created by Uriana on 13-11-19.
//  Copyright (c) 2013年 Uriana. All rights reserved.
//


#include <iostream>
#include <time.h>
#include <vector>
#include <GLUI/glui.h>

#include "Vector3D.h"
#include "EulerIntegration.h"
#include "RK4Integration.h"
#include "RigidBodyObject.h"

#define WIDTH 800
#define HEIGHT 600

#define SIZE 2
#define NUM SIZE*SIZE*SIZE
#define STATE_SIZE 18
#define NBODIES 2

GLfloat posx = -1.0;    //camera position
GLfloat posy = 1.0;    //camera position
double frames_per_second = 30;   // display rate
double size = 2;        //the size of the box
double t = 0;
double h = 0.01;

Integration * integ;
EulerIntegration ei(0.1);
RK4Integration rk(0.01);
bool isEuler = false;

RigidBodyObject body[NBODIES];
RigidBodyObject body_next[NBODIES];

std::vector<vector<Particle *> > s(NBODIES, vector<Particle *>(NUM));
std::vector<vector<Particle *> > s_next(NBODIES, vector<Particle *>(NUM));


World w;

using namespace std;

void initialize()
{
    float mass = 0.1;
    std::vector<Vector3D > angular_w(NBODIES);
    for (int nbody = 0; nbody < NBODIES; nbody++) {
        angular_w[nbody].Set(0.01*rand()/RAND_MAX, 0.01*rand()/RAND_MAX, 0.01*rand()/RAND_MAX);
    
    
    //for (int n = 0; n < N; n++) {
    for (int i = 0; i < SIZE; i++)
        for (int j = 0; j < SIZE; j++)
            for (int k = 0; k < SIZE; k++) {
                int n = i*SIZE*SIZE + j*SIZE + k;
                Particle *p = new Particle(Point3D((float)i/SIZE,(float)j/SIZE,(float)k/SIZE), Vector3D(0,0.1,0), mass, Point3D(0.5,0.5,0.5));
                Particle *p1 = new Particle(Point3D((float)i/SIZE - 0.5,(float)j/SIZE,(float)k/SIZE), Vector3D(0.0,0.001,0.0), mass, Point3D(0.5,0.5,0.2));
                //printf("%f %f %f\n",p->getPosition().x, p->getPosition().y, p->getPosition().z);
                s[0][n] = p;
                s[1][n] = p1;
                //Particle *init = new Particle();
                //s_next[nbody][n] = s[nbody][n];
                
            }

    
    for (int n = 0; n < NUM; n++) {
        body[nbody].p_init.empty();
        body[nbody].p_init.push_back(new Particle());
        body[nbody].p_init[n] = s[nbody][n];
        body[nbody].p.push_back(new Particle());
        body[nbody].p[n] = s[nbody][n];
        body_next[nbody].p_init.empty();
        body_next[nbody].p_init.push_back(new Particle());
        body_next[nbody].p_init[n] = s[nbody][n];
        body_next[nbody].p.push_back(new Particle());
        body_next[nbody].p[n] = s[nbody][n];

    }
    body[nbody].InitParticleToRigidBody();
    body[nbody].InitialState(s[nbody][0]->getVelocity(), angular_w[nbody]);
        body_next[nbody].InitParticleToRigidBody();
        body_next[nbody].InitialState(s[nbody][0]->getVelocity(), angular_w[nbody]);
        //body[nbody].RigidBodyToParticle();
        //body_next[nbody] = body[nbody];
        
    }
    
    w.addObstacle(new ObstaclePlane(Vector3D(0,1,0), Point3D(0,-1.9,0)));
    w.addObstacle(new ObstaclePlane(Vector3D(0,-1,0), Point3D(0,1.9,0)));
    w.addObstacle(new ObstaclePlane(Vector3D(1,0,0), Point3D(-1.9,0,0)));
    w.addObstacle(new ObstaclePlane(Vector3D(-1,0,0), Point3D(1.9,0,0)));
    w.addObstacle(new ObstaclePlane(Vector3D(0,0,1), Point3D(0,0,-1.9)));
    w.addObstacle(new ObstaclePlane(Vector3D(0,0,-1), Point3D(0,0,1.9)));
    

}

void ArrayToBodies(double x[]) {
    for(int i = 0; i < NBODIES; i++)
        body_next[i].ArrayToState(&x[i * STATE_SIZE]);
}

void BodiesToArray(double x[]) {
    for(int i = 0; i < NBODIES; i++)
        body_next[i].StateToArray(&x[i * STATE_SIZE]);
}

double * Dxdt ( double t, int n, double u[] )
{
    double *xdot;
    xdot = new double[n];
    /* put data in x[] into Body state */
    
    ArrayToBodies(u);
    for(int i = 0; i < NBODIES; i++)
    {
    body_next[i].ComputeForceAndTorque(t);
    body_next[i].DdtStateToArray(&xdot[i * STATE_SIZE]);
    }
    return xdot;
}


void rk4update()
{

    double *x0 = new double[STATE_SIZE * NBODIES];
    double *xFinal = new double[STATE_SIZE * NBODIES];
    
    BodiesToArray(x0);
    //body.StateToArray(xFinal);
    
    xFinal = rk4vec( t, STATE_SIZE * NBODIES, x0, h, Dxdt );
    //
    //  Shift the data to prepare for another step.
    //
    t = t + h;
    for ( int i = 0; i < STATE_SIZE * NBODIES; i++ )
    {
        x0[i] = xFinal[i];
    }
    delete [] xFinal;
    
    ArrayToBodies(x0);
    for(int i = 0; i < NBODIES; i++){
        body_next[i].RigidBodyToParticle();
        w.checkCollision(body_next[i]);
    //printf("%f\t",body[0].p[0]->getPosition().y);
    for (int n = 0; n < NUM; n++) {
        s[i][n]->setPosition(body_next[i].p[n]->getPosition());
        //s[i] = s_next[i];
        //printf("%f\t",s[n]->getPosition().y);
    }
        //body[i] = body_next[i];
    }
    //*body = *body_next;
    
}

void idle()
{
    double start = t;
    clock_t start_time = clock();
    rk4update();

    double tau = 1.0 / frames_per_second;
    while (t - start < tau)
    {
        rk4update();
    }
    
    while (((double)(clock()) - start_time) / CLOCKS_PER_SEC < tau)
        ;
    glutPostRedisplay();
    
}

void display()
{
    //printf("refresh\n");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//clear buffer
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
 	gluLookAt(posx, posy, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//isometric view
    
    //BOX
    glPushMatrix();
    glScaled((size)*2, (size)*2, (size)*2);
    glColor3f(0.1, 1.0, 0);
    glutWireCube(1.0);

    //BOX BOTTOM PLANE
    glColor3f(0.1, 0.2, 0.1);
    glBegin(GL_POLYGON);
    glVertex3f(  0.5, -0.5, -0.5 );
    glVertex3f(  0.5, -0.5,  0.5 );
    glVertex3f( -0.5, -0.5,  0.5 );
    glVertex3f( -0.5, -0.5, -0.5 );
    glEnd();
    glPopMatrix();
    
    for (int nbody = 0; nbody < NBODIES; nbody++) {
    //VERTICES
    glPointSize(5);
    glBegin(GL_POINTS);
    
    for (int n = 0; n < NUM; n++) {
        glColor3f(s[nbody][n]->getColor().x,s[nbody][n]->getColor().y,s[nbody][n]->getColor().z);
        //printf("%f \t",s[n]->getPosition().y);
        glVertex3f(s[nbody][n]->getPosition().x, s[nbody][n]->getPosition().y, s[nbody][n]->getPosition().z);
    }
    
    glEnd();
    
    for (int n = 0; n < NUM; n++) {
        //printf("%f \t",s[n]->getPosition().x);
        for (int m = 0; m < NUM; m++) {
            //printf("%f\t",Magnitude(s[m]->getPosition() - s[n]->getPosition()));
            float tolerance = 0.1;
            if ((Magnitude(s[nbody][m]->getPosition() - s[nbody][n]->getPosition()) > (float)(SIZE-1)/SIZE - tolerance) && (Magnitude(s[nbody][m]->getPosition() - s[nbody][n]->getPosition()) < (float)(SIZE-1)/SIZE + tolerance)) {
                glBegin(GL_LINE_LOOP);
                glColor3f(s[nbody][n]->getColor().x,s[nbody][n]->getColor().y,s[nbody][n]->getColor().z);
                glVertex3f(s[nbody][n]->getPosition().x, s[nbody][n]->getPosition().y, s[nbody][n]->getPosition().z);
                glVertex3f(s[nbody][m]->getPosition().x, s[nbody][m]->getPosition().y, s[nbody][m]->getPosition().z);
                glEnd();
            }
        }
    }
    }
    //glutPostRedisplay();

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
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glClearColor(0.1, 0.1, 0.1, 1.0);
    glEnable(GL_DEPTH_TEST);       // so the renderer considers depth
}


// The usual main function.
int main(int argc, char** argv) {
    initialize();
    if (isEuler) {
        integ = &ei;
    }else {
        integ = &rk;
    }
    
    
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Rigid Body");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mymouse);
    glutSpecialFunc(keyboard);
    glutIdleFunc(idle);
    init();
    
    glutMainLoop();
}

