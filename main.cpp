//
//  main.c
//  ParticleSystem
//
//  Created by Uriana on 9/28/13.
//  Copyright (c) 2013 Uriana. All rights reserved.
//

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifdef __APPLE__
#  include <GLUI/glui.h>
#else
#  include <GL/glui.h>
#endif

#define WIDTH 800
#define HEIGHT 600

#define N 200     //number of particles
#define PI 3.141592654

//used colors
GLfloat black[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat yellow[] = { 1.0, 1.0, 0.0, 1.0 };
GLfloat pink[] = { 0.9, 0.4, 0.4, 1.0 };
GLfloat cyan[] = { 0.0, 1.0, 1.0, 1.0 };
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat direction[] = { 1.0, 1.0, 1.0, 0.0 };



//structures
typedef struct {
    float position[N][3];
    float velocity[N][3];
    float color[N][3];
    int lifespan[N];
}State;

typedef struct {
    float value[3];
}Force;

typedef struct {
    float normal[3];
    float point[3];
    
}Plane;

//public variables
State s_initial, s_current, s_next;
Plane p_left, p_right, p_bot, p_top, p_back, p_front, p_polygen;
float acceleration[N][3];

float tranPos[N][3];     //translating position
double timestep;             // simulation time
int planeIndicator;     //which plane it collides

//public defined parameters
double size = 2;        //the size of the box
float vertices[3][3];   //vertices of polygen
double v_max = 1;     // maximum initial speed of ball
double x_max = 1;       // maximum initial position of ball
double c_max = 0.5;

float mass = 1;     //mass of ball
float g = 1;        //gravity
float ad = 0.1;      //air resistance factor
float e = 0.5;      //elasticity factor
float miu = 0.5;    //friction factor


double h = 0.01;     // time step for Euler Integration
double frames_per_second = 30;   // display rate

float tolerance = 0.01;    // position error tolerance
float vtolerance = 0.15;     // velocity tolerance

GLfloat posx = -1.0;    //camera position
GLfloat posy = 1.0;    //camera position
GLUquadricObj *quadratic;

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

float * crossProduct(float u[], float v[])
{
    static float cp[3];
    cp[0] = u[1]*v[2] - u[2]*v[1];
    cp[1] = -u[0]*v[2] + u[2]*v[0];
    cp[2] = u[0]*v[1] - u[1]*v[0];
    return cp;
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
    glColor3f(1.0, 1.0, 0);
    glutWireCube(1.0);
    glPopMatrix();
    
//    glPushMatrix();
//    glColor3f(0.9, 0.5 ,0.5 );
//    
//    glTranslatef(0.0f,1.5f,0.0f);
//    glRotated(90, 1, 0, 0);
//    gluCylinder(quadratic,1.0f,0.0f,2.0f,3,3);
//    glPopMatrix();
    
    glPushMatrix();
    // Yellow side - BACK
//    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, white);
//    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
//    glMaterialf(GL_FRONT, GL_SHININESS, 80);
//
    glColor3f(0.9, 0.5 ,0.5 );
    glBegin(GL_POLYGON);
    glVertex3f(  vertices[0][0],vertices[0][1],vertices[0][2] );
    glVertex3f(  vertices[1][0],vertices[1][1],vertices[1][2] );
    glVertex3f(  vertices[2][0],vertices[2][1],vertices[2][2] );
    glEnd();
    
    
    
    glPopMatrix();
    
}
void initPlaneNormal()
{
    
    float v1[3], v2[3];
    float *polygenNormal;
    float normalValue = 0;
    for (int i=0; i<3; i++) {
        v1[i] = vertices[1][i] - vertices[0][i];
        v2[i] = vertices[2][i] - vertices[1][i];
    }
    
    polygenNormal = crossProduct(v1, v2);
    normalValue =
    sqrt(powf(polygenNormal[0], 2) +powf(polygenNormal[1], 2) +powf(polygenNormal[2], 2));
    
    p_polygen.normal[0] = polygenNormal[0]/normalValue;
    p_polygen.normal[1] = polygenNormal[1]/normalValue;
    p_polygen.normal[2] = polygenNormal[2]/normalValue;
    
    p_polygen.point[0] = vertices[1][0];
    p_polygen.point[1] = vertices[1][1];
    p_polygen.point[2] = vertices[1][2];
    printf("polygen point %f %f %f\n",p_polygen.point[0],p_polygen.point[1],p_polygen.point[2]);

    //back plane
    p_back.normal[0] = 0;
    p_back.normal[1] = 0;
    p_back.normal[2] = 1;
    
    p_back.point[0] = 0;
    p_back.point[1] = 0;
    p_back.point[2] = -2;
    //front plane
    p_front.normal[0] = 0;
    p_front.normal[1] = 0;
    p_front.normal[2] = -1;
    
    p_front.point[0] = 0;
    p_front.point[1] = 0;
    p_front.point[2] = 2;
    
    //left plane
    p_left.normal[0] = 1;
    p_left.normal[1] = 0;
    p_left.normal[2] = 0;
    
    p_left.point[0] = -2;
    p_left.point[1] = 0;
    p_left.point[2] = 0;
    
    
    //right plane
    p_right.normal[0] = -1;
    p_right.normal[1] = 0;
    p_right.normal[2] = 0;
    
    p_right.point[0] = 2;
    p_right.point[1] = 0;
    p_right.point[2] = 0;
    
    //bottom plane
    p_bot.normal[0] = 0;
    p_bot.normal[1] = 1;
    p_bot.normal[2] = 0;
    
    p_bot.point[0] = 0;
    p_bot.point[1] = -2;
    p_bot.point[2] = 0;
    
    //top plane
    p_top.normal[0] = 0;
    p_top.normal[1] = -1;
    p_top.normal[2] = 0;
    
    p_top.point[0] = 0;
    p_top.point[1] = 2;
    p_top.point[2] = 0;

}

void initialization(int n)
{
    
    
    s_current.position[n][0] = 1.9;
    s_current.position[n][1] = 1.9;
    s_current.position[n][2] = -2;

    
    s_current.velocity[n][0] = v_max * gaussrand();
    s_current.velocity[n][1] = v_max * gaussrand();
    s_current.velocity[n][2] = v_max * gaussrand();
    
    s_current.color[n][0] = c_max * ((double)rand() / (double)RAND_MAX *2 - 1)+0.5;
    s_current.color[n][1] = c_max ;
    s_current.color[n][2] = c_max * ((double)rand() / (double)RAND_MAX *2 - 1)+0.5;
    
//    printf(" y %f",s_current.position[n][1] );
//    printf(" z %f",s_current.position[n][2] );
//    printf(" vx %f",s_current.velocity[10][0] );
//    printf(" vy %f",s_current.velocity[10][1] );
//    printf(" vz %f",s_current.velocity[10][2] );
    
}

float PlaneCollision(int n, Plane p)
{
    //printf("plan collision function!");
    float s_vector_current[3] = {0,0,0};
    float s_vector_next[3]={0,0,0};
    double DotProduct_current = 0 ;
    double DotProduct_next = 0;
    
    for (int i=0; i<3; i++) {
        s_vector_current[i] = s_current.position[n][i] - p.point[i];
        s_vector_next[i] = s_next.position[n][i] - p.point[i];
        DotProduct_current += s_vector_current[i] * p.normal[i];
        DotProduct_next += s_vector_next[i] * p.normal[i];
        //printf("vector next in plane collision %f \n",s_next.position[n][i]);

    }
    //printf("dotproduct in plane collision %f \n",DotProduct_next);

    // Determine if it didnt collide
    float d = 0;
    if (( DotProduct_current >= 0 && DotProduct_next >= 0) || (DotProduct_current <= 0 && DotProduct_next <= 0))
        d = 0;
    else
        d = fabsf(DotProduct_next);
    // distance of xi+1 to the plane
    
    //printf("distance in plane collision %f \n",d);
    return d;
    
}

float CollisionDetection(int n)
{
    
    //printf("collision detect function!");

    float d = 0;
    if (PlaneCollision(n,p_polygen) != 0) {
        planeIndicator = 0;
        d = PlaneCollision(n,p_polygen);
        return d;
    }

    if (PlaneCollision(n,p_front) != 0) {
        planeIndicator = 1;
        d = PlaneCollision(n,p_front);
        return d;
    }
    if (PlaneCollision(n,p_back) != 0) {
        planeIndicator = 2;
        d = PlaneCollision(n, p_back);
        return d;
    }
    if (PlaneCollision(n,p_right) != 0) {
        planeIndicator = 3;
        d = PlaneCollision(n,p_right);
        return d;
    }
    if (PlaneCollision(n,p_left) != 0) {
        planeIndicator = 4;
        d = PlaneCollision(n,p_left);
        return d;
    }
    if (PlaneCollision(n,p_bot) != 0) {
        planeIndicator = 5;
        d = PlaneCollision(n,p_bot);
        return d;
    }
    if (PlaneCollision(n,p_top) != 0) {
        planeIndicator = 6;
        d = PlaneCollision(n,p_top);
        return d;
    }
    
    return 0;
    
}

float ** CollisionResponse (float d, float p[],float v[])
{
    //printf("collision response function! %f,%f,%f\n",v[0],v[1],v[2]);
    //printf("RESPONSE distance %f\n",d);
    float vn[3];
    static float v_next[3],p_next[3];
    float vn_value = 0;
    
    //initialize array
    float **next = 0;
    next = new float*[2];
    for (int h = 0; h < 2; h++) {
        next[h] = new float[3];
    }
    
    for (int i = 0; i < 3; i++) {
        switch (planeIndicator) {
            case 0:
            {
                float pc[3], p0c[3], p1c[3], p2c[3];
                float a0[3], a1[3], a2[3];
                float sign_a0 = 0, sign_a1 = 0, sign_a2 = 0;
                
                    pc[i] = p[i] + d*p_polygen.normal[i];
                    p0c[i] = vertices[0][i] - pc[i];
                    p1c[i] = vertices[1][i] - pc[i];
                    p2c[i] = vertices[2][i] - pc[i];

                
                //printf("pc %f %f %f",pc[0],pc[1],pc[2]);
                a0[0] = p1c[1]*p2c[2] - p1c[2]*p2c[1];
                a0[1] = -p1c[0]*p2c[2] + p1c[2]*p2c[0];
                a0[2] = p1c[0]*p2c[1] - p1c[1]*p2c[0];
                
                a1[0] = p2c[1]*p0c[2] - p2c[2]*p0c[1];
                a1[1] = -p2c[0]*p0c[2] + p2c[2]*p0c[0];
                a1[2] = p2c[0]*p0c[1] - p2c[1]*p0c[0];
                
                a2[0] = p0c[1]*p1c[2] - p0c[2]*p1c[1];
                a2[1] = -p0c[0]*p1c[2] + p0c[2]*p1c[0];
                a2[2] = p0c[0]*p1c[1] - p0c[1]*p1c[0];
//                a0 = crossProduct(p1c, p2c);
//                a1 = crossProduct(p2c, p0c);
//                a2 = crossProduct(p0c, p1c);
                
                
                    sign_a0 += a0[i] * p_polygen.normal[i];
                    sign_a1 += a1[i] * p_polygen.normal[i];
                    sign_a2 += a2[i] * p_polygen.normal[i];

                
                //printf("sign %f %f %f\n",sign_a0, sign_a1, sign_a2);
                if ((sign_a0 >= 0 && sign_a1 >= 0 && sign_a2 >= 0)||(sign_a0 <= 0 && sign_a1 <= 0 && sign_a2 <= 0)) {
                    p_next[i] = p[i] + (1+e)*d*p_polygen.normal[i];
                    vn_value += v[i] * p_polygen.normal[i];
                    vn[i] = fabsf(vn_value) * p_polygen.normal[i];
                    v_next[i] = (1+e) * vn[i] + v[i];
                }else{
                    //printf("OUTSIDE POLYGEN ");
                    p_next[i] = p[i];
                    v_next[i] = v[i];
                }
                break;
                
            }
            case 1:
                p_next[i] = p[i] + (1+e)*d*p_front.normal[i];
                vn_value += v[i] * p_front.normal[i];
                vn[i] = fabsf(vn_value) * p_front.normal[i];
                v_next[i] = (1+e) * vn[i] + v[i];

                break;
            case 2:
                p_next[i] = p[i] + (1+e)*d*p_back.normal[i];
                vn_value += v[i] * p_back.normal[i];
                vn[i] = fabsf(vn_value) * p_back.normal[i];
                v_next[i] = (1+e) * vn[i] + v[i];

                break;
            case 3:
                p_next[i] = p[i] + (1+e)*d*p_right.normal[i];
                vn_value += v[i] * p_right.normal[i];
                vn[i] = fabsf(vn_value) * p_right.normal[i];
                v_next[i] = (1+e) * vn[i] + v[i];

                break;
            case 4:
                p_next[i] = p[i] + (1+e)*d*p_left.normal[i];
                vn_value += v[i] * p_left.normal[i];
                vn[i] = fabsf(vn_value) * p_left.normal[i];
                v_next[i] = (1+e) * vn[i] + v[i];

                break;
            case 5:
                p_next[i] = p[i] + (1+e)*d*p_bot.normal[i];
                vn_value += v[i] * p_bot.normal[i];
                vn[i] = fabsf(vn_value) * p_bot.normal[i];
                v_next[i] = (1+e) * vn[i] + v[i];

                break;
            case 6:
                p_next[i] = p[i] + (1+e)*d*p_top.normal[i];
                vn_value += v[i] * p_top.normal[i];
                vn[i] = fabsf(vn_value) * p_top.normal[i];
                v_next[i] = (1+e) * vn[i] + v[i];

                break;
            default:
                break;
        }
        
        next[0][i] = p_next[i];
        next[1][i] = v_next[i];

    }
    
    return next;
}

float * computeAcceleration(int n)
{
    Force gravity;
    Force airResist;
    Force wind;
    static float acceleration[3] = { 0, 0, 0 };
    
    gravity.value[0] = 0;
    gravity.value[1] = - mass * g;
    gravity.value[2] = 0;
    wind.value[0] = 0.1;
    wind.value[1] = 0;
    wind.value[2] = 0.1;
    
    for (int i = 0; i < 3; i++) {
        airResist.value[i] = - ad * s_current.velocity[n][i];
        acceleration[i] = (gravity.value[i]+airResist.value[i]+wind.value[i]) / mass;
    }
    return acceleration;
    
    
}

int restingContact( float position[], float velocity[], float accel[])
{
    
    float vx = velocity[0];
    float vy = velocity[1];
    float vz = velocity[2];
    
    float v = sqrtf(vx*vx + vy*vy + vz*vz);
    //front
    if ((abs (position[2] - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += accel[i] *p_front.normal[i];
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //back
    if ((abs (- position[2] - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += accel[i] *p_back.normal[i];
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //right
    if ((abs (position[0] - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += accel[i] *p_right.normal[i];
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //left
    if ((abs (-position[0] - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += accel[i] *p_left.normal[i];
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //bot
    if ((abs (- position[1] - size)) < tolerance) {
        //printf("checking bottom plane");
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += accel[i] *p_bot.normal[i];
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    //top
    if ((abs (position[1] - size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        
        if (v > vtolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += accel[i] *p_top.normal[i];
            if (DotProduct < 0) {
                return 1;
            }
        }
    }
    return 0;
}

void update()
{
    float *accel;
    float **next;
    
    for (int n=0; n<N; n++) {
        
        //compute acceleration
        accel = computeAcceleration(n);
        for (int i=0; i<3; i++) {
            acceleration[n][i] = accel[i];
        }
        //integrate to get new state
        for (int i=0; i<3; i++) {
            s_next.velocity[n][i] = s_current.velocity[n][i] + acceleration[n][i] * h;
            s_next.position[n][i] = s_current.position[n][i] + s_current.velocity[n][i] * h;
            s_next.color[n][i] = s_current.color[n][i]+h/10;
        }
        //printf("next position for %d is %f", n, s_next.position[n][0]);
        
        //check for collision
        float d = CollisionDetection(n); //distance
        
        if (d != 0) {
            //compute new velocity and position
            next = CollisionResponse( d, s_next.position[n], s_next.velocity[n] );
            for (int i=0; i<3; i++) {
                s_next.velocity[n][i] = next[1][i];
                s_next.position[n][i] = next[0][i];
            }

        }
        
    }
    
    //update state
    s_current = s_next;
    timestep += h;
    for (int n=0; n<N; n++) {
        for (int i=0; i<3; i++) {
                tranPos[n][i] = s_current.position[n][i];
        }
        
        int outOfBox = 0;
        for (int i=0; i<3; i++) {
            if (fabsf(s_current.position[n][i]) > (2) )
                outOfBox = 1;
        }
        int stopped = restingContact(s_current.position[n],s_current.velocity[n],acceleration[n]);
        if (stopped == 1 || outOfBox == 1) {
            initialization(n);
        }
//        if (outOfBox == 1) {
//            initialization(n);
//        }
    
        
    }
//    printf("-----------------------------\n");
//    printf(" tx %f",tranPos[10][0] );
//    printf(" ty %f",tranPos[10][1] );
//    printf(" tz %f",tranPos[10][2] );

}

void idle() {
    double start = timestep;
    clock_t start_time = clock();
    update();
    
    double tau = 1.0 / frames_per_second;
    while (timestep - start < tau)
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
    
    // particles
    // Add particles to the scene.
    glPointSize(3);
    glBegin(GL_POINTS);
    for (int n=0; n<N; n++) {
        glColor3f(s_current.color[n][0],s_current.color[n][1],s_current.color[n][2]);
        glVertex3f(tranPos[n][0], tranPos[n][1], tranPos[n][2]);
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

void init() {
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.7, 0.7, 0.7, 1.0);
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
    
    //set initial state
    srand ((unsigned int)time(0));

    for (int n=0; n<N; n++) {
        initialization(n);
    }
    
    //initialize polygen
//    for (int i=0; i<3; i++) {
//        for (int j=0; j<3; j++) {
//            vertices[i][j] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1);
//        }
//    }
    vertices[0][0] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)-1.5;
    vertices[0][1] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)+1.5;
    vertices[0][2] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)+1.5;
    vertices[1][0] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)-1.5;
    vertices[1][1] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)-1.5;
    vertices[1][2] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)-1.5;
    vertices[2][0] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)+1.5;
    vertices[2][1] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)-1.5;
    vertices[2][2] = 0.5*((double)rand() / (double)RAND_MAX *2 - 1)+1.5;
    initPlaneNormal();
    //s_current = s_initial;

}




// The usual main function.
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    
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
    init();
    
    glutMainLoop();
}
