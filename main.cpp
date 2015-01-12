//
//  main.c
//  BouncingBall
//
//  Created by Uriana on 9/10/13.
//  Copyright (c) 2013 Uriana. All rights reserved.
//


#include <math.h>
#include <stdio.h>
#include <stdlib.h>   
#include <time.h>
#include <unistd.h>

#ifdef __APPLE__
#  include <GLUI/glui.h>
#else
#  include <GL/glui.h>
#endif

#define WIDTH 800
#define HEIGHT 600

#define RANDOM_BUTTON 1

//used colors
GLfloat black[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat yellow[] = { 1.0, 1.0, 0.0, 1.0 };
GLfloat pink[] = { 0.9, 0.4, 0.4, 1.0 };
GLfloat cyan[] = { 0.0, 1.0, 1.0, 1.0 };
GLfloat white[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat direction[] = { 1.0, 1.0, 1.0, 0.0 };

//structures
typedef struct {
    float position[3];
    float velocity[3];
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
Plane p_left, p_right, p_bot, p_top, p_back, p_front;
float acceleration[3];


float tranPos[3];     //translating position
double t;             // simulation time
int planeIndicator;     //which plane it collides

//public defined parameters
double size = 2;
double radius = 0.2;    //radius of ball
double v_max = 1;     // maximum initial speed of ball
double x_max = 2;       // maximum initial position of ball
float mass = 3;     //mass of ball
float g = 1;        //gravity
float d = 0.1;      //air resistance factor
float e = 0.5;      //elasticity factor
float miu = 0.5;    //friction factor


double h = 0.1;     // time step for Euler Integration
int frames_per_second = 30;   // display rate
float tolerance = 0.001;//fabsf(h/log(h));    // error tolerance

//GUI parameters
int main_window;
GLboolean paused = GL_FALSE;    //pause the animation
GLboolean windExist = GL_FALSE;
GLfloat posx = -1.0;    //camera position
GLboolean randomVelocity = GL_FALSE;

GLUI_EditText *vmaxField;
GLUI_Spinner *vx;
GLUI_Spinner *vy;
GLUI_Spinner *vz;
GLUI_Spinner *m;
GLUI_Spinner *r;
GLUI_Spinner *timeS;
GLUI_RadioGroup *disL;
GLUI_EditText *gField;
GLUI_EditText *dField;
GLUI_EditText *eField;
GLUI_EditText *miuField;
GLUI_Checkbox *windBox;

void box()
{
    glPushMatrix();
    glScaled((size+radius)*2, (size+radius)*2, (size+radius)*2);
    glRotatef(5.0, 0.0, 1.0, 0.0);
    
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cyan);
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialf(GL_FRONT, GL_SHININESS, 50);
    glDepthMask(GL_TRUE);
    glutWireCube(1.0);
    
    // Yellow side - BACK
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, yellow);
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialf(GL_FRONT, GL_SHININESS, 50);
    
    glBegin(GL_POLYGON);
    glVertex3f(  0.5, -0.5, -0.5 );
    glVertex3f(  0.5,  0.5, -0.5 );
    glVertex3f( -0.5,  0.5, -0.5 );
    glVertex3f( -0.5, -0.5, -0.5 );
    glEnd();
    
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

    
        
    // Cyan side - LEFT
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cyan);
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialf(GL_FRONT, GL_SHININESS, 50);
    
    glBegin(GL_POLYGON);
    glVertex3f( 0.5, -0.5, -0.5 );
    glVertex3f( 0.5,  0.5, -0.5 );
    glVertex3f( 0.5,  0.5,  0.5 );
    glVertex3f( 0.5, -0.5,  0.5 );

    glEnd();
    
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


    
    // Black side - BOTTOM
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, black);
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialf(GL_FRONT, GL_SHININESS, 50);
    
    glBegin(GL_POLYGON);
    glVertex3f(  0.5, -0.5, -0.5 );
    glVertex3f(  0.5, -0.5,  0.5 );
    glVertex3f( -0.5, -0.5,  0.5 );
    glVertex3f( -0.5, -0.5, -0.5 );
     
    glEnd();
    
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

    
    glPopMatrix();
    
}

void ball()
{
    glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, pink);
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialf(GL_FRONT, GL_SHININESS, 30);
    glDepthMask(GL_TRUE);
    
    glTranslatef(tranPos[0],tranPos[1],tranPos[2]);
    glutSolidSphere(radius, 100, 100);
    
    glPopMatrix();
}

void initialization()
{
    srand ((unsigned int)time(0));
    
    s_initial.position[0] = x_max * ((double)rand() / (double)RAND_MAX *2 - 1);
    s_initial.position[1] = x_max * ((double)rand() / (double)RAND_MAX *2 - 1);
    s_initial.position[2] = x_max * ((double)rand() / (double)RAND_MAX *2 - 1);

    s_initial.velocity[0] = 0;
    s_initial.velocity[1] = 0;
    s_initial.velocity[2] = 0;
    
    
    s_current = s_initial;
    //paused = GL_TRUE;
}

void restart()
{
    srand ((unsigned int)time(0));
    
    s_current.position[0] = x_max * ((double)rand() / (double)RAND_MAX *2 - 1);
    s_current.position[1] = x_max * ((double)rand() / (double)RAND_MAX *2 - 1);
    s_current.position[2] = x_max * ((double)rand() / (double)RAND_MAX *2 - 1);
    
    if (randomVelocity) {
        s_current.velocity[0] = v_max * ((double)rand() / (double)RAND_MAX *2 - 1);
        s_current.velocity[1] = v_max * ((double)rand() / (double)RAND_MAX *2 - 1);
        s_current.velocity[2] = v_max * ((double)rand() / (double)RAND_MAX *2 - 1);
        vx->set_float_val(s_current.velocity[0]);
        vy->set_float_val(s_current.velocity[1]);
        vz->set_float_val(s_current.velocity[2]);

    }else {
        s_current.velocity[0] = vx->get_float_val();
        s_current.velocity[1] = vy->get_float_val();
        s_current.velocity[2] = vz->get_float_val();
    }
    randomVelocity = GL_FALSE;

    mass = m->get_float_val();
    radius = (double) r->get_float_val();
    v_max = (double) vmaxField->get_float_val();     // maximum initial speed of ball
    g = gField->get_float_val();        //gravity
    d = dField->get_float_val();      //air resistance factor
    e = eField->get_float_val() ;      //elasticity factor
    miu = miuField->get_float_val();    //friction factor
    h = (double)timeS->get_float_val();
    //printf("RESTART!!!!!!!!! mass %f radius %f vmax %f gra %f airr %f elas %f fric %f timestep %f",mass, radius,v_max,g,d,e,miu,h);
    frames_per_second = 30;
    disL->set_int_val(1);
    windExist = windBox->get_int_val();
}

float PlaneCollision(Plane p)
{
    float s_vector_current[3];
    float s_vector_next[3];
    float DotProduct_current = 0 ;
    float DotProduct_next = 0;
    
    for (int i=0; i<3; i++) {
        s_vector_current[i] = s_current.position[i] - p.point[i];
        s_vector_next[i] = s_next.position[i] - p.point[i];
        DotProduct_current += s_vector_current[i] * p.normal[i];
        DotProduct_next += s_vector_next[i] * p.normal[i];
    }

    // Determine if it didnt collide
    float f;

    if (( DotProduct_current > 0 && DotProduct_next > 0) || (DotProduct_current < 0 && DotProduct_next < 0))
        f = 0;
    else 
        f = (fabsf(DotProduct_current)) / (fabsf(DotProduct_next)+fabsf(DotProduct_current));
    // fraction of timestep where it happened
    

    return f;
    
}

float CollisionDetection()
{
    
    float f = 0;
    
    if (PlaneCollision(p_front) != 0) {
        planeIndicator = 1;
        f = PlaneCollision(p_front);
        return f;
    }
    if (PlaneCollision(p_back) != 0) {
        planeIndicator = 2;
        f = PlaneCollision(p_back);
        return f;
    }
    if (PlaneCollision(p_right) != 0) {
        planeIndicator = 3;
        f = PlaneCollision(p_right);
        return f;
    }
    if (PlaneCollision(p_left) != 0) {
        planeIndicator = 4;
        f = PlaneCollision(p_left);
        return f;
    }
    if (PlaneCollision(p_bot) != 0) {
        planeIndicator = 5;
        f = PlaneCollision(p_bot);
        return f;
    }
    if (PlaneCollision(p_top) != 0) {
        planeIndicator = 6;
        f = PlaneCollision(p_top);
        return f;
    }

    return 0;

}

float * CollisionResponse (float v[])
{
    float vn[3], vn_next[3], vt[3], vt_next[3];
    static float v_next[3];
    float vn_value = 0;
    
    for (int i = 0; i < 3; i++) {
        switch (planeIndicator) {
            case 1:
                vn_value += v[i] * p_front.normal[i];
                vn[i] = vn_value * p_front.normal[i];
                break;
            case 2:
                vn_value += v[i] * p_back.normal[i];
                vn[i] = vn_value * p_back.normal[i];
                break;
            case 3:
                vn_value += v[i] * p_right.normal[i];
                vn[i] = vn_value * p_right.normal[i];
                break;
            case 4:
                vn_value += v[i] * p_left.normal[i];
                vn[i] = vn_value * p_left.normal[i];
                break;
            case 5:
                vn_value += v[i] * p_bot.normal[i];
                vn[i] = vn_value * p_bot.normal[i];
                break;
            case 6:
                vn_value += v[i] * p_top.normal[i];
                vn[i] = vn_value * p_top.normal[i];
                break;
            default:
                break;
        }
        //compute vt
        vt[i] = v[i] - vn[i];
        //compute elasticity
        vn_next[i] = - e * vn[i];
        //compute friction
        vt_next[i] = ( 1 - miu) * vt[i];
        v_next[i] = vn_next[i] + vt_next[i];
    }

    return v_next;
}

float * computeAcceleration()
{
    Force gravity;
    Force airResist;
    Force wind;
    static float acceleration[3] = { 0, 0, 0 };
    if (windExist) {
        wind.value[0] = 0.5;
        wind.value[1] = 0;
        wind.value[2] = 0;
    }else{
    wind.value[0] = 0;
    wind.value[1] = 0;
    wind.value[2] = 0;
    }
    gravity.value[0] = 0;
    gravity.value[1] = - mass * g;
    gravity.value[2] = 0;
    
    for (int i = 0; i < 3; i++) {
        airResist.value[i] = - d * s_current.velocity[i];
        acceleration[i] = (wind.value[i]+gravity.value[i]+airResist.value[i]) / mass;
    }
    return acceleration;
    
    
}

float toleranceControl(double h)
{
    float tolerance;
    float accel_value;
    accel_value =
    sqrtf(acceleration[0] * acceleration[0] +acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]);
//    tolerance = 0.1 / powf(0.1,(accel_value *h));
    if (h *accel_value < 0.1) {
        tolerance = 0.001;
    }
    else if (h *accel_value<0.2) {
        tolerance = 0.05;
    }else if (h *accel_value<0.5) {
        tolerance = 0.1;
    }else if (h *accel_value<0.7) {
        tolerance = 0.5;
    }else if (h *accel_value<0.9)
        tolerance = 0.7;
    else
        tolerance = 0.85;
    return tolerance;
}

void restingContact()
{

    //printf("tolerance %f",tolerance);
    tolerance = toleranceControl(h);
    //printf("tolerance %f",tolerance);

    float v_x = s_current.velocity[0];
    float v_y = s_current.velocity[1];
    float v_z = s_current.velocity[2];

    float v = sqrtf(v_x*v_x + v_y*v_y + v_z*v_z);
    
    //front
    if ((fabsf (s_current.position[2] - (float)size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        if (v > tolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += acceleration[i] *p_front.normal[i];
            if (DotProduct < 0) {
                restart();
            }
        }
    }
    //back
    if ((fabsf (- s_current.position[2] - (float)size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;
        if (v > tolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += acceleration[i] *p_back.normal[i];
            if (DotProduct < 0) {
                restart();
            }
        }
    }
    //right
    if ((fabsf (s_current.position[0] - (float)size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;

        if (v > tolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += acceleration[i] *p_right.normal[i];
            if (DotProduct < 0) {
                restart();
            }
        }
    }
    //left
    if ((fabsf (- s_current.position[0] - (float)size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;

        if (v > tolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += acceleration[i] *p_left.normal[i];
            if (DotProduct < 0) {
                restart();
            }
        }
    }
    //bot
    if ((fabsf (- s_current.position[1] - (float)size)) < tolerance) {
        //printf("current timestep %f",h);
        //printf("v value %f",v);
        GLboolean noVelocity = GL_TRUE;
        if (v > tolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += acceleration[i] *p_bot.normal[i];
            if (DotProduct < 0) {
                restart();
            }
        }
    }
    //top
    if ((abs (s_current.position[1] - (float)size)) < tolerance) {
        GLboolean noVelocity = GL_TRUE;

        if (v > tolerance)
            noVelocity = GL_FALSE;
        if (noVelocity == GL_TRUE) {
            float DotProduct = 0;
            for (int i = 0; i < 3 ; i++)
                DotProduct += acceleration[i] *p_top.normal[i];
            if (DotProduct < 0) {
                restart();
            }
        }
    }
}

void update()
{
    float *accel;
    double timestepremain = h;
    State s_c;
    float * v_next;
    
    while (timestepremain > 0) {
        //compute acceleration
        accel = computeAcceleration();
        for (int i=0; i<3; i++) {
            acceleration[i] = accel[i];
        }
        //integrate to get new state
        for (int i=0; i<3; i++) {
            s_next.velocity[i] = s_current.velocity[i] + acceleration[i] * h;
            s_next.position[i] = s_current.position[i] + s_current.velocity[i] * h;
        }
        
        //check for collision
        float f = CollisionDetection(); //fraction of timestep
        
        if (f != 0) {
            for (int i=0; i<3; i++) {
                //get collision state
                s_c.velocity[i] = s_current.velocity[i] + acceleration[i] * h * f;
                s_c.position[i] = s_current.position[i] + s_current.velocity[i] * h * f;
                //compute new position
                s_next.position[i] = s_c.position[i];
            }
            //compute new velocity
            v_next = CollisionResponse( s_c.velocity );
            for (int i=0; i<3; i++) {
                s_next.velocity[i] = v_next[i];
            }
            
            timestepremain = f * timestepremain;
            
        }else{
            timestepremain = 0;
        }
        
        //update state
        s_current = s_next;
        t += h;
        for (int i=0; i<3; i++) {
            tranPos[i] = s_current.position[i];
        }
        restingContact();
    
    }
}

void idle() {        
    
    glutSetWindow(main_window);
    
    if (!paused) {
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
    //sleep (100);
    
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//clear buffer
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
 	gluLookAt(posx, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);//isometric view
    
    // BALL
    // Add a sphere to the scene.
    ball();
    
    // BOX
    box();

    
	glutSwapBuffers();
}

void reshape(GLint w, GLint h)
{
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
    if(btn==GLUT_LEFT_BUTTON && state == GLUT_DOWN){
        paused = !paused;
    }
}

void keyboard(int key, int xx, int yy) {
    switch (key) {
        case GLUT_KEY_LEFT:
            posx -= 0.1;
            break;
        case GLUT_KEY_RIGHT:
            posx += 0.1;
            break;
        default:
            break;
    }
}

void gluicallback(int control) {
    switch (control) {
 
        case 1:
            randomVelocity = GL_TRUE;
            break;
        case 2:
            if (disL->get_float_val() == 0) {
                frames_per_second = 10;
            }else if (disL->get_float_val() ==1){
                frames_per_second = 30;
            }else {
                frames_per_second = 60;
            }
            break;
        case -1:
            restart();
            break;
        default:
            break;
    }
       
}
void init() {
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    
    
    glLightfv(GL_LIGHT0, GL_AMBIENT, black);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
    glLightfv(GL_LIGHT0, GL_POSITION, direction);
    glEnable(GL_NORMALIZE);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glEnable(GL_LIGHTING);                // so the renderer considers light
    glEnable(GL_LIGHT0);                  // turn LIGHT0 on
    glEnable(GL_DEPTH_TEST);              // so the renderer considers depth
    
    //set initial state
    initialization();

}


// The usual main function.
int main(int argc, char** argv) {
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(80, 80);
    glutInitWindowSize(WIDTH, HEIGHT);
    main_window = glutCreateWindow("Bouncing Ball");
    glShadeModel(GL_FLAT);
    glutMouseFunc(mymouse);
    glutSpecialFunc(keyboard);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
 

    init();
    //  pointer to a GLUI window
    GLUI * glui_window;
    
    //  Create GLUI window
    glui_window = GLUI_Master.create_glui ("Options");
    
    GLUI_Panel *bp_panel = glui_window->add_panel ("Ball Properties");
    vmaxField = glui_window->add_edittext_to_panel(bp_panel, "max velocity",GLUI_EDITTEXT_FLOAT);
    vmaxField->set_float_limits(0, 10);
    vmaxField->set_float_val(v_max);
    glui_window->add_separator_to_panel(bp_panel);

    vx = glui_window->add_spinner_to_panel(bp_panel, "initial vx",GLUI_SPINNER_FLOAT);
    vx->set_float_limits(-v_max, v_max);
    vx->set_speed(0.01);
    
    vy = glui_window->add_spinner_to_panel(bp_panel, "initial vy",GLUI_SPINNER_FLOAT);
    vy->set_float_limits(-v_max, v_max);
    vy->set_speed(0.01);

    vz = glui_window->add_spinner_to_panel(bp_panel, "initial vz",GLUI_SPINNER_FLOAT);
    vz->set_float_limits(-v_max, v_max);
    vz->set_speed(0.01);

    glui_window->add_button_to_panel(bp_panel, "Random",1,gluicallback);
    
    glui_window->add_separator_to_panel(bp_panel);
    
    m = glui_window->add_spinner_to_panel(bp_panel, "mass",GLUI_SPINNER_FLOAT);
    m->set_float_limits(0, 10);
    m->set_speed(0.1);
    m->set_float_val(mass);
    r = glui_window->add_spinner_to_panel(bp_panel, "radius",GLUI_SPINNER_FLOAT);
    r->set_float_limits(0, 1);
    r->set_speed(0.1);
    r->set_float_val(radius);
    
    GLUI_Panel *sp_panel = glui_window->add_panel ("Simulation Properties");
    timeS = glui_window->add_spinner_to_panel(sp_panel, "time step size",GLUI_SPINNER_FLOAT);
    timeS->set_float_limits(0, 1);
    timeS->set_speed(0.01);
    timeS->set_float_val(h);
    disL = glui_window->add_radiogroup_to_panel(sp_panel,NULL,2,gluicallback);
    new GLUI_RadioButton(disL, "10 fps");
    new GLUI_RadioButton(disL, "30 fps");
    new GLUI_RadioButton(disL, "60 fps");
    disL->set_int_val(1);

    GLUI_Panel *ep_panel = glui_window->add_panel ("Environment Properties");
    windBox = glui_window->add_checkbox_to_panel(ep_panel, "Wind");
    windBox->set_int_val(0);
    gField = glui_window->add_edittext_to_panel(ep_panel, "gravity constant",GLUI_EDITTEXT_FLOAT);
    gField->set_float_limits(0, 10);
    gField->set_float_val(g);
    
    dField = glui_window->add_edittext_to_panel(ep_panel, "airresis constant",GLUI_EDITTEXT_FLOAT);
    dField->set_float_limits(0, 5);
    dField->set_float_val(d);

    eField = glui_window->add_edittext_to_panel(bp_panel, "Elasticity factor",GLUI_EDITTEXT_FLOAT);
    eField->set_float_limits(0, 1.5);
    eField->set_float_val(e);

    miuField = glui_window->add_edittext_to_panel(bp_panel, "Friction factor",GLUI_EDITTEXT_FLOAT);
    miuField->set_float_limits(0, 1);
    miuField->set_float_val(miu);

    
    glui_window->add_button("Restart",-1, gluicallback);
    glui_window->add_button("Quit",0, (GLUI_Update_CB)exit);

    GLUI_Master.sync_live_all();
    GLUI_Master.set_glutIdleFunc (idle);
    glui_window->set_main_gfx_window (main_window);
    glutMainLoop();
}