#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <GL/freeglut.h>
#include <GL/glu.h>

#include "wiiuse.h"

#define MAX_WIIMOTES 1

/////////////////////////////////////
// constants
/////////////////////////////////////

// threshold for gyroscopes update
const double omega_threshold = 1e-12;

// threshold on the accelerometer error to apply the correction
const double sin_threshold = 1e-6;

// strength of the accelerometers correction
const double alpha = 0.02;

// number of iteration to apply when resetting the matrix to cancel the drift (ugly hack)
const int nbIterReset = 200;

// conversion factors between degrees (wiimote, opengl) and radians (eigen)
const double deg_to_radians = M_PI/180.;
const double randians_to_deg = 180./M_PI;

// cooredinates constants to draw a cube
float cube_vert[8][3];

// global attributes
struct State {
    bool run; // true if the program should be running
    wiimote** wiimotes; // stores wiimote pointers
    std::chrono::steady_clock::time_point time;
    bool b_pressed;
    // IMU
    double pitch;
    double roll;
    double yaw;
    Eigen::Matrix3d wiimote_orient;
};

///////////////////////////////////////////
// Wiimote orientation update functions
///////////////////////////////////////////

// update the current wiimote rotation matrix with last data from the gyroscopes
// with code from chatgpt
// param: gyro_vec is the vector of angular speed from the gyroscopes, in radians per seconds
// param: elapsed_time elapsed time in seconds
// constant: omega_threshold, delat angle to allow the update (filter noise)
void updateFromGyroscopes(State& state, Eigen::Vector3d gyro_vec, double elapsed_time) {
    double angleOmega = gyro_vec.norm()*elapsed_time;
    if (angleOmega > omega_threshold) {
        Eigen::Vector3d axis = gyro_vec.normalized();
        Eigen::AngleAxisd delta(angleOmega, axis);
        state.wiimote_orient = state.wiimote_orient*delta.toRotationMatrix(); // rotate the current wiimote rotation matrix
    }
}

// update the rotation matrix from accelerometer data (gravity correction)
// with code from gemini
// param: accel, normalized acceleration vector extracted from the wiimote
// constant: alpha, correction strength
// constant: sin_threshold, error threshold to apply the correction
void updateFromAccelerometers(State& state, Eigen::Vector3d accel) {
    // extract the current "predicted" gravaity vector from the current wiimote rotation matrix
    Eigen::Vector3d gravity_predicted = state.wiimote_orient.row(2).transpose();
    Eigen::Vector3d error_axis = gravity_predicted.cross(accel);
    double error_sin = error_axis.norm();
    if (error_sin > sin_threshold) {
        error_axis.normalize();
        double error_angle = std::asin(error_sin);
        // Apply a small correction factor (alpha ~ 0.02) to avoid jitter from linear movements
        Eigen::AngleAxisd correction(alpha * error_angle, error_axis);
        // Update state.wiimote_orient to tilt its Z-axis back into alignment with gravity
        state.wiimote_orient = state.wiimote_orient*correction.toRotationMatrix();
    }
}

// update euler angles from the current predicted wiimote rotation matrix
void updateEulerAngles(State& state) {
    // extract angle from the current matrix
    Eigen::Vector3d ea = state.wiimote_orient.eulerAngles(2, 1, 0);
    ea *= randians_to_deg; // convert to degrees per seconds
    state.yaw = ea[0]; state.pitch = ea[1]; state.roll = ea[2]; 
    // std::cout << "Euler angles estimated: " << ea[0] << ", " << ea[1] << ", " << ea[2] << std::endl;
}


///////////////////////////////////////////
// Drawing functions
///////////////////////////////////////////

void initScene() {
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    int w = glutGet( GLUT_WINDOW_WIDTH );
    int h = glutGet( GLUT_WINDOW_HEIGHT );
    gluPerspective( 60, w / h, 0.1, 100 );
    glEnable(GL_DEPTH_TEST);

    // init cube coordinates
    // upper face
    cube_vert[0][0] = -1; cube_vert[0][1] = -1; cube_vert[0][2] = 1;
    cube_vert[1][0] = -1; cube_vert[1][1] = 1; cube_vert[1][2] = 1;
    cube_vert[2][0] = 1; cube_vert[2][1] = 1; cube_vert[2][2] = 1;
    cube_vert[3][0] = 1; cube_vert[3][1] = -1; cube_vert[3][2] = 1;
    // lower face
    cube_vert[4][0] = -1; cube_vert[4][1] = -1; cube_vert[4][2] = -1;
    cube_vert[5][0] = -1; cube_vert[5][1] = 1; cube_vert[5][2] = -1;
    cube_vert[6][0] = 1; cube_vert[6][1] = 1; cube_vert[6][2] = -1;
    cube_vert[7][0] = 1; cube_vert[7][1] = -1; cube_vert[7][2] = -1;
}

// the display function
void displayScene() {

    State& state = *static_cast<State*>(glutGetWindowData());

    //Clear all pixels
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(-3, 0, 0, 0, 0, 0, 0, 0, 1);
    glLineWidth(4.);

    // draw local wiimote frame
    glPushMatrix();
    // wiimote orientation
    glRotatef(state.yaw, 0, 0, 1);
    glRotatef(state.pitch, 0, 1, 0);
    glRotatef(state.roll, 1, 0, 0);
    glTranslatef(0., 0., 0.1);
    glScalef(0.5, 0.5, 0.5);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);
    glEnd();
    glPopMatrix();

    // draw a cube
    glPushMatrix();
    // set wiimote orientation
    glRotatef(state.yaw, 0, 0, 1);
    glRotatef(state.pitch, 0, 1, 0);
    glRotatef(state.roll, 1, 0, 0);
    glScalef(0.5, 0.2, 0.05);
    glBegin(GL_QUADS); // Cube
    glColor3f(1, 1, 0); // lower face
    glVertex3fv( cube_vert[0] ); glVertex3fv( cube_vert[1] ); glVertex3fv( cube_vert[2] ); glVertex3fv( cube_vert[3] );
    glColor3f(0, 1, 0); // // upper face
    glVertex3fv( cube_vert[4] ); glVertex3fv( cube_vert[5] ); glVertex3fv( cube_vert[6] ); glVertex3fv( cube_vert[7] );
    glColor3f(0, 0, 1); // left 0, 3, 4, 7
    glVertex3fv( cube_vert[0] ); glVertex3fv( cube_vert[3] ); glVertex3fv( cube_vert[7] ); glVertex3fv( cube_vert[4] );
    glColor3f(1, 0, 0); // right 1, 2, 5, 6
    glVertex3fv( cube_vert[1] ); glVertex3fv( cube_vert[2] ); glVertex3fv( cube_vert[6] ); glVertex3fv( cube_vert[5] );
    glColor3f(0, 1, 1); // front 2 3 6 7
    glVertex3fv( cube_vert[2] ); glVertex3fv( cube_vert[3] ); glVertex3fv( cube_vert[7] ); glVertex3fv( cube_vert[6] );
    glColor3f(1, 0, 1); // back  0 1 4 5
    glVertex3fv( cube_vert[0] ); glVertex3fv( cube_vert[1] ); glVertex3fv( cube_vert[5] ); glVertex3fv( cube_vert[4] );
    glEnd();
    glPopMatrix();

    // Don't wait start processing buffered OpenGL routines
    glFlush();

    glutSwapBuffers();
}

void processKeys(unsigned char key, int x, int y) {
    if(key == 27) { // exit key
        glutLeaveMainLoop();
        State& state = *static_cast<State*>(glutGetWindowData());
        state.run = false;
    }
}

void onIdle() {  
    glutPostRedisplay();
}


// process wiimote events and update the wiimote states on the user side
void updateWiimoteStates() {

    State& state = *static_cast<State*>(glutGetWindowData());

    // ideally, motion data should be processed only when new data packets are received from the wiimote
    // this is important for efficiency to avoid processing the same data twice
    // but also for accuracy since accelerometer data are integrated over time
    // unfortunately, wiiuse does not allow this (gyroscopes data update is not performed event-based)
    // a consequence is a sub-optimal accuracy regarding the wiimote gyroscope chip (in the wii_motion_plus for older wiimotes)
    // libxwiimote may be a good alternative

    while(state.run) {
        if(wiiuse_poll(state.wiimotes, MAX_WIIMOTES)) {
            wiimote_t* wm = state.wiimotes[0];

            if (IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_B)) {
                state.roll = 0;
                state.pitch = 0;
                state.yaw = 0;
                state.wiimote_orient = Eigen::Matrix3d::Identity();
                // ugly hack to re-orient the matrix according to current accelerometers values
                Eigen::Vector3d accel(state.wiimotes[0]->gforce.y, -state.wiimotes[0]->gforce.x, -state.wiimotes[0]->gforce.z);
                accel.normalize();
                for(int i = 0; i < nbIterReset; i ++) {
                    updateFromAccelerometers(state, accel);
                }
                updateEulerAngles(state);
            } else {

                ////////////////////////////////////////////////////
                // Wiimote orientation update procedure
                ////////////////////////////////////////////////////
                
                // update the roatation matrix from gyroscopes data
                // extract processed angular speed from the gyroscopes (wiiuse already add some processing from raw values)
                Eigen::Vector3d gyro_vec(-state.wiimotes[0]->exp.mp.angle_rate_gyro.roll, state.wiimotes[0]->exp.mp.angle_rate_gyro.pitch, state.wiimotes[0]->exp.mp.angle_rate_gyro.yaw);
                gyro_vec *= deg_to_radians; // convert to radians per seconds (wiiuse provide speeds in degree per seconds)
                // elapsed time, again, this should be recorded just after wii_motion_plus data packet is received (see comments in updateWiimoteStates function)
                auto elapsed_time = std::chrono::steady_clock::now()-state.time;
                state.time = std::chrono::steady_clock::now();
                double elapsed_time_sec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count()/1000.; 
                updateFromGyroscopes(state, gyro_vec, elapsed_time_sec);

                // accelerometers update, gemini code
                // extract current acceleration vector
                Eigen::Vector3d accel(state.wiimotes[0]->gforce.y, -state.wiimotes[0]->gforce.x, -state.wiimotes[0]->gforce.z);
                accel.normalize();
                updateFromAccelerometers(state, accel);

                // update euler angles after the rotation matrix update
                updateEulerAngles(state);
            }

    
        }
    }
}


int main(int argc, char** argv) {

    State state;
    // wiimote rotation matrix
    state.wiimote_orient = Eigen::Matrix3d::Identity();
    state.time = std::chrono::steady_clock::now();
    // yaw,pitch,roll angles
    state.roll = 0.;
    state.pitch = 0;
    state.yaw = 0;
    state.run = true;

    //////////////////////////////////
    // wiimote init
    //////////////////////////////////
	state.wiimotes =  wiiuse_init(MAX_WIIMOTES);
	bool found = wiiuse_find(state.wiimotes, MAX_WIIMOTES, 5);
	if (!found) {
		std::cout << "No wiimotes found." << std::endl;
		return 0;
	}
	bool connected = wiiuse_connect(state.wiimotes, MAX_WIIMOTES);
	if (connected) {
		std::cout << "Connected to " << connected << " wiimotes (of " << found << " found)." << std::endl;
	} else {
		std::cout << "Failed to connect to any wiimote." << std::endl;
		return 0;
	}
    for(int i = 0; i <  MAX_WIIMOTES; i ++) { // enable wii motion sensing (accelerometers + gyroscopes from wii_motion_plus)
        wiiuse_motion_sensing(state.wiimotes[i], 1);
        wiiuse_set_motion_plus(state.wiimotes[i], 1);   
        wiiuse_set_leds(state.wiimotes[i], WIIMOTE_LED_1);
    }

    //Initialise GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode( GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE );
    glutInitWindowSize(800, 600); //Set the window size
    glutInitWindowPosition(100,100); //Set the window position
    glutCreateWindow("Wiiuse IMU"); //Create the window
    glutSetWindowData(&state);

    // set up the wiimote thread
    // wiimote update occurs in a separate thread than the drawing one for efficiency reasons
    // note: no mutex is used here since each field of the state data structure is only modified by one thread
    std::thread wiimote_thread(updateWiimoteStates);
    wiimote_thread.detach(); // the function will run in background

    initScene();

    // set glut callback functions
    glutIdleFunc(onIdle);
    glutDisplayFunc(displayScene);
    glutKeyboardFunc(processKeys);

    // run the display loop
    glutMainLoop();

    // clean the wiimote objects
    wiiuse_cleanup(state.wiimotes, MAX_WIIMOTES);

    return 0;
}
