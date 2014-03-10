// File: main.cpp
// Author: Philip Salvaggio
// Computer Graphics I: Project 3

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <jpeglib.h>

// Scene Constants
const int WIDTH = 1280;
const int HEIGHT = 720;

// Camera parameters
GLdouble camera0[9] = { 10,  10, 20, 0, 0, 0, 0, 1, 0};
GLdouble camera1[9] = {-20,  10, 20, 0, 0, 0, 0, 1, 0};
int active_camera = 0;

const GLdouble FOV_Y = 10;  // [deg]
const GLdouble Z_NEAR = 1;
const GLdouble Z_FAR = 10000;

// Lighting Parameters
const GLfloat AMBIENT_LIGHT0[4] = { 0.15, 0.17, 0.2, 1 };

const GLfloat DIFFUSE_LIGHT1[4] = { 1, 0.3, 0, 1 };
const GLfloat LIGHT1_POS[4] = {-5, 5,  2.5, 1};
const GLfloat LIGHT1_DIR[3] = { 5,-5, -2.5};

const GLfloat SPECULAR_LIGHT2[4] = { 1, 1, 1, 1 };
const GLfloat DIFFUSE_LIGHT2[4] = {  0, 0.3, 1, 1 };
const GLfloat LIGHT2_POS[4] = {0, 3,  3, 1};
const GLfloat LIGHT2_DIR[3] = {0, -3, -3};

int lights_on[4] = {1, 1, 1, 1};

// Material Parameters
const GLfloat MATERIAL_SPECULAR[4] = {0, 0, 0, 1};
const GLint NOT_SHINY = 0;

const GLfloat TEAPOT_COLOR[4] = {0.2, 0.15, 0.05, 1};
const GLfloat CUBE_COLOR[4] = {0.1, 0.3, 0.1, 1};
const GLfloat WALL_COLOR[4] = {0.75, 0.55, 0.25, 1};
const GLfloat FLOOR_COLOR[4] = {1, 1, 1, 1};

GLuint texture_brick = 0, texture_floor = 0, texture_world = 0;

// Animation Parameters
GLdouble ball_position[3] = {-1.5, 0, 0};
GLdouble ball_velocity[3] = {0, 3, -0.2};
const GLdouble g = -9.8;

GLdouble cone_spin_angle = 0;
GLdouble cone_revs_per_sec = 1;
GLdouble cone_center_of_orbit[3] = {1.5, 0, 0};
GLdouble cone_orbit_angle = 0;
GLdouble cone_orbit_radius = 1.5;
const GLdouble cone_orbital_speed = 0.5; // [rev/s]

int animation_active = 0;

// Scene Boundaries
const GLdouble FLOOR_POS = -0.5;
const GLdouble WALL_POS = -3;


void Display();

/**
 * LoadTexture - Loads an texture from a raw data file.
 *               Data stored as RGBRGBRGB (8-bit).
 * Returns texture id.
 */
GLuint LoadTexture(const char* filename, int width, int height) {
  GLuint texture = 0;
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

  FILE* file = fopen(filename, "rb");

  unsigned char* data = new unsigned char[width * height * 3];

  fread(data, width * height * 3, 1, file);
  fclose(file);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
               GL_RGB, GL_UNSIGNED_BYTE, data);

  delete[] data;

  return texture;
}

/**
 * UseCamera - Switches the camera.
 */
void UseCamera(int camera) {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  if (active_camera) {
    gluLookAt(camera1[0], camera1[1], camera1[2], camera1[3], camera1[4],
              camera1[5], camera1[6], camera1[7], camera1[8]);
  } else {
    gluLookAt(camera0[0], camera0[1], camera0[2], camera0[3], camera0[4],
              camera0[5], camera0[6], camera0[7], camera0[8]);
  }
  glutPostRedisplay();
}

/**
 * Move the fractal cone, both spin and orbital
 */
void MoveCone() {
  const double t = 1 / 30.0;

  cone_spin_angle += cone_revs_per_sec * t * 360;
  if (cone_spin_angle > 360)
    cone_spin_angle -= ((int) cone_spin_angle / 360) * 360;
  if (cone_spin_angle < 0)
    cone_spin_angle -= (((int) cone_spin_angle + 1) / 360 - 1) * 360;

  cone_orbit_angle += t * cone_orbital_speed * 360;
  if (cone_orbit_angle > 360)
    cone_orbit_angle -= ((int) cone_orbit_angle / 360) * 360;
  if (cone_orbit_angle < 0)
    cone_orbit_angle -= (((int) cone_orbit_angle + 1) / 360 - 1) * 360;
}

/**
 * MoveBall - Update the position and velocity of the ball.
 */
void MoveBall() {
  const double t = 1 / 30.0;
  ball_position[0] = ball_position[0] + ball_velocity[0] * t;

  double next_y = ball_position[1] + ball_velocity[1] * t + 0.5 * g * t * t;
  double next_vy = ball_velocity[1] + g;
  if (next_y < 0) {
    // 0 = 1/2 * g * t^2  + v_y * t + y_0
    // t = (-vy +/- sqrt(v_y^2 - 2gy_0)) / g;
    double sqrt_term = sqrt(ball_velocity[1]*ball_velocity[1] - 
                            2 * g * ball_position[1]) / g;
    double t_hit = -ball_velocity[1]/g + fabs(sqrt_term);
    double v_at_hit = -1 * (ball_velocity[1] + t_hit * g);
    next_y = v_at_hit * (t-t_hit) + 0.5 * g * (t-t_hit)*(t-t_hit);

    v_at_hit += (t-t_hit) * g;
    ball_velocity[1] = v_at_hit;
  } else {
    ball_velocity[1] += g * t;
  }
  ball_position[1] = next_y; 

  double collision_pt = WALL_POS + 0.5;
  double next_z = ball_position[2] + ball_velocity[2];
  if (next_z < collision_pt) {
    next_z = 2 * collision_pt - next_z;
    ball_velocity[2] = -1 * ball_velocity[2];
  } else if (next_z > 0) {
    next_z = -1 * next_z;
    ball_velocity[2] = -1 * ball_velocity[2];
  }
  ball_position[2] = next_z;
}

/**
 * AnimationMain - The main animation handler.
 */
void AnimationMain(int value) {
  if (value == 0) return;

  MoveBall();
  MoveCone();

  Display();
  glutTimerFunc(33, AnimationMain, animation_active);
}

/**
 * ToggleAnimation - Turns animation off and on.
 */
void ToggleAnimation() {
  animation_active = 1 - animation_active;
  if (animation_active) {
    glutTimerFunc(33, AnimationMain, animation_active);
  }
}

/**
 * SetUpLights - Sets up the lights in the current OpenGL modelview matrix.
 */
void SetUpLights() {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLightfv(GL_LIGHT0, GL_AMBIENT, AMBIENT_LIGHT0);
  glLightfv(GL_LIGHT0, GL_SPECULAR, MATERIAL_SPECULAR);
  glLightfv(GL_LIGHT0, GL_POSITION, LIGHT1_POS);

  glLightfv(GL_LIGHT1, GL_DIFFUSE, DIFFUSE_LIGHT1);
  glLightfv(GL_LIGHT1, GL_SPECULAR, SPECULAR_LIGHT2);
  glLightfv(GL_LIGHT1, GL_POSITION, LIGHT1_POS);
  glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, LIGHT1_DIR); 

  glLightfv(GL_LIGHT2, GL_DIFFUSE, DIFFUSE_LIGHT2);
  glLightfv(GL_LIGHT2, GL_SPECULAR, SPECULAR_LIGHT2);
  glLightfv(GL_LIGHT2, GL_POSITION, LIGHT2_POS);
  glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, LIGHT2_DIR); 
  glPopMatrix();
}

/**
 * init - Sets up initial OpenGL configuration.
 */
void init() {
  glShadeModel(GL_SMOOTH);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(FOV_Y, static_cast<double>(WIDTH) / HEIGHT, Z_NEAR, Z_FAR);

  UseCamera(0);

  glEnable(GL_DEPTH_TEST);
  
  glClearColor(0, 0, 0, 0);
  glColor4f(1, 1, 1, 1);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT2);
  glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL,GL_SEPARATE_SPECULAR_COLOR);
   
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
  texture_brick = LoadTexture("brick.texture", 512, 512);
  texture_floor = LoadTexture("floor.texture", 512, 512);
  texture_world = LoadTexture("world.texture", 1024, 512);

  glEnable(GL_COLOR_MATERIAL);  

  SetUpLights();
}

/**
 * ToggleLight - Turns lights off and on.
 */
void ToggleLight(int light_index) {
  lights_on[light_index] = 1 - lights_on[light_index];
  GLint light = GL_LIGHT0;
  switch (light_index) {
    case 0: light = GL_LIGHT0; break;
    case 1: light = GL_LIGHT1; break;
    case 2: light = GL_LIGHT2; break;
    case 3: light = GL_LIGHT3; break;
  }
  if (lights_on[light_index])
    glEnable(light);
  else
    glDisable(light);

  glutPostRedisplay();
}

/**
 * KeyboardHandler - Manages user interaction.
 */
void KeyboardHandler(unsigned char key, int x, int y) {
  switch (key) {
    case 99:  // 'c'
      active_camera = 1 - active_camera;
      UseCamera(active_camera);
      break;
    case 97:  // 'a'
      ToggleAnimation();
      break;
    case 113:  //'q'
      exit(0);
    case 43:  // '+'
      cone_revs_per_sec += 0.25;
      break;
    case 45:  // '-'
      cone_revs_per_sec -= 0.25;
      break;
    case 49:  // '1'
    case 50:  // '2'
    case 51:  // '3'
    case 52:  // '4'
      int light_index = static_cast<int>(key) - 49;
      ToggleLight(light_index);
      break;
  }
}

void DisplayConeHierarchy(int level) {
  if (level == 6) return;

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  double scale = 1.0/(1 << level);
  
  glutSolidCone(scale/2.0, scale, 60, 1);

  glPushMatrix();
  glRotatef(60, 0, 1, 0);
  glTranslatef(-scale / 4.0, 0, scale / 3.0);
  DisplayConeHierarchy(level+1);
  glPopMatrix();

  glPushMatrix();
  glRotatef(-60, 0, 1, 0);
  glTranslatef(scale / 4.0, 0, scale / 3.0);
  DisplayConeHierarchy(level+1);
  glPopMatrix();

  glPopMatrix();
}

/**
 * Display - The main display function.
 */
void Display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  SetUpLights();

  // Draw the teapot.
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(cone_center_of_orbit[0], cone_center_of_orbit[1],
               cone_center_of_orbit[2]);

  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128);
  glColorMaterial(GL_FRONT, GL_SPECULAR);
  glColor3f(1, 1, 1);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColor4fv(TEAPOT_COLOR);
  glutSolidTeapot(0.5);

  glPopMatrix();

  // Draw the ball.
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColor3f(1, 1, 1);
  glColorMaterial(GL_FRONT, GL_SPECULAR);
  glColor3f(1, 1, 1);

  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(1,-1,1);

  glEnable(GL_TEXTURE_2D);
  glEnable(GL_TEXTURE_GEN_S);
  glEnable(GL_TEXTURE_GEN_T);
  glBindTexture(GL_TEXTURE_2D, texture_world);
  glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
  glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(ball_position[0], ball_position[1], ball_position[2]);
  glutSolidSphere(0.5, 35, 35);
  glPopMatrix();

  glMatrixMode(GL_TEXTURE);
  glPopMatrix();

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);

  // Draw the cube.
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColor4fv(CUBE_COLOR);
  glColorMaterial(GL_FRONT, GL_SPECULAR);
  glColor3f(1, 1, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(ball_position[0], FLOOR_POS + 0.5, 1);
  glutSolidCube(1);
  glPopMatrix();

  // Render the brick wall.
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColor4fv(WALL_COLOR);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_brick);
  glBegin(GL_QUADS);
    glNormal3f(0, 0, 1);
    glTexCoord2i(0,0); glVertex3f(-100, -100, WALL_POS);
    glNormal3f(0, 0, 1);
    glTexCoord2i(200,0); glVertex3f(100, -100, WALL_POS);
    glNormal3f(0, 0, 1);
    glTexCoord2i(200,200); glVertex3f(100, 100, WALL_POS);
    glNormal3f(0, 0, 1);
    glTexCoord2i(0,200); glVertex3f(-100, 100, WALL_POS);
  glEnd();
  glDisable(GL_TEXTURE_2D);

  // Render the floor.
  glColorMaterial(GL_FRONT, GL_SPECULAR);
  glColor3f(1, 1, 1);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glColor4fv(FLOOR_COLOR);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture_floor);
  glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glTexCoord2i(0,0); glVertex3f(-100, FLOOR_POS, -100);
    glNormal3f(0, 1, 0);
    glTexCoord2i(200,0); glVertex3f(100, FLOOR_POS, -100);
    glNormal3f(0, 1, 0);
    glTexCoord2i(200,200); glVertex3f(100, FLOOR_POS, 100);
    glNormal3f(0, 1, 0);
    glTexCoord2i(0,200); glVertex3f(-100, FLOOR_POS, 100);
  glEnd();
  glDisable(GL_TEXTURE_2D);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(cone_center_of_orbit[0], cone_center_of_orbit[1],
               cone_center_of_orbit[2]);
  glRotatef(cone_orbit_angle, 0, 1, 0);
  glTranslated(cone_orbit_radius, 0, 0);
  glRotatef(cone_spin_angle, 0, 1, 0);
  glRotatef(-90, 1, 0, 0);
  DisplayConeHierarchy(0);
  glPopMatrix();

  glutSwapBuffers();
  glFlush();
}

/**
 * main - Launch the application and configure GLUT.
 */
int main(int argc, char** argv) {
  glutInit(&argc, argv);

  // Animation is smoother if we use double buffering,
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

  // Set up the window
  glutInitWindowSize(WIDTH, HEIGHT); // 720p
  glutInitWindowPosition(0, 0);
  glutCreateWindow("Philip Salvaggio's Project 3");

  init();

  glutKeyboardFunc(KeyboardHandler);
  glutDisplayFunc(Display);
  glutMainLoop();
  return 0;
}
