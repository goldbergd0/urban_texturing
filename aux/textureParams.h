//
//  lightingParams.h
//  
//
//  Created by Joe Geigel on 1/23/13.
//
//

#ifndef __textureParams__
#define __textureParams__

#ifdef __APPLE__
#include <GLUT/GLUT.h>
#include <OpenGL/gl.h>
#else
#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#endif

void loadTexture (char *filename);
void setUpTexture (GLuint program);

#endif 
