/**
 *
 * simpleShape.cpp
 *
 * routines for adding triangles to create a new mesh
 *
 *
 */

#ifdef __APPLE__
#include <OPENGL/gl.h>
#else
#include <GL/glew.h>
#include <GL/gl.h>
#endif

#include <vector>
#include <iostream>

using namespace std;

/**
 * common variables...should probably make this a class and make these
 * data members
 */
vector<float> points;
vector<float> normals;
vector<float> uv;
float *pointArray = 0;
float *normalArray = 0;
float *uvArray = 0;
GLushort *elemArray = 0;


/**
 * clear the current shape
 */
void clearShape ()
{
    if (pointArray) {
        delete pointArray;
        delete elemArray;
        delete normalArray;
        delete uvArray;
        pointArray = 0;
        elemArray = 0;
        normalArray = 0;
        uvArray = 0;
    }
    points.clear();
    normals.clear();
    uv.clear();
}


/**
 * adds a triangle to the current shape
 */
void addTriangle (float x0, float y0, float z0, float u0, float v0, 
                  float x1, float y1, float z1, float u1, float v1, 
                  float x2, float y2, float z2, float u2, float v2)
{
    points.push_back (x0);
    points.push_back (y0);
    points.push_back (z0);
    points.push_back (1.0);
    
    points.push_back (x1);
    points.push_back (y1);
    points.push_back (z1);
    points.push_back (1.0);
    
    points.push_back (x2);
    points.push_back (y2);
    points.push_back (z2);
    points.push_back (1.0);
    
    // calculate the normal
    float ux = x1 - x0;
    float uy = y1 - y0;
    float uz = z1 - z0;
    
    float vx = x2 - x0;
    float vy = y2 - y0;
    float vz = z2 - z0;
    
    float nx = (uy * vz) - (uz * vy);
    float ny = (uz * vx) - (ux * vz);
    float nz = (ux * vy) - (uy * vx);
    
    // Attach the normal to all 3 vertices
    for (int i=0; i < 3; i++) {
        normals.push_back (nx);
        normals.push_back (ny);
        normals.push_back (nz);
    }
    
    // Attach the texture coords
    uv.push_back (u0);
    uv.push_back (v0);
    uv.push_back (u1);
    uv.push_back (v1);
    uv.push_back (u2);
    uv.push_back (v2);
    
}


/**
 * gets the vertex points for the current shape
 */
float *getVerticies ()
{
    // delete the old point array of we have one
    if (pointArray) {
        delete pointArray;
    }
    
    // create and fill a new point array
    pointArray = new float[points.size()];
    for (int i=0; i < points.size(); i++) {
        pointArray[i] = points[i];
    }
        
    return pointArray;
}

/**
 * gets the normals for the current shape
 */
float *getNormals ()
{
    // delete the old point array of we have one
    if (normalArray) {
        delete normalArray;
    }
    
    // create and fill a new point array
    normalArray = new float[normals.size()];
    for (int i=0; i < normals.size(); i++) {
        normalArray[i] = normals[i];
    }
    
    return normalArray;
}

/**
 * gets the texture coords for the current shape
 */
float *getUV ()
{
    // delete the old point array of we have one
    if (uvArray) {
        delete uvArray;
    }
    
    // create and fill a new point array
    uvArray = new float[uv.size()];
    for (int i=0; i < uv.size(); i++) {
        uvArray[i] = uv[i];
    }
    
    return uvArray;
}



/**
 * gets the  array of elements for the  current shape
 */
GLushort *getElements ()
{
    // delete the old point array of we have one
    if (elemArray) {
        delete elemArray;
    }
    
    // create and fill a new point array
    elemArray = new GLushort[points.size()];
    for (int i=0; i < points.size(); i++) {
        elemArray[i] = i;
    }
    
    return elemArray;
}



/**
 * returns number of verticies in current shape
 */
int nVerticies ()
{
    return points.size() / 4;
}
