/**
* Dan Goldberg
* 
*/

#include <iostream>
#include <vector>
#include <string>

#include "aux/patch.h"
#include "aux/cloud.h"

std::string dir("/home/dan/pmvs/");
std::string plyname("models/option-0000-crop-alpha.ply");

Cloud c;
c.getPoints();
//c.readPly(dir+plyname);
//c.readPatchInfo(dir);
//c.readCameras(dir);
