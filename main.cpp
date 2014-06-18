/**
* Dan Goldberg
* 
*/

#include <iostream>
#include <vector>
#include <string>

#include "aux/patch.h"
#include "aux/cloud.h"

int main() {
  std::string dir("/home/dan/pmvs/");
  std::string plyname("models/option-0000-crop-alpha.ply");
  
  MyCloud c;
  
  c.readPly(dir+plyname);
  c.readPatchInfo(dir);
  c.readCameras(dir);
  
}
