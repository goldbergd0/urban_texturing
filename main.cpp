/**
* Dan Goldberg
* 
*/

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include "aux/patch.h"
#include "aux/cloud.h"

int main() {
  std::string dir("/home/dan/pmvs/");
  std::string plyname("models/option-0000-crop-alpha.ply");
  
  MyCloud c;
  
  std::cout<<c.readPly(dir+plyname)<<"\n";
  std::cout<<c.readPatchInfo(dir)<<"\n";
  std::cout<<c.readCameras(dir)<<"\n";
  
  std::string line;
  std::string fl("-2.17703 1.54542 -8.80738 1 \n"
                  "-1.24213 123.123 124.120");
  std::istringstream ss;
  std::istringstream ss2;
  ss.str(fl);
  float f(0);
  getline(ss,line);
  ss2.str(line);
  ss2>>f;
  std::cout<<f<<"\n";
}
