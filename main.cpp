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
  
  //std::cout<<c.readPly(dir+plyname)<<"\n";
  //std::cout<<c.readPatchInfo(dir)<<"\n";
  //std::cout<<c.readCameras(dir)<<"\n";
  double a;
  size_t SIZE(2000000);
  std::vector<Patch> A;
  for (int i=0;i<SIZE;i++) A.push_back(Patch p());
  std::cin.get();
}
