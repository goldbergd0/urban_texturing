/**
* Dan Goldberg
* 
*/

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <Eigen/Dense>

#include <pcl/point_types.h>

#include "aux/patch.h"
#include "world/world.h"

int main() {
  std::string dir("/home/dan/pmvs/");
  std::string plyname("models/option-0000-crop-alpha.ply");
  
  int totNimgs(48);
  
  World w;
  
  std::cout<<w.readPly(dir+plyname)<<"\n";
  std::cout<<w.readPatchInfo(dir)<<"\n";
  std::cout<<w.readCameras(dir,totNimgs)<<"\n";
  
  
  
  /* THIS WORKS!?!?!?!!
  double a;
  pcl::PointXYZ p;
  size_t SIZE(20);
  std::vector<Patch> A(SIZE);
  for (int i=0;i<SIZE;i++){
    p.x = i;
    A.at(i).setPoint(p);
  }
  */
  std::cin.get();
}
