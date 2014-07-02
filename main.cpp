/**
* Dan Goldberg
* 
*/

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <pcl/point_types.h>

#include "aux/patch.h"
#include "aux/cloud.h"

int main() {
  std::string dir("/home/dan/pmvs/");
  std::string plyname("models/option-0000-crop-alpha.ply");
  
  MyCloud c;
  
  std::cout<<c.readPly(dir+plyname)<<"\n";
  std::cout<<c.readPatchInfo(dir)<<"\n";
  std::cout<<c.readCameras(dir)<<"\n";

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
