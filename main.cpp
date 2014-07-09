/**
* Dan Goldberg
* 
*/

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <time.h>

#include <Eigen/Dense>

#include <pcl/point_types.h>

#include "opencv2/highgui/highgui.hpp"

#include "aux/patch.h"
#include "world/world.h"

int main() {
  clock_t t1,t2;
  t1 = clock();
  std::string dir("/home/dan/pmvs/");
  std::string plyname("models/option-0000-crop-alpha.ply");
  
  int totNimgs(48);
  
  World w(totNimgs);
  
  std::cout <<"Reading PLY: ";
  std::cout <<w.readPly(dir+plyname)<<"\n";
  std::cout <<"Reading Cameras: ";
  std::cout << w.readCameras(dir)<<"\n";
  std::cout <<"Reading Patch file: ";
  std::cout << w.readPatchInfo(dir)<<"\n";
  std::cout <<"Buildling Triangles: ";
  std::cout << w.buildTriangles()<<"\n";
  std::cout <<"Map UV: ";
  std::cout << w.mapUV()<<"\n";

  /*http://opencv-srf.blogspot.com/2013/06/load-display-image.html
  cv::Mat img = cv::imread("/home/dan/pmvs/visualize/00000000.jpg",CV_LOAD_IMAGE_UNCHANGED);
  
  if (img.empty()){
    std::cout<<"err\n";
    return -1;
  }
  cv::namedWindow("MyWindow",CV_WINDOW_AUTOSIZE);
  cv::imshow("MyWindow",img);

  cv::waitKey(0);

  cv::destroyWindow("MyWindow");
  */
  t2 = clock();
  int time = (t2-t1)/CLOCKS_PER_SEC;
  int minutes(time/60);
  int seconds(time%60);
  std::cout<<"Elapsed Time: "<<minutes<<" minutes and "<<seconds<< " seconds\n";
  
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
  //std::cin.get();
}
