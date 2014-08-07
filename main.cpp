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
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"

#include "aux/camera.h"
#include "world/world.h"

int main() {
  clock_t t1,t2;
  t1 = clock();
  std::string dir("/home/dan/pmvs/");
  //std::string plyname("models/mesh.ply");
 
  int imHeight,imWidth; 
  int totNimgs(48);
  
  World w(totNimgs);
  
  std::cout <<"Reading PLY: ";
  std::cout <<w.readPly(dir)<<std::endl;
  std::cout <<"Reading Cameras: ";
  std::cout << w.readCameras(dir)<<std::endl;
  std::cout <<"Reading Patch file: ";
  std::cout << w.readPatchInfo(dir)<<std::endl;
  std::cout <<"Buildling Triangles: ";
  std::cout << w.buildTriangles()<<std::endl;
  std::cout <<"Map Local UV: ";
  std::cout << w.mapLocalUV()<<std::endl;
  std::cout <<"Make Texture Atlas: ";
  std::cout << w.makeTextureAtlas(imWidth,imHeight) <<std::endl;
  std::cout <<"Map Global UV: ";
  std::cout << w.mapGlobalUV(imWidth,imHeight)<<std::endl;
//  std::cout <<"Make Texture Mesh: ";
//  std::cout << w.makeTextureMesh()<<std::endl;
  std::cout <<"Write obj File: ";
  std::cout << w.writeOBJ()<<std::endl;

/*  
  std::vector<Camera> cams = w.getCameras();
  for (size_t i = 0; i<cams.size();++i){
    std::cout<<"Camera "<<i<<":"<<std::endl;
    std::cout<<cams[i].getMat()<<std::endl;
  }

Triangle #48
Points: 
(0.0537752,-0.576609,-10.2631)
(0.0544958,-0.578752,-10.2644)
(0.051931,-0.584528,-10.2638)
Im num: 1

  pcl::PointXYZ p1(0.0537752,-0.576609,-10.2631);
  pcl::PointXYZ p2(0.0544958,-0.578752,-10.2644);
  pcl::PointXYZ p3(0.051931,-0.584528,-10.2638);
  Camera c((w.getCameras())[1]);
  c.project(p1);
  c.project(p2);
  c.project(p3);
*/

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
