// Dan Goldberg
// Cloud object
/**
  
*/

#ifndef WORLD_H_
#define WORLD_H_

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>

//#include <pcl/io/point_cloud.h>
//#include <pcl/io/vtk_io.h> 
//#include <pcl/io/vtk_lib_io.h> 
//#include <pcl/io/ply.h> 
#include <pcl/io/ply_io.h> 
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/Vertices.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/TextureMesh.h>

//#include <boost/ptr_container/ptr_vector.hpp>

#include "opencv2/opencv.hpp"

#include "../aux/patch.h"
#include "../aux/camera.h"
#include "../aux/triangle.h"

struct uvl_t {
  Eigen::Vector2f uv;
  int imnum;
};

class World{
public:
  World();
  World(const int& nCams);
//  Cloud(CloudInfo info, std::string name);
  World(const World& c);
  virtual ~World();

  pcl::PointCloud<pcl::PointXYZ>::Ptr getPoints()const{return points_;};
  pcl::KdTreeFLANN<pcl::PointXYZ> getTree()const{return kdtree_;};
  pcl::PolygonMesh getMesh()const{return mesh_;};
  std::vector<Patch> getPatches()const{return patches_;};
  //std::vector<Eigen::MatrixXd> getCameras()const{return cameras_;};
  std::vector<Camera> getCameras()const{return cameras_;};
  std::vector<Triangle<Patch> > getTriangles()const{return triangles_;};
  size_t getN()const{return N_;};
  pcl::TextureMesh getTexMesh()const{return texMesh_;};

  void setNCameras(const int& n){cameras_=std::vector<Camera>(n);};

  bool readPly(const std::string& fname);
  bool readPatchInfo(const std::string& fname);
  bool readCameras(const std::string& fname);
  bool buildTriangles();
  bool mapLocalUV();
  bool mapGlobalUV(const int& imWidth);
  bool makeTextureAtlas();
  bool makeTextureMesh();
  bool writeOBJ();

  cv::Mat createOneImage(const std::vector<cv::Mat>& images,int& imWidth)const;
  int getBestImage(const Triangle<Patch>& t)const;
  int getGoodIndex(const std::vector<int>& inds, std::vector<size_t>& allIndices)const;
  Patch findPatch(const size_t& ind)const;

  void printPct(size_t i, size_t sz)const;
  void printVectorInt(const std::vector<int>& v)const;
  std::string toString()const;
  
/*  friend bool operator==(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
  friend bool operator!=(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);
 */ 
private:
  size_t N_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  pcl::PolygonMesh mesh_;
  std::vector<Patch> patches_;
  std::vector<Triangle<Patch> > triangles_;
  std::vector<Triangle<uvl_t> > uvl_;
  std::vector<Triangle<Eigen::Vector2f> > uvg_;
  std::vector<Camera> cameras_;
  pcl::TextureMesh texMesh_;

};

#endif
