// Dan Goldberg
// Cloud object
/**
  Reads and writes PLY files
  Can add triangles and UV coords
  
*/

#ifndef CLOUD_H_
#define CLOUD_H_

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>

#include "../aux/cloudinfo.h"
#include "../aux/patchinfo.h"

class Cloud{
public:
  Cloud();
  Cloud(CloudInfo info, std::string name);
  Cloud(std::string name, CloudInfo info);
  virtual ~Cloud();

  bool readPly(const std::string& fname)const;
  bool readPatches(const std::string& fname)const;
  bool readCameras(const std::string& fname)const;
  
  bool plyWriteHeader(const std::string& fname)const;
  bool plyWriteData(const std::string& fname)const;
  bool plyWrite(const std::string& fname)const;

private:
  unsigned int N_;
  pcl::PointCloud<pcl::pointXYZ> points_;
  pcl::PolygonMesh mesh_;
  std::vector<Patch> patches_;
  std::vector<Eigen::MatrixXd(3,4)> cameras_;

}

#endif
