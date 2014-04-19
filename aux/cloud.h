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
#include "../aux/cloudinfo.h"
#include "../aux/patchinfo.h"

class Cloud{
public:
  Cloud();
  Cloud(CloudInfo info, std::string name);
  Cloud(std::string name, CloudInfo info);
  virtual ~Cloud();

  void setCloudInfo(const CloudInfo& info){points_=info;};
  void setPatchInfo(const PatchInfo& info){patches_=info;};
  
  bool readPatchInfo(const std::string& fname)const;
  bool readCameraInfo(const std::string& fname)const;
  

  bool plyWriteHeader(const std::string& fname)const;
  bool plyWriteData(const std::string& fname)const;
  bool plyWrite(const std::string& fname)const;

private:
  CloudInfo points_;
  PatchInfo patches_;
  std::vector<Eigen::MatrixXd(3,4)> cameras_;

}

#endif
