// Dan Goldberg
// Cloud object
/**
  Reads and writes PLY files
  Can add triangles and UV coords
  
*/

#ifndef CLOUD_H_
#define CLOUD_H_

#include <string>

class Cloud{
public:
  Cloud();
  Cloud(CloudInfo info, std::string name);
  Cloud(std::string name, CloudInfo info);
  virtual ~Cloud();

  void setCloudInfo(const CloudInfo& info){info_=info;};
  void setName(const std::string name){name_=name;};
  bool plyWriteHeader();
  bool plyWriteData();
  bool plyWrite();
  


private:
  CloudInfo info_;
  std::string name_;

}

#endif
