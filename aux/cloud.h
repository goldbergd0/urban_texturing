// Dan Goldberg
// Cloud object
/**
  Reads and writes PLY files
  Can add triangles and UV coords
  
*/

#ifndef CLOUD_H_
#define CLOUD_H_

class Cloud{
public:
  Cloud();
  Cloud(CloudInfo info);
  virtual ~Cloud();

  void setCloudInfo();
  void plyWriteHeader();
  void plyWriteData();
  


private:
  CloudInfo info_;

}

#endif
