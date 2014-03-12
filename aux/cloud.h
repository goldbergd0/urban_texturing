// Dan Goldberg
// Cloud object
/**
  Reads and writes PLY files
  Can add triangles and UV coords
  
*/

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
