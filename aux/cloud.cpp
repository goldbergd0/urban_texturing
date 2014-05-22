// Dan Goldberg
// Cloud class

#include "cloud.h"

Cloud::Cloud()
  : N_(),points_(new pcl::PointCloud<pcl::PointXYZ>),mesh_(),patches_(),cameras_() {}

Cloud::Cloud(unsigned int N)
  : N_(N),points_(new pcl::PointCloud<pcl::PointXYZ>),patches(std::vector<Patch>(N)) {}

Cloud::Cloud(const Cloud& c)
  : N_(c.getN()),points_(c.getPoints()),patches_(c.getPatches()),cameras_(c.getCameras())
  {}

Cloud::~Cloud(){}

bool Cloud::readPly(const std::string& fname)const{
  /*
  Example from http://www.pcl-users.org/Registering-PolygonMeshes-td4025472.html
    pcl::PolygonMesh mesh; 
    pcl::PointCloud<PointT> point_cloud; 
    pcl::io::loadPolygonFilePLY("filename.ply", mesh); 
    pcl::fromROSMsg(mesh.cloud, point_cloud); 
    // Do registration manipulations to point_cloud 
    pcl::toROSMsg(point_cloud, mesh.cloud); 
  */
  int status = pcl::io::loadPolygonFilePLY(fname,mesh_);
  if (status==-1){
    return false;
  }
  pcl::fromROSMsg(mesh_.cloud,points_);
  N_ = (unsigned int) points_.points.size();
  return true;
}

bool Cloud::readPatchInfo(const std::string& fname)const{
  pcl::PointXYZ p;
  pcl::Normal n;
  unsigned int num;
  std::vector<unsigned int> top3; 
  Patch patch;
  
  ////////// READ FILE ///////////
  std::string line;
  std::ifstream file;
  file.open(fname);
  if (file.is_open()){
    getline(file,line);
    if (line.compare(0,7,"PATCHES")!=0){
      return false;
    }
    getline(file,line);
    std::istringstream ss(line);
    ss >> num;
    
    getline(file,line);
    while (true){
      
    }
    return true;
  }
  return false;
}

bool Cloud::plyWriteHeader();
bool Cloud::plyWriteData();
bool Cloud::plyWtite();
