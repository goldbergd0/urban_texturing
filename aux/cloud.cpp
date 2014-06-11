// Dan Goldberg
// Cloud class

#include "cloud.h"

Cloud::Cloud()
  : N_(),points_(new pcl::PointCloud<pcl::PointXYZ>),mesh_(),patches_(),cameras_() {}

Cloud::Cloud(unsigned int N)
  : N_(N),points_(new pcl::PointCloud<pcl::PointXYZ>),patches(std::vector<Patch>(N)) {}

Cloud::Cloud(const Cloud& c)
  : N_(c.getN()),
    points_(c.getPoints()),
    patches_(c.getPatches()),
    cameras_(c.getCameras())
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
  unsigned int NUM
  if (file.is_open()){
    getline(file,line);
    if (line.compare(0,7,"PATCHES")!=0){
      return false;
    }
    getline(file,line);
    std::istringstream ss(line);
    ss >> NUM;
    ss.clear();
    while (NUM>0){
      getline(file,line); // PATCHS
      getline(file,line); // X Y Z 1
      ss.str(line);
      //  pcl::PointXYZ pt; 
      //  pt.getVector3fMap() = anotherVec3f; 
      ss >> p.x;
      ss >> p.y;
      ss >> p.z;
      ss.clear();
      getline(file,line); // Nx Ny Nz 0
      ss.str(line);
      ss >> n.normal_x;
      ss >> n.normal_y;
      ss >> n.normal_z;
      ss.clear();
      getline(file,line); // goodness debug debug
      getline(file,line); // N (images point visible in)
      ss.str(line);
      ss >> num;
      getline(file,line); // N number of image indices
      getline(file,line); // N2 (textures don't agree well)
      getline(file,line); // N2 number of image indices
      getline(file,line); // [EMPTY]
      
      NUM--;
    }
    return true;
  }
  return false;
}

bool Cloud::plyWriteHeader();
bool Cloud::plyWriteData();
bool Cloud::plyWtite();
