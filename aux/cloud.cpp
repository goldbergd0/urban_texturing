// Dan Goldberg
// Cloud class

#include "cloud.h"

Cloud::Cloud()
  : N_(),
    points_(),
    kdtree_(),
    mesh_(),
    patches_(),
    cameras_(){}
/*
Cloud::Cloud(unsigned int N)
  : N_(N),
    points_(new pcl::PointCloud<pcl::PointXYZ>),
    kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>),
    patches_(std::vector<Patch>(N)) {}
*/
Cloud::Cloud(const Cloud& c)
  : N_(c.getN()),
    points_(c.getPoints()),
    kdtree_(c.getTree()),
    mesh_(c.getMesh()),
    patches_(c.getPatches()),
    cameras_(c.getCameras())
    {}

Cloud::~Cloud(){}

// fname is name and path
bool Cloud::readPly(const std::string& fname){
  /*
  Example from http://www.pcl-users.org/Registering-PolygonMeshes-td4025472.html
    pcl::PolygonMesh mesh; 
    pcl::PointCloud<PointT> point_cloud; 
    pcl::io::loadPolygonFilePLY("filename.ply", mesh); 
    pcl::fromROSMsg(mesh.cloud, point_cloud); 
    // Do registration manipulations to point_cloud 
    pcl::toROSMsg(point_cloud, mesh.cloud); 
  */
  pcl::PLYReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  return (reader.read(fname,*cloud)<0);
  /*int status = pcl::io::loadPolygonFilePLY(fname,mesh_);
  if (status==-1){
    return false;
  }
  pcl::fromROSMsg(mesh_.cloud,points_);
  N_ = points_.points.size();
  kdtree.setInputCloud(points_);
  */
  
  return true;
}

// fpath is path to (and including) PMVS
bool Cloud::readPatchInfo(const std::string& fpath){
  std::string name("/models/option-0000.patch");
  if (fpath.compare(fpath.size()-1,1,"/")){
    name = name.substr(1,name.size()-1);
  }
  std::string fname(fpath+name);
  // Variables to be read in
  pcl::PointXYZ p;
  pcl::Normal n;
  size_t num;
  int ind;
  std::vector<int> inds; 
 
  // K-D tree search
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
       
  ////////// READ FILE ///////////
  std::string line;
  std::ifstream file;
  file.open(fname.c_str());
  size_t NUM;
  if (file.is_open()){
    getline(file,line);
    if (line.compare(0,7,"PATCHES")!=0){
      return false;
    }
    line.clear();
    getline(file,line);
    std::istringstream ss(line);
    ss >> NUM;
    ss.clear();
    line.clear();
    while (NUM>0){
      Patch patch;
      getline(file,line); // PATCHS
      line.clear();
      getline(file,line); // X Y Z 1
      ss.str(line);
      //  pcl::PointXYZ pt; 
      //  pt.getVector3fMap() = anotherVec3f; 
      ss >> p.x;
      ss >> p.y;
      ss >> p.z;
      ss.clear();
      line.clear();
      getline(file,line); // Nx Ny Nz 0
      ss.str(line);
      ss >> n.normal_x;
      ss >> n.normal_y;
      ss >> n.normal_z;
      ss.clear();
      line.clear();
      getline(file,line); // goodness debug debug
      line.clear();
      getline(file,line); // N (images point visible in)
      ss.str(line);
      ss >> num;
      ss.clear();
      line.clear();
      getline(file,line); // N number of image indices
      ss.str(line);
      // from 
      // http://stackoverflow.com/questions/455483/c-going-from-string-to-stringstream-to-vectorint
      //      inds = (istream_iterator<unsigned int>(ss)),
      //      istream_iterator<unsigned int>();
      while (ss >> ind) inds.push_back(ind);
      ss.clear();
      line.clear();
      getline(file,line); // N2 (textures don't agree well)
      line.clear();
      getline(file,line); // N2 number of image indices
      line.clear();
      getline(file,line); // [EMPTY]
      line.clear();
      
      patch.setPoint(p);
      patch.setNormal(n);
      patch.setNImages(num);
      patch.setInds(inds);

      if ( kdtree_.nearestKSearch( p, K, pointIdxNKNSearch, pointNKNSquaredDistance) ) {
        if( (patch) == points_[ pointIdxNKNSearch[0] ] ){
          patch.setPointInd(pointIdxNKNSearch[0]);
          patches_.push_back(patch);
        }
      }
      
      NUM--;
    }
    file.close();
    return true;
  }
  return false;
} // readPatchInfo


// fpath is path to pmvs
bool Cloud::readCameras(const std::string& fpath){
  std::string name("/txt");
  if (fpath.compare(fpath.size()-1,1,"/")){
    name = name.substr(1,name.size()-1);
  }
  
  int ncams = 48;
  Eigen::MatrixXd camera(3,4);
  std::string fname;
  std::string strnum;
  std::string str0s = "000000";
  std::string str0 = "0";
  int r,c;
  
  // File reading stuff
  std::string line;
  std::ifstream file;
  std::istringstream ss(line);
  std::stringstream ss2;
  for (int i=0;i<ncams;++i){
    ss2 << i;
    strnum = ss2.str();
    // Building filename
    if (i<10){
      strnum = str0 + strnum;
    }
    fname = fpath + name + str0s + strnum + ".txt";
    // Reading file
    file.open(fname.c_str());
    if (file.is_open()){
      getline(file,line); // CONTOUR
      line.clear();
      // Read file into matrix
      r = 0;
      while(getline(file,line)){
        ss.str(line);
        c = 0;
        while(ss >> camera(r,c)) c++;
        r++;
        ss.clear();
        line.clear();
      }
      cameras_.at(i) = camera;
    } else { // if file is open
      return false;
    }
    file.close();
  } // Camera loop (0-47)
  return true;
} // readCameras
