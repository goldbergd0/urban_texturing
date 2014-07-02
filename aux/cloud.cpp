// Dan Goldberg
// Cloud class

#include "cloud.h"

MyCloud::MyCloud()
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
MyCloud::MyCloud(const MyCloud& c)
  : N_(c.getN()),
    points_(c.getPoints()),
    kdtree_(c.getTree()),
    mesh_(c.getMesh()),
    patches_(c.getPatches()),
    cameras_(c.getCameras())
    {}

MyCloud::~MyCloud(){
  }

// fname is name and path
bool MyCloud::readPly(const std::string& fname){
  /*
  Example from http://www.pcl-users.org/Registering-PolygonMeshes-td4025472.html
  
    pcl::PolygonMesh mesh; 
    pcl::PointCloud<PointT> point_cloud; 
    pcl::io::loadPolygonFilePLY("filename.ply", mesh); 
    pcl::fromROSMsg(mesh.cloud, point_cloud); 
    // Do registration manipulations to point_cloud 
    pcl::toROSMsg(point_cloud, mesh.cloud); 
  */
  //pcl::PLYReader reader;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //return (reader.read(fname,*cloud)<0);
  
  int status = pcl::io::loadPolygonFilePLY(fname,mesh_);
  if (status==-1){
    return false;
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh_.cloud,cloud);
  points_.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud));
  N_ = cloud.points.size();
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (points_);
  kdtree_.setInputCloud(points_);
  
  
  return true;
}

// fpath is path to (and including) PMVS
bool MyCloud::readPatchInfo(const std::string& fpath){
  std::string name("/models/option-0000.patch");
  if (fpath.compare(fpath.size()-1,1,"/")){
    name = name.substr(1,name.size()-1);
  }
  std::string fname(fpath+name);
  // Variables to be read in
  Patch patch;
  pcl::PointXYZ p;
  pcl::Normal n;
  size_t numImg;
  int ind;
  std::vector<int> inds; 
  size_t numPatch;
  std::string contents;
  std::istringstream sswhole;
  std::vector<size_t> badPatch;
 
  // K-D tree search
  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
       
  ////////// READ FILE ///////////
  std::ifstream file(fname.c_str());
  if (file){
    file.seekg(0,std::ios::end);
    contents.resize(file.tellg());
    file.seekg(0,std::ios::beg);
    file.read(&contents[0],contents.size());
    if (!(file)) return false;
    file.close();
  } else return false;
  sswhole.str(contents);
  
  //size_t loc(0);
  std::string line;
  
  //line = grabline(loc,contents);
  std::getline(sswhole,line);
  if (line.compare(0,7,"PATCHES")!=0){
      return false;
  }
  std::getline(sswhole,line);
  std::istringstream ss(line);
  ss >> numPatch;
  ss.str("");
  ss.clear();
  patches_ = std::vector<Patch>(N_);
  int patchi(0);
  std::cin.get();
  for (size_t i=0;i<numPatch;i++){
    std::getline(sswhole,line); // PATCHS
    //std::cout<<line<<"\n";
    std::getline(sswhole,line); // X Y Z 1
    ss.str(line);
    //  pcl::PointXYZ pt; 
    //  pt.getVector3fMap() = anotherVec3f; 
    ss >> p.x;
    ss >> p.y;
    ss >> p.z;
    ss.str("");
    ss.clear();
    std::getline(sswhole,line); // Nx Ny Nz 0
    ss.str(line);
    ss >> n.normal_x;
    ss >> n.normal_y;
    ss >> n.normal_z;
    ss.str("");
    ss.clear();
    std::getline(sswhole,line); // goodness debug debug
    std::getline(sswhole,line); // N (images point visible in)
    ss.str(line);
    ss >> numImg;
    ss.str("");
    ss.clear();
    std::getline(sswhole,line); // N number of image indices
    ss.str(line);
    // from 
    // http://stackoverflow.com/questions/455483/c-going-from-string-to-stringstream-to-vectorint
    //      inds = (istream_iterator<unsigned int>(ss)),
    //      istream_iterator<unsigned int>();

    // THISSSSSSS!!!!
    inds = std::vector<int>();
    while (ss >> ind) inds.push_back(ind);
    ss.str("");
    ss.clear();
    std::getline(sswhole,line); // N2 (textures don't agree well)
    std::getline(sswhole,line); // N2 number of image indices
    std::getline(sswhole,line); // [EMPTY]
    
    if ( kdtree_.nearestKSearch( p, K, pointIdxNKNSearch, pointNKNSquaredDistance) ) {
      //if( patch == points_->points.at( pointIdxNKNSearch[0] ) ){
      if (pointNKNSquaredDistance[0] < 0.00000001){
        patches_[patchi].setPoint(p);
        patches_[patchi].setNormal(n);
        patches_[patchi].setNImages(numImg);
        patches_[patchi].setInds(inds);
        patches_[patchi].setPointInd(pointIdxNKNSearch[0]);
        patchi++;
        // VERBOSE std::cout<<patchi<<": "<<p<<"\n";
      }
    }
    
    // VERBOSE if (!(i%100000))std::cout<<((int)(.5+100*(float)i/(float)numPatch))<<"\%\n";
  }
  return true;

} // readPatchInfo


// fpath is path to pmvs
bool MyCloud::readCameras(const std::string& fpath){
  std::string name("txt/");
  /*if (fpath.compare(fpath.size()-1,1,"/")){
    name = name.substr(1,name.size()-1);
  }
  */
  int ncams = 48;
  Eigen::MatrixXd camera;
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
    camera = Eigen::MatrixXd(3,4);
    ss2.str("");
    ss2.clear();
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
        while(ss >> (camera(r,c))) c++;
        r++;
        ss.str("");
        ss.clear();
        line.clear();
      }
      cameras_.push_back(camera);
    } else { // if file is open
      return false;
    }
    file.close();
  } // Camera loop (0-47)
  return true;
} // readCameras

std::string MyCloud::toString()const{
  std::string outStr("My Cloud Object: ");
  std::string tab("\t");
  std::string newl("\n");
  //std::string cloudsz(points_->points.size());
   
  
  
  
  return outStr;
}

bool operator==(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
  return ( (p1.x==p2.x) && (p1.y==p2.y) && p1.z==p2.z);
}

bool operator!=(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
  return ( (p1.x!=p2.x) || (p1.y!=p2.y) || p1.z!=p2.z);
}
