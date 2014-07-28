// Dan Goldberg
// World class

/*
  The world class includes everything that is part of the environment:
    point cloud
    mesh
    kdtree
    vector of all patches
    vector of all cameras
    vector of all triangles
    vector of all UV coords with those triangles
*/

#include "world.h"
/**
  

*/
World::World()
  : N_(),
    kdtree_(),
    mesh_(),
    patches_(),
    triangles_(),
    cameras_(){}
    //texMesh_(){}
/*
Cloud::Cloud(unsigned int N)
  : N_(N),
    points_(new pcl::PointCloud<pcl::PointXYZ>),
    kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>),
    patches_(std::vector<Patch>(N)) {}
*/
World::World(const int& nCams)
  : N_(),
    kdtree_(),
    mesh_(),
    patches_(),
    triangles_(),
    cameras_(std::vector<Camera>(nCams)) {}
    //texMesh_(){}


World::World(const World& c)
  : N_(c.getN()),
    kdtree_(c.getTree()),
    mesh_(c.getMesh()),
    patches_(c.getPatches()),
    triangles_(c.getTriangles()),
    cameras_(c.getCameras()) {}
    //texMesh_(c.getTexMesh()){}

World::~World(){}

// fname is name and path
bool World::readPly(const std::string& fname){
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  points.reset(new pcl::PointCloud<pcl::PointXYZ>(cloud));
  N_ = cloud.points.size();
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (points_);
  kdtree_.setInputCloud(points);
  /*
  std::vector<pcl::Vertices>verts(mesh_.polygons);
  uint32_t num;
  uint32_t min(1000000);
  uint32_t max(0);
  for (size_t i=0; i < verts.size(); i++){
    //std::cout<<verts[i]<<"\n";
    for (int ii=0;ii<3;ii++){
      num = verts[i].vertices[ii];
      if (num<min){
        min=num;
      }
      if (num>max){
        max=num;
      }
    }
  }
  std::cout<<"min: "<<min<<"\n";
  std::cout<<"max: "<<max<<"\n";
  std::cout<<"N: "<<N_<<"\n";*/
  triangles_ = std::vector<Triangle<Patch> >(mesh_.polygons.size());
  uvl_ = std::vector<Triangle<uvl_t> >(mesh_.polygons.size());
  uvg_ = std::vector<Triangle<Eigen::Vector2f> >(mesh_.polygons.size());
  //http://docs.pointclouds.org/trunk/_vertices_8h_source.html

  return true;
}

// fpath is path to (and including) PMVS
bool World::readPatchInfo(const std::string& fpath){
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
  std::vector<size_t> allIndices;
  size_t pointIndex;
 
  // K-D tree search
  int K = 4;
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
  allIndices = std::vector<size_t>(N_);
//  int patchi(0);
  //std::cin.get();
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
        //VERBOSE std::cout<<"ind4 "<<pointIdxNKNSearch[4]<<"\n";
        //VERBOSE std::cout<<p<<" ind: "<<pointIdxNKNSearch[0]<<"\n";
        pointIndex=getGoodIndex(pointIdxNKNSearch,allIndices);
        patches_[pointIndex].setPoint(p);
        patches_[pointIndex].setNormal(n);
        patches_[pointIndex].setNImages(numImg);
        patches_[pointIndex].setInds(inds);
        patches_[pointIndex].setPointInd(pointIndex);
      }
    }
    
    printPct(i,numPatch);
    //if (!(i%100000))std::cout<<((int)(.5+100*(float)i/numPatch))<<"\%\n";
  }
  
  /* VERBOSE
  size_t max(0),wheremax(0),unique(0);
  for (size_t i=0;i<allIndices.size();i++){
    if (allIndices[i]>max){
      max=allIndices[i];
      wheremax=i;
    }
    if (allIndices[i]>0) unique++;
  }
  std::cout<<"Max: "<<max<<"\n";
  std::cout<<"Where: "<<wheremax<<"\n";
  std::cout<<"# Unique: "<<unique<<"\n";
  */
  return true;

} // readPatchInfo


// fpath is path to pmvs
bool World::readCameras(const std::string& fpath){
  std::string name("txt/");
  std::string visname("visualize/");
  /*if (fpath.compare(fpath.size()-1,1,"/")){
    name = name.substr(1,name.size()-1);
  }
  */
  //cameras_ = std::vector<Eigen::MatrixXd>(ncams);
  int ncams(cameras_.size());
  Eigen::MatrixXf mat;
  std::string fname;
  std::string imageFname;
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
    mat = Eigen::MatrixXf(3,4);
    ss2.str("");
    ss2.clear();
    ss2 << i;
    strnum = ss2.str();
    // Building filename
    if (i<10){
      strnum = str0 + strnum;
    }
    fname = fpath + name + str0s + strnum + ".txt";
    imageFname = fpath + visname + str0s + strnum + ".jpg";
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
        while(ss >> (mat(r,c))) c++;
        r++;
        ss.str("");
        ss.clear();
        line.clear();
      }
      cameras_[i].setMat(mat);
      cameras_[i].setFileName(imageFname);
    } else { // if file is open
      return false;
    }
    file.close();
  } // Camera loop 
  return true;
} // readCameras

bool World::buildTriangles(){
 
  std::vector<pcl::Vertices>verts(mesh_.polygons);
  Patch p;
  size_t patchIndex;
  for (size_t i=0; i<verts.size(); i++){
    for (int ii=0;ii<3;ii++){
      patchIndex = verts[i].vertices[ii];
      p = patches_[patchIndex];
      triangles_[i].setv(ii,p);
    }
  }
  
/* 
  std::vector<pcl::Vertices>verts(mesh_.polygons);
  Patch empty;
  Patch p;
  for (size_t i=0; i<verts.size(); i++){
    p = findPatch(verts[i].vertices[0]);
    if (p==empty) {
      std::cout<<"Ind: "<<i<<"\n";
      std::cout<<"looking for ind: " << verts[i].vertices[0]<<"\n";
      return false;
    }
    triangles_[i].setv0(p);
  
    p = findPatch(verts[i].vertices[1]);
    if (p==empty) {
      std::cout<<"Ind: "<<i<<"\n";
      std::cout<<"looking for ind: " << verts[i].vertices[1]<<"\n";
      return false;
    }
    triangles_[i].setv1(p);

    p = findPatch(verts[i].vertices[2]);
    if (p==empty){
      std::cout<<"Ind: "<<i<<"\n";
      std::cout<<"looking for ind: " << verts[i].vertices[2]<<"\n";
      return false;
    }
    triangles_[i].setv2(p);
  }
  
*/
  return true;
}

bool World::mapLocalUV(){
  Eigen::Vector2f uv(Eigen::Vector2f::Zero());
  int imnum(0);
  uvl_t tempuv;
  Camera cam;
  Triangle<Patch> tri;
  Patch pat;
  pcl::PointXYZ p;

  for (size_t i=0;i<triangles_.size();i++){
    imnum = getBestImage(triangles_[i]);
    if (imnum<0){
      std::cout<<"imnum: "<<imnum<<"\n";
      std::cout<<"exiting \n";
      return false;
    }
    cam = cameras_[imnum];
    tri = triangles_[i];
    for (int iv=0;iv<3;iv++){
      pat = tri.getv(iv);
      p = pat.getPoint();
      uv = cam.project(p);
      //uv = cameras_[imnum].project(triangles_[i].getv(iv).getPoint());
      if (uv.any()){
        tempuv.uv = uv;
        tempuv.imnum = imnum;
        uvl_[i].setv(iv,tempuv);
      } else {
        std::cout<<"uv: "<<uv<<"\n";
        std::cout<<"exiting \n";
        return false;
      }
    }
    printPct(i,triangles_.size());
  }
  return true;
}

bool World::makeTextureAtlas(int& imWidth, int& imHeight){
  int numImages(cameras_.size());
  std::vector<cv::Mat> images(numImages);
  std::string fname;
  std::string textureAtlasFName("./texture_atlas.png");
  if (access(textureAtlasFName.c_str(), F_OK) == -1) {
    for (int i=0;i<numImages;++i){
      fname = cameras_[i].getFileName();
      images[i]=cv::imread(fname, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_GRAYSCALE);
    }
    cv::Mat atlas = createOneImage(images, imWidth,imHeight);
    mapGlobalUV(imWidth,imHeight);
    std::vector<int> imwriteParams(CV_IMWRITE_PNG_COMPRESSION,0);
    return cv::imwrite(textureAtlasFName,
                       atlas,
                       imwriteParams );
  } else {
    cv::Mat im = cv::imread(textureAtlasFName);
    imWidth = im.cols;
    imHeight = im.rows;
  }
  return true;
}
/*
bool World::makeTextureMesh(){
  // http://www.pcl-users.org/I-want-to-solve-surface-problem-please-td4028099.html
  // http://pointclouds.org/blog/gsoc/ktran/blog_6_7_obj_io.php
  //pcl::TextureMapping<pcl::PointXYZ> tm;
  //tm.setF(.01);
  //tm.setVectorField(1,0,0);
  
  pcl::TexMaterial texMat;
  texMat.tex_Ka.r = 0.2f;
  texMat.tex_Ka.g = 0.2f;
  texMat.tex_Ka.b = 0.2f;
  texMat.tex_Kd.r = 0.8f;
  texMat.tex_Kd.g = 0.8f;
  texMat.tex_Kd.b = 0.8f;
  texMat.tex_Ks.r = 1.0f;
  texMat.tex_Ks.g = 1.0f;
  texMat.tex_Ks.b = 1.0f;
  texMat.tex_d = 1.0f;
  texMat.tex_Ns = 0.0f;
  texMat.tex_illum = 2;
  tm.setTextureMaterials(texMat);
  
  std::string texFile("./texture_atlas.png");
//  tm.setTextureFiles(texFiles);
  size_t numTriangles(triangles_.size());
  texMesh_.header = mesh_.header;
  texMesh_.cloud = mesh_.cloud;
  std::vector<pcl::Normal> normals(N_);
  for (size_t i=0;i<N_;++i){
    normals[i] = patches_[i].getNormal();
  }
  //texMesh_.tex_normals = normals;

  std::vector<Eigen::Vector2f> texCoord(numTriangles*3);
  std::vector<pcl::Vertices> polygons(numTriangles);
  pcl::Vertices vert;
  Triangle<Patch> t;
  Patch p;
  for (size_t triInd=0;triInd<numTriangles;++triInd){
    for (int vertInd=0;vertInd<3;++vertInd){
//      t = triangles_[triInd];
//      p = t.getv(vertInd);
//      std::cout<<p<<std::endl;
//      std::cout<<p.getPointInd()<<std::endl;
//      vert.vertices[vertInd] = p.getPointInd();
      texCoord[triInd*3+vertInd] = uvg_[triInd].getv(vertInd);
    }
//    polygons[triInd] = vert;
  }
//  texMesh_.tex_polygons.push_back( mesh_.polygons );
  texMesh_.tex_polygons = mesh_.polygons;

  texMat.tex_name = "material_0";
  texMat.tex_file = texFile;
  //texMesh_.tex_materials.push_back(texMat);
  texMesh_.tex_materials = texMat;
  
  //texMesh_.tex_coordinates.push_back(texCoord);
  texMesh_.tex_coordinates = texCoord;
  
  return true;
}
*/
bool World::writeOBJ(const std::string& filename)const{
  //int status = pcl::io::saveOBJFile("./out.obj", texMesh_);
  int start = filename.find_last_of('/');
  int end = filename.find_last_of('.');
  int len = end - start;

  std::string base_name = filename.substr(start+1, len-1);
  std::string mtl_name = filename.substr(0, end) + ".mtl";

  std::ofstream output(filename.c_str());
  std::ofstream mtl(mtl_name.c_str());
  
  std::string texFile("./texture_atlas.png");

  if(!output) {
      std::cerr << "Can't open " << filename << " for writing" << std::endl;
      return false;
  }

  if(!mtl) {
      std::cerr << "Can't open " << mtl_name << " for writing" << std::endl;
      return false;
  }

  std::cout << "Saving " << filename << " ..." << std::endl;
  std::cout << "Saving " << mtl_name << " ..." << std::endl;

  // MTL file
  mtl << "newmtl Texture_0" << std::endl;
  mtl << "Ka 1 1 1" << std::endl;
  mtl << "Kd 1 1 1" << std::endl;
  mtl << "Ks 0 0 0" << std::endl;
  mtl << "d 1.0" << std::endl;
  mtl << "illum 2" << std::endl;
  mtl << "map_Kd " << texFile << std::endl;
  mtl << std::endl;
  
  // Writing OBJ file
  output << "mtllib " << base_name << ".mtl" << std::endl;
  
  pcl::PointXYZ point;

  // Write vertices
  for(size_t i=0; i < patches_.size(); i++) {
    point = patches_[i].getPoint();
    output << "v " << point.x << " " << point.y << " " << point.z << std::endl;
  }

  // Write texture coordinates, ordered by camera and face
  Eigen::Vector2f uv;
  for(size_t triInd=0; triInd < triangles_.size(); triInd++) {
    for(int vInd=0; vInd < 3; vInd ++) {
      uv = uvg_[triInd].getv(vInd);
      output << "vt " << uv(0) << " " << uv(1)  << std::endl;
    }
  }

  // Write face
  int idx=0;
  Patch p;
  output << "usemtl Texture_" << 0 << endl;
  for(size_t triInd=0; triInd < triangles_.size(); triInd++) {

    output << "f ";
    for (int i=0;i<3;i++){
      p = triangles_[triInd].getv(i);
      output << (p.getPointInd()+1) << "/" << (idx + 1);
      if (i<2) output<<" ";
    }
    output << std::endl;
    idx += 3;
  }
  return true;
}

bool World::writeOBJ()const{
  
  return writeOBJ("out.obj");
}

bool World::mapGlobalUV(const int& imWidth,const int& imHeight){
  uvg_ = std::vector<Triangle<Eigen::Vector2f> >(uvl_.size());
  Triangle<uvl_t> localUVtri;
  Triangle<Eigen::Vector2f> uvTri;
  int numCameras = cameras_.size();
  Eigen::Vector2f uv;
  int xoffset;
  for (size_t triInd=0;triInd<uvl_.size();++triInd){
    localUVtri = uvl_[triInd];
    for (size_t vInd=0;vInd<3;++vInd){
      xoffset = localUVtri.getv(vInd).imnum * imWidth;
      uv(0) = localUVtri.getv(vInd).uv(0)+xoffset;
      uv(1) = localUVtri.getv(vInd).uv(1);
      uv(0) /= imWidth*numCameras;
      uv(1) /= imHeight;
      uvTri.setv( vInd,uv );
    }
    uvg_[triInd] = uvTri;
  }
  
  return true;
}

cv::Mat World::createOneImage(const std::vector<cv::Mat>& images,int& imWidth, int& imHeight)const{
// Inspired by http://answers.opencv.org/question/13876/read-multiple-images-from-folder-and-concat/
  
  int numImages(images.size());
  int width(images[0].cols);
  int height(images[0].rows);
  
  cv::Mat result(cv::Mat::zeros(height, numImages*width, images[0].type()));
  
  cv::Mat roiInResult;
  int curWidth(0);
  for (int i=0;i<numImages;++i){
    /* VERBOSE
    std::cout<<"\nImage: "<<i<<std::endl;
    std::cout<<"Height: "<<images[i].rows<<std::endl;
    std::cout<<"Width: "<<images[i].cols<<std::endl;
    std::cout<<"Type: "<<CV_MAT_TYPE(images[i].type())<<std::endl;
    */
    if (images[i].cols != width){
      std::cerr<<"Warning: createOneImage fail, images are different widths\n";
      return result;
    }
    if (images[i].rows != height){
      std::cerr<<"Warning: createOneImage fail, images are different heights\n";
      return result;
    }
    roiInResult = cv::Mat(result,
                          cv::Range(0,height),
                          cv::Range(curWidth,curWidth + width));
    images[i].copyTo(roiInResult);
    curWidth += width;
  }
  imWidth = width;
  imHeight = height;
  return result;
}

int World::getBestImage(const Triangle<Patch>& t)const{
  std::vector<int> vim;
  std::vector<int> allim;
  std::vector<int> newallim;
  int bestim;
  allim = t.getv(0).getInds();
  for (int i=0;i<3;i++){
    vim = t.getv(i).getInds();
    for (size_t ii=0;ii<vim.size();ii++){
      if (std::find(allim.begin(), allim.end(), vim[ii]) == allim.end()) {
        allim.push_back(vim[ii]);
      }
    }
  }
  int count(0);
  int max(0);
  int wheremax(-1);
  std::vector<int> counts = std::vector<int>(allim.size());
  for (size_t i=0;i<allim.size();i++){
    count = 0;
    for (size_t ii=0;ii<3;ii++){
      vim = t.getv(ii).getInds();
      for (size_t iii=0;iii<vim.size();iii++){
        if (vim[iii]==allim[i]){
          count++;
        }
      
      }
    }
    if (count>max){
      max = count;
      wheremax = i;
    }
    counts[i]=count;
  }
  
  /*
  allim = newallim;
  newallim = std::vector<int>();

  if (allim.empty()){
    bestim = -1;
  } else {
    bestim = allim[0];
  }
  */
  bestim = allim[wheremax];
  return bestim;
}

int World::getGoodIndex(const std::vector<int>& inds, std::vector<size_t>& allIndices)const{
  
  int ind;
  for (size_t i=0;i<inds.size();++i){
    ind = inds[i];
    if (!(allIndices[ind]>0)){
      allIndices[ind]++;
      return ind;
    }
  }
  return -1;
}

Patch World::findPatch(const size_t& ind)const{
  return patches_[ind];
/*  
  for (size_t i=0;i<patches_.size();i++){
    if (patches_[i]==ind){
      return patches_[i];
    }
  }
  return Patch();
*/
}

/*bool operator==(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
  return ( (p1.x==p2.x) && (p1.y==p2.y) && p1.z==p2.z);
}

bool operator!=(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
  return ( (p1.x!=p2.x) || (p1.y!=p2.y) || p1.z!=p2.z);
}*/


void World::printPct(size_t i, size_t sz)const {
  if (!(i%((int)sz/20))){
     std::cout<<(int)(.5+100*(float)i/sz)<<"%\n";
  }
}

void World::printVectorInt(const std::vector<int>& v)const{
  std::cout<<"[";
  for (int i=0;i<(v.size()-1);i++){
    std::cout<<v[i]<<", ";
  }
  std::cout<<v[v.size()-1]<<"]\n";
}
