// Dan Goldberg
// Cloud class

#include "cloud.h"

Cloud::Cloud()
  : info_(CloudInfo()), name_(std::string()) {}

Cloud::Cloud(CloudInfo info, std::string name);
  : info_(info), name_(name) {}

Cloud::Cloud(std::string name, CloudInfo info);
  : info_(info), name_(name) {}

Cloud::~Cloud(){}

bool readPatchInfo(const std::string& fname)const{
  unsigned int num;
  int numimages;
  int* ims;
  Patch* patch = new Patch[num];
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
    PatchInfo p(num);
    
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
