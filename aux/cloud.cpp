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

bool Cloud::plyWriteHeader();
bool Cloud::plyWriteData();
bool Cloud::plyWtite();
