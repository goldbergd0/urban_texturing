#include "patchinfo.h"

PatchInfo::PatchInfo(const unsigned int& n)
  : nVerts_(n),patches_(std::vector<Patch>(n)){}
