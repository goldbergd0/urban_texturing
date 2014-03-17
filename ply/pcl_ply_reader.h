// Copyright 2011, Digital Imaging and Remote Sensing Laboratory
// All Rights Reserverd.
// Written by Philip Salvaggio (pss7119@rit.edu)

#ifndef FRAMEWORK_IO_PLY_PCL_PLY_READER_H_
#define FRAMEWORK_IO_PLY_PCL_PLY_READER_H_

#include <string>
#include "framework/types/point_cloud/point_cloud.h"

namespace dpf_io {

class PclPlyReader {
 public:
  PclPlyReader();
  virtual ~PclPlyReader();

  virtual dpf_types::PointCloud* Read(const std::string& filename);
};

}  // namespace dpf_io

#endif  // FRAMEWORK_IO_PLY_PCL_PLY_READER_H_
