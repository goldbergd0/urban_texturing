// Copyright 2011, Digital Imaging and Remote Sensing Laboratory
// All Rights Reserverd.
// Written by Philip Salvaggio (pss7119@rit.edu)

#ifndef FRAMEWORK_IO_POINT_CLOUD_PLY_PLY_WRITER
#define FRAMEWORK_IO_POINT_CLOUD_PLY_PLY_WRITER

#include <string>
#include <vector>

#include "framework/types/point/point.h"

namespace dpf_types {
class PointCloud;
}

namespace dpf_io {

class PlyWriter {
 public:
  PlyWriter();
  virtual ~PlyWriter();

  void AddPoints(const dpf_types::PointCloud& pt_cloud);

  void AddTriangles(const std::vector<dpf_types::Point>& pts,
                    const std::vector<size_t>& tris);
   
  bool Write(const std::string& filename);

 protected:
  std::vector<dpf_types::Point> pts_;
  std::vector<size_t> tris_;
  
};

}  // namespace dpf_io

#endif  // FRAMEWORK_POINT_CLOUD_POINT_CLOUD_READER_H_
