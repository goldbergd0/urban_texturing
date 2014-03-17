// Copyright 2011, Digital Imaging and Remote Sensing Laboratory
// All Rights Reserverd.
// Written by Philip Salvaggio (pss7119@rit.edu)

#include "pcl_ply_reader.h"

#include "framework/types/point_cloud/point_cloud.h"
#include "framework/types/point_cloud/pcl_point_cloud_impl.h"

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

namespace dpf_io {

using dpf_types::PointCloud;

PclPlyReader::PclPlyReader() {}
PclPlyReader::~PclPlyReader() {}

PointCloud* PclPlyReader::Read(const std::string& filename) {
  pcl::PointCloud<pcl::PointXYZ>* pts = new pcl::PointCloud<pcl::PointXYZ>();
  pcl::PLYReader reader;
  int status = reader.read(filename, *pts);

  if (status != 0) {
    delete pts;
    return NULL;
  }

  PointCloud* pt_cloud = new PointCloud(); 
  pt_cloud->set_impl(new dpf_types::PclPointCloud3DImpl(pts));
  return pt_cloud;
}

}  // namespace dpf_io
