// Copyright 2011, Digital Imaging and Remote Sensing Laboratory
// All Rights Reserverd.
// Written by Philip Salvaggio (pss7119@rit.edu)

#include "ply_writer.h"

#include <vector>
#include "framework/types/point_cloud/point_cloud.h"

namespace dpf_io {

using dpf_types::Point;
using dpf_types::PointCloud;
using std::vector;

PlyWriter::PlyWriter() : pts_(), tris_() {}
PlyWriter::~PlyWriter() {}

void PlyWriter::AddPoints(const PointCloud& pt_cloud) {
  for (size_t i = 0; i < pt_cloud.Size(); i++) {
    Point p;
    pts_.push_back(p);
    pt_cloud.at(i, &(pts_[pts_.size() - 1]));
  }
}

void PlyWriter::AddTriangles(const vector<Point>& pts,
                             const vector<size_t>& tris) {
  int offset = pts_.size();
  pts_.insert(pts_.end(), pts.begin(), pts.end());
  for (size_t i = 0; i < tris.size(); i++) {
    tris_.push_back(tris[i] + offset);
  }
}
   
bool PlyWriter::Write(const std::string& filename) {
  FILE* file = fopen(filename.c_str(), "w");
  if (!file) return false;

  fprintf(file, "ply\n"
                "format ascii 1.0\n"
                "element vertex %lu\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "element face %lu\n"
                "property list uchar int vertex_index\n"
                "end_header\n", pts_.size(), tris_.size()/3);

  for (size_t i = 0; i < pts_.size(); i++) {
    fprintf(file, "%f %f %f\n", pts_[i][0], pts_[i][1], pts_[i][2]);
  }
  for (size_t i = 0; i < tris_.size(); i += 3) {
    fprintf(file, "3 %lu %lu %lu\n", tris_[i], tris_[i+1], tris_[i+2]);
  }

  fclose(file);
  return true;
}

}  // namespace dpf_io
