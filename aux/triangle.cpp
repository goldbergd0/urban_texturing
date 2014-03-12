// Dan Goldberg
// Triangle Face class for 3D point clouds

#include "triangle.h"

Triangle::Triangle(){
  face_ = new Eigen::Vector3f[3];
}

Triangle::Triangle(Eigen::Vector3f& v1, Eigen::Vector3f& v2, Eigen::Vector3f& v3,
            Eigen::Vector2f& uv1, Eigen::Vector2f& uv2, Eigen::Vector2f& uv3){
  face_ = new Eigen::Vector3f[3];
  face_[0] = v1;
  face_[1] = v2;
  face_[2] = v3;
  uv_ = new Eigen::Vector2f[3];
  uv_[0] = uv1;
  uv_[1] = uv2;
  uv_[2] = uv3;
}

Triangle::~Triangle(){
  delete[] face_;
  delete[] uv_;
}

void Triangle::setVert(Eigen::Vector3f& v, int ind){
  assert (ind>=0 && ind<3);
  face_[ind] = v;
}

void Triangle::setUV(Eigen::Vector2f& uv, int ind){
  assert (ind>=0 && ind<3);
  uv_[ind] = uv;
}

float getNormal()const{
  Eigen::Vector3f side1 = face_[1]-face[0];
  Eigen::Vector3f side2 = face_[2]-face[0];
  return side1.cross(side2).normalized();
}

/*void Triangle::setVert(std::string v, int ind){
  assert (ind>=0 && ind<3);
  face_[ind] = 
}*/
