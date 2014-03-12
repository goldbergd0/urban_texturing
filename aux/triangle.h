// Dan Goldberg
// Triangle class

#ifndef TRIANGLE_H_
#define VECTOR_H_

#include <string>
#include <Eigen/Dense>

class Triangle {
  private:
    Eigen::Vector3f* face_;
    Eigen::Vector2f* uv_;

  public:
    Triangle();
    Triangle(Eigen::Vector3f& v1, Eigen::Vector3f& v2, Eigen::Vector3f& v3,
              Eigen::Vector2f& uv1, Eigen::Vector3f& uv2, Eigen::Vector2f& uv3);
    virtual ~Triangle();
 
    void setVert(Eigen::Vector3f& v, int ind);
    void setUV(Eigen::Vector2f& uv, int ind);
//    void setVert(std::string v, int ind);

    Eigen::Vector3f* getFace()const{return face_;};
    Eigen::Vector2f* getUV()const{return uv_;};
    float getNormal()const;
};    
#endif
