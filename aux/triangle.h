// Dan Goldberg
// Triangle class

#ifndef TRIANGLE_H_
#define VECTOR_H_

#include <Eigen/Dense>

class Triangle {
  public:
    Triangle();
    Triangle(Eigen::Vector3f& v1, Eigen::Vector3f& v2, Eigen::Vector3f& v3);

    
    
  private:
    Eigen::Vector3f v1_;
    Eigen::Vector3f v2_;
    Eigen::Vector3f v3_;

    


};


#endif
