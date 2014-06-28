// Dan Goldberg
// Triangle object

#ifndef TRIANGLE_H_
#define TRIANGLE_H_

#include <vector>

#include "../aux/patch.h"

class Triangle{
  public:
    Triangle();
    ~Triangle();

    void setv0(const Patch& v){v0_=v;};
    void setv1(const Patch& v){v1_=v;};
    void setv2(const Patch& v){v2_=v;};

    void calcUV();
    
  private:
    Patch v0_;
    Patch v1_;
    Patch v2_;
    std::vector<int> uv0_;
    std::vector<int> uv1_;
    std::vector<int> uv2_;

};

#endif
