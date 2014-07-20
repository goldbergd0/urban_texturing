/*
 * texture.h
 *
 *  Created on: Jun 10, 2011
 *      Author: ktran
 */

#ifndef TEXTURE_H_
#define TEXTURE_H_
#include <string>
#include <vector>
#include <ostream>

// Include the correct Header path here
#include <pcl/PolygonMesh.h>
#include "pcl/Vertices.h"
#include "pcl/point_types.h"

namespace my
{
  struct RGB{
    float r;
        float g;
        float b;
  }; //RGB

  struct TexMaterial
  {
          std::string           tex_name;       // texture name
      std::string               tex_file;       // texture file
      pcl::RGB                  tex_Ka;         // defines the ambient color of the material to be (r,g,b).
      pcl::RGB                  tex_Kd;         // defines the diffuse color of the material to be (r,g,b).
      pcl::RGB                  tex_Ks;         // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
      float                     tex_d;          // defines the transparency of the material to be alpha.
      float                     tex_Ns;         // defines the shininess of the material to be s.
      int                               tex_illum;      // denotes the illumination model used by the material.
                                                                                        // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
                                                                                    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
  }; // TexMaterial
  struct TextureMesh
  {
        TextureMesh () : header (), cloud (), tex_polygons ()
        {}

        pcl::PCLHeader                  header;

        pcl::PointCloud2                cloud;

        std::vector< ::pcl::Vertices>   tex_polygons; // polygon which is mapped with specific texture defined in TexMaterial
        std::vector< ::pcl::Normal>     tex_normals; // normal vertices
        std::vector< ::pcl::PointXY>    tex_coordinates; // UV coordinates
        TexMaterial                                             tex_material; // define texture material
        public:
          typedef boost::shared_ptr< ::pcl::PolygonMesh> Ptr;
          typedef boost::shared_ptr< ::pcl::PolygonMesh const> ConstPtr;

  }; // struct TextureMesh

  typedef boost::shared_ptr< ::pcl::TextureMesh> TextureMeshPtr;
  typedef boost::shared_ptr< ::pcl::TextureMesh const> TextureMeshConstPtr;

} // namespace pcl

#endif /* TEXTURE_H_ */

