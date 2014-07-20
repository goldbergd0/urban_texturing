#ifndef OBJ_IO_H_
#define OBJ_IO_H_
#include <pcl/pcl_macros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/texture.h>
namespace pcl
{
  namespace io
  {
    /** \brief Saves a PolygonMesh in ascii OBJ format.
      * \param file_name the name of the file to write to disk
      * \param triangles the polygonal mesh to save
      * \param precision the output ASCII precision
      * \ingroup io
      */
    PCL_EXPORTS int
    saveOBJFile (const std::string &file_name,
            const pcl::TextureMesh &tex_Mesh, unsigned precision = 5);
  }
}

#endif /* OBJ_IO_H_ */

