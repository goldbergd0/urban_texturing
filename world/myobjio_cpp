/*
 * obj_io.cpp
 *
 *  Created on: Jun 7, 2011
 *      Author: ktran
 */
#include <pcl/io/obj_io.h>
#include <fstream>
#include <iostream>
#include <pcl/io/io.h>

//////////////////////////////////////////////////////////////////////////////////////////////
int myobjio::saveOBJFile (const std::string &file_name,
            const my::TextureMesh &tex_Mesh, unsigned precision)
{
  if (tex_Mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  std::string mtl_file_name = file_name.substr(0, file_name.find_last_of("."))+".mtl";

  /* Write 3D information */
  // number of points
  int nr_points  = tex_Mesh.cloud.width * tex_Mesh.cloud.height;
  int point_size = tex_Mesh.cloud.data.size () / nr_points;
  // number of facets
  int nr_faces = tex_Mesh.tex_polygons.size();
  // number of normal vectors
  int nr_normals = tex_Mesh.tex_normals.size();
  // number of normal vectors
  int nr_texcoordinates = tex_Mesh.tex_coordinates.size();

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs <<"mtllib " <<mtl_file_name << std::endl;
  fs << "####" << std::endl;
  fs << "g Group001" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;

  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_Mesh.cloud.fields.size (); ++d)
    {
      int count = tex_Mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_Mesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
                  tex_Mesh.cloud.fields[d].name == "x" ||
                  tex_Mesh.cloud.fields[d].name == "y" ||
                  tex_Mesh.cloud.fields[d].name == "z"))
      {
        if(!v_written)
        {
                 // write vertices beginning with v
                fs << "v ";
                v_written = true;
        }
        float value;
        memcpy (&value, &tex_Mesh.cloud.data[i * point_size + tex_Mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
      }
      fs << " ";
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

  // Write normal vector with "vn" (now we don't have it)
  fs << "# "<< nr_normals <<" normal" << std::endl;
  for (size_t i = 0; i < tex_Mesh.tex_normals.size(); ++i){
        fs << "vn ";
        fs << tex_Mesh.tex_normals[i].normal_x;
        fs << " " ;
        fs << tex_Mesh.tex_normals[i].normal_y;
        fs << " ";
        fs << tex_Mesh.tex_normals[i].normal_z << std::endl;
  }
    // Write vertex texture with "vt" (adding latter)
  fs << "# "<< nr_texcoordinates <<" vertex textures" << std::endl;
  for (size_t i = 0; i < tex_Mesh.tex_coordinates.size(); ++i){
        fs << "vt ";
        fs <<  tex_Mesh.tex_coordinates[i].x << " " << tex_Mesh.tex_coordinates[i].y << std::endl;
  }

  // Specify the material will be used
  fs << "# The material will be used" << std::endl;
  fs << "usemtl " <<  tex_Mesh.tex_material.tex_name << std::endl;
  // Write faces with "f"
  fs << "# Faces" << std::endl;
  for (size_t i = 0; i < tex_Mesh.tex_polygons.size (); ++i)
  {
        fs << "f ";
    size_t j = 0;
    for (j = 0; j < tex_Mesh.tex_polygons[i].vertices.size () - 1; ++j)
      fs << tex_Mesh.tex_polygons[i].vertices[j] +1 <<"/" << tex_Mesh.tex_polygons[i].vertices[j] +1 << " "; // vertex index in obj file format starting with 1
    fs << tex_Mesh.tex_polygons[i].vertices[j]+1 <<"/" << tex_Mesh.tex_polygons[i].vertices[j] +1 << std::endl;
  }
  fs << "# "<< nr_faces <<" faces" << std::endl;
  fs << "# End of File";

  // Close obj file
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;

  m_fs << "newmtl " << tex_Mesh.tex_material.tex_name << std::endl;
  m_fs << "Ka "<< tex_Mesh.tex_material.tex_Ka.r << " " << tex_Mesh.tex_material.tex_Ka.g << " " << tex_Mesh.tex_material.tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
  m_fs << "Kd "<< tex_Mesh.tex_material.tex_Kd.r << " " << tex_Mesh.tex_material.tex_Kd.g << " " << tex_Mesh.tex_material.tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
  m_fs << "Ks "<< tex_Mesh.tex_material.tex_Ks.r << " " << tex_Mesh.tex_material.tex_Ks.g << " " << tex_Mesh.tex_material.tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
  m_fs << "d "<< tex_Mesh.tex_material.tex_d << std::endl; // defines the transparency of the material to be alpha.
  m_fs << "Ns "<< tex_Mesh.tex_material.tex_Ns  << std::endl; // defines the shininess of the material to be s.
  m_fs << "illum "<< tex_Mesh.tex_material.tex_illum << std::endl; // denotes the illumination model used by the material.
                                          // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
                                          // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
  m_fs << "map_Kd " << tex_Mesh.tex_material.tex_file << std::endl;
  m_fs.close();
  return (0);
}

