//SDFGen - A simple grid-based signed distance field (level set) generator for triangle meshes.
//Written by Christopher Batty (christopherbatty@yahoo.com, www.cs.columbia.edu/~batty)
//...primarily using code from Robert Bridson's website (www.cs.ubc.ca/~rbridson)
//This code is public domain. Feel free to mess with it, let me know if you like it.

#include "makelevelset3.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>

std::string replace_all(std::string str, const std::string &remove, const std::string &insert)
{
  std::string::size_type pos = 0;
  while ((pos = str.find(remove, pos)) != std::string::npos)
  {
    str.replace(pos, remove.size(), insert);
    pos++;
  }

  return str;
}

int main(int argc, char* argv[]) {
  
  if(argc != 5) {
    std::cout << "SDFGen - A utility for converting closed oriented triangle meshes into grid-based signed distance fields.\n";

    std::cout << "Usage: SDFGen <filename> <templatejs> <dx> <padding>\n\n";
    std::cout << "Where:\n";
    std::cout << "\t<filename> specifies a Wavefront OBJ (text) file representing a *triangle* mesh (no quad or poly meshes allowed). File must use the suffix \".obj\".\n";
    std::cout << "<templatejs> is the template file for the output in javascript format, which should have a ${SDF} string somewhere in it, where the data is printed.\n\n";
    std::cout << "\t<dx> specifies the length of grid cell in the resulting distance field.\n";
    std::cout << "\t<padding> specifies the number of cells worth of padding between the object bound box and the boundary of the distance field grid. Minimum is 1.\n\n";
    
    exit(-1);
  }

  std::string filename(argv[1]);
  if(filename.size() < 5 || filename.substr(filename.size()-4) != std::string(".obj")) {
    std::cerr << "Error: Expected OBJ file with filename of the form <name>.obj.\n";
    exit(-1);
  }

  std::string jstemplate(argv[2]);

  std::stringstream arg2(argv[3]);
  float dx;
  arg2 >> dx;
  
  std::stringstream arg3(argv[4]);
  int padding;
  arg3 >> padding;

  if(padding < 1) padding = 1;
  //start with a massive inside out bound box.
  Vec3f min_box(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max()), 
    max_box(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
  
  std::cout << "Reading data.\n";

  std::ifstream infile(argv[1]);
  if(!infile) {
    std::cerr << "Failed to open. Terminating.\n";
    exit(-1);
  }

    std::vector<Vec3f> vertList;
    std::vector<Vec3ui> faceList;

    /*******************************************************/

    std::string inputfile = argv[1];
    std::cout << "Loading " << inputfile << std::endl;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string err;
    bool triangulate = true;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, inputfile.c_str(),
                                NULL, triangulate);

    size_t numVertices = attrib.vertices.size()/3;
    for (size_t vi=0; vi<numVertices; vi++)
    {
        Vec3f point;
        point[0] = attrib.vertices[3*vi];
        point[1] = attrib.vertices[3*vi+1];
        point[2] = attrib.vertices[3*vi+2];
        vertList.push_back(point);
        update_minmax(point, min_box, max_box);
    }
    for (size_t i = 0; i < shapes.size(); i++)
    {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[i].mesh.num_face_vertices.size(); f++) {
            size_t fnum = shapes[i].mesh.num_face_vertices[f];
            for (size_t v = 2; v < fnum; v++) {
                tinyobj::index_t idxA = shapes[i].mesh.indices[index_offset + v];
                tinyobj::index_t idxB = shapes[i].mesh.indices[index_offset + v - 1];
                tinyobj::index_t idxC = shapes[i].mesh.indices[index_offset + v - 2];
                faceList.push_back(Vec3ui(idxA.vertex_index, idxB.vertex_index, idxC.vertex_index));
            }
            index_offset += fnum;
        }
    }

    /*******************************************************/

  std::cout << "Read in " << vertList.size() << " vertices and " << faceList.size() << " faces." << std::endl;

  //Add padding around the box.
  Vec3f unit(1,1,1);
  min_box -= padding*dx*unit;
  max_box += padding*dx*unit;
  Vec3ui sizes = Vec3ui((max_box - min_box)/dx);
  
  std::cout << "Bound box size: (" << min_box << ") to (" << max_box << ") with dimensions " << sizes << "." << std::endl;

  std::cout << "Computing signed distance field.\n";
  Array3f phi_grid;
  make_level_set3(faceList, vertList, min_box, dx, sizes[0], sizes[1], sizes[2], phi_grid);

  // Write SDF results in JS form:
  float edge_x = phi_grid.ni * dx;
  float edge_y = phi_grid.nj * dx;
  float edge_z = phi_grid.nk * dx;

  std::stringstream sdf_js;
  sdf_js << "\tlet asset = {" << std::endl;
  sdf_js << "\t\t metadata: { ORIG: [" <<  min_box[0] << ", " << min_box[1] << ", " << min_box[2] << "]," << std::endl;
  sdf_js << "\t\t             EDGE: [" <<  edge_x << ", " << edge_y << ", " << edge_z << "]," << std::endl;
  sdf_js << "\t\t             GRES: [" <<  phi_grid.ni << ", " << phi_grid.nj << ", " << phi_grid.nk << "] }," << std::endl;
  sdf_js << "\t\t data: new Float32Array( [ ";
  for(unsigned int i = 0; i < phi_grid.a.size(); ++i) { if (i>0) sdf_js << ", "; sdf_js << phi_grid.a[i]; }
  sdf_js << "\t\t ] )" << std::endl;
  sdf_js << "\t};" << std::endl;

  // template file for javascript generator
  std::ifstream t(jstemplate.c_str());
  std::string template_js((std::istreambuf_iterator<char>(t)),
                  std::istreambuf_iterator<char>());
  std::cout << template_js << std::endl;
  std::string generated_js = replace_all(template_js, "${SDF}", sdf_js.str());

    // Very hackily strip off file suffix.
    std::string outname;
    outname = filename.substr(0, filename.size()-4) + std::string(".html");
    std::cout << "Writing results to: " << outname << "\n";
    std::ofstream outfile( outname.c_str());
    outfile << generated_js << std::endl;

  std::cout << "Processing complete.\n";

return 0;
}
