#include "project5/Map_manager.hpp"
#include "/home/ivanimmnl/MATLAB/polyspace/verifier/cxx/include/include-stl/fstream.h"

using namespace cv;

Map_manager::Map_manager(){
    int x_count = 0;
    int y_count = 0;
    int z_count = 0;
    long index;

    char ch[1];
    std::ifstream voxel_data;
    voxel_data.open("/home/ivanimmnl/ethz_waypoint/src/3D_RRTstar_SmartFN/voxels.txt");
    while(voxel_data.get(ch[0])){
        if (ch[0] == 32) {y_count++; continue;} //comma
        if (ch[0] == 10) {z_count++; y_count = 0; continue;} //newline
        if (z_count == voxel_dim) {z_count = 0; x_count++;}
        index = x_count + y_count*voxel_dim + z_count*voxel_dim*voxel_dim;
        voxel[index] = atoi(ch);
    }
    voxel_data.close();
}

// returns the Cspace
//std::vector<std::vector<int> > Map_manager::getCfree(){
//  return Cfree;
//}

// returns the color at the given coordinate
int Map_manager::get_state(int x, int y, int z){
        long lin = x + y*voxel_dim + z*voxel_dim*voxel_dim;
        return (voxel[lin]);
}

// Check if the particular grid has obstacle
bool Map_manager::checkObstacle(std::vector<int> grid){
  if (get_state(grid[0], grid[1], grid[2])==1)
    return true;
  else
    return false;
}

// get position in meter and return in voxel coordinates in image
std::vector<float> Map_manager::computeGridCoordinate(std::vector<float> position){
  std::vector<float> grid;
  grid.push_back(floor((position[0]+26.3329)*voxel_dim/90.9341 - 1));
  grid.push_back(floor((position[1]+25.4699)*voxel_dim/90.9341 - 1));
  grid.push_back(floor((position[2])*voxel_dim/90.9341 - 1));
  return grid;
}

// get position in pixel coordinates and return in meter in image
std::vector<double> Map_manager::computeDistanceCoordinate(std::vector<float> position){
  std::vector<double> distance;
  distance.push_back((position[0]+1)*90.9341/(float)voxel_dim - 26.3329);
  distance.push_back((position[1]+1)*90.9341/(float)voxel_dim - 25.4699);
  distance.push_back((position[2]+1)*90.9341/(float)voxel_dim);
  return distance;
}

// show the read image in a window
void Map_manager::show_image(){
  namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
  imshow( "Display window", image ); 
  waitKey(60000);
}

