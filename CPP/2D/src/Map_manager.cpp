#include "project5/Map_manager.hpp"

using namespace cv;

Map_manager::Map_manager(){
  image = imread( "/home/ivanimmnl/ethz_waypoint/src/2D_RRTstar_turtlebot3/maps/JEselect.png", cv::IMREAD_GRAYSCALE );

  if ( !image.data )
  {
    std::cout<< "No Data " <<  std::endl;
  }
  
  // create Cfree space to produce random nodes
  for(int i=0;i<384;i++)
    for(int j=0;j<608;j++) //384
      if(Map_manager::get_state(i,j)>150)
      {
        std::vector<int> point;
        point.push_back(i);
        point.push_back(j);
        Cfree.push_back(point);
      }      
}

// returns the Cspace
std::vector<std::vector<int> > Map_manager::getCfree(){
  return Cfree;
}

// returns the color at the given coordinate
int Map_manager::get_state(int x, int y){	
        return (int)image.at<uchar>((608-y),x); //384
}

// Check if the particular grid has obstacle
bool Map_manager::checkObstacle(std::vector<int> grid){
  if (get_state(grid[0], grid[1])==0)
    return true;
  else
    return false;
}

// get position in meter and return in pixel coordinates in image
std::vector<float> Map_manager::computeGridCoordinate(std::vector<float> position){
  std::vector<float> grid;
  grid.push_back(floor((position[0]+10)*20));
  grid.push_back(floor((position[1]+10)*20));
  return grid;
}

// get position in pixel coordinates and return in meter in image
std::vector<double> Map_manager::computeDistanceCoordinate(std::vector<float> position){
  std::vector<double> distance;
  distance.push_back((position[0]*0.05) - 10);
  distance.push_back((position[1]*0.05) - 10);
  return distance;
}

// show the read image in a window
void Map_manager::show_image(){
  namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.
  imshow( "Display window", image ); 
  waitKey(60000);
}

