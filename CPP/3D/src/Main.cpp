#include "project5/Map_manager.hpp"
#include "project5/Planner.hpp"
#include "project5/matplotlibcpp.h"
#include "project5/controller.hpp"

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geometry_msgs/Twist.h>

#include <cmath>

#include "/home/ivanimmnl/MATLAB/polyspace/verifier/cxx/include/include-stl/fstream.h"

#define DEBUG false
#define STEP_SIZE 2

namespace plt = matplotlibcpp;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "project5_node");

  ros::NodeHandle node;

  // Initialization
  Map_manager manager;
  Planner planner(manager, STEP_SIZE);
  controller control(node, STEP_SIZE);

  // Initialized position corresponds to the spawn position in gazebo environment
  vector<float> start_state, end_state;
  vector<float> start_point, target_point;

  start_state.push_back(1);
  start_state.push_back(1);
  start_state.push_back(5);

  end_state.push_back(52);
  end_state.push_back(10);
  end_state.push_back(5);
  end_state.push_back(0);

  float final_yaw = end_state[3];

  target_point = manager.computeGridCoordinate(end_state);
  start_point = manager.computeGridCoordinate(start_state);

  if (DEBUG)
  {
    cout << "Start Point: " << start_point[0] << ", " << start_point[1] << endl;
    cout << "End Point: " << target_point[0] << ", " << target_point[1] << endl;
  }

  // Make the plan with RRT
  std::vector<geometry_msgs::PoseStamped> plan;
  plan = planner.makePlan(start_point, target_point);
  
  //WRITE TO FILE
  ifstream xfile, yfile, zfile, yawfile, templatefile;
  xfile.open("xfile.txt"); yfile.open("yfile.txt"); zfile.open("zfile.txt"); yawfile.open("yawfile.txt"); templatefile.open("waypoint_template.txt");
  ofstream outfile;
  outfile.open("waypoints_3D.yaml", std::ios::trunc);
  char ch;
  while(templatefile.get(ch)) outfile << ch;
  while(xfile.get(ch)) outfile << ch;
  while(yfile.get(ch)) outfile << ch;
  while(zfile.get(ch)) outfile << ch;
  while(yawfile.get(ch)) outfile << ch;

  xfile.close(); yfile.close(); zfile.close(); yawfile.close(); outfile.close();
  //PROCEED TO WAYPOINT_NAVIGATOR TO USE THE GENERATED YAML FILE

  return 0;
}