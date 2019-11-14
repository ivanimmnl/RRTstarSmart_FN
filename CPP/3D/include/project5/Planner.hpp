#include <math.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>

#include "project5/Map_manager.hpp"

// Structure used to store the Closed Nodes

class Planner
{
public:
  Planner(Map_manager manager, int step_size);
  std::vector<geometry_msgs::PoseStamped> makePlan(std::vector<float> root, std::vector<float> target);
  float calculateDistance(std::vector<float> first_point, std::vector<float> second_point);

  //for tree
  std::vector<std::vector<float>> node;
  std::vector<float> node_cost;
  std::vector<int> node_parent;
  std::vector<std::vector<float>> Xbeacon_old;
  std::vector<std::vector<float>> Xbeacon_new;

private:
  std::vector<std::vector<std::vector<int>>> Cfree;
  float region_radius;
  float beacon_radius;
  float goal_radius;
  int branch_length;
  float distance_to_target;
  const int b = 5;

  int voxel_dim = 64; //change according to binvox

  int max_nodes = 400;

  Map_manager map;

  std::vector<float> get_random_point();
  std::vector<float> get_random_point_beacon(std::vector<std::vector<float>> Xbeacon_old);
  std::vector<float> find_nearest(std::vector<float> Xrand);
  bool hasObstacle(std::vector<float> Xnear, std::vector<float> Xnew);
  std::vector<float> new_node(std::vector<float> Xnear, std::vector<float> Xrand);
  std::vector<int> get_neighbourhood(std::vector<float> Xnew);
  std::vector<float> get_best_parent(std::vector<int> neighbourhood);

  int randNum(int min, int max);
};
