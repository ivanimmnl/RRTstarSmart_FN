#include "project5/Planner.hpp"
#include "/home/ivanimmnl/MATLAB/polyspace/verifier/cxx/include/include-stl/fstream.h"
#include <time.h>


#define DEBUG false

Planner::Planner(Map_manager manager, int step_size)
{
    Cfree = manager.getCfree();

    branch_length = step_size;
    region_radius = step_size*3/2;
    beacon_radius = step_size*1/2;
    goal_radius = step_size * 3;

    map = manager;
}

// Return a random point from the Cspace
std::vector<float> Planner::get_random_point()
{
    //int random = randNum(0, Cfree.size());
    std::vector<float> floatVec;
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<> x(0, 384);
    std::uniform_real_distribution<> y(0, 608); //384
    floatVec.push_back(x(gen));
    floatVec.push_back(y(gen));

    return floatVec;
}

// Return a random point from the Cspace near a random beacon node
std::vector<float> Planner::get_random_point_beacon(std::vector<std::vector<float> > Xbeacon_old)
{
    //int random = randNum(0, Cfree.size());
    float angle = 2 * M_PI * randNum(1, 100)/100;
    float radius = beacon_radius * randNum(1, 100)/100;
    int j = randNum(0, Xbeacon_old.size() - 3) + 1;
    std::vector<float> floatVec;

    float x = floor(Xbeacon_old[j][0] + radius * cos(angle));
    float y = floor(Xbeacon_old[j][1] + radius * sin(angle));

    floatVec.push_back(x);
    floatVec.push_back(y);

    return floatVec;
}

bool Planner::hasObstacle(std::vector<float> Xnear, std::vector<float> Xnew)
{
    bool result = false;

    float diff1 = (Xnew[0] - Xnear[0] + 0.00001);
    float diff2 = (Xnew[1] - Xnear[1] + 0.00001);

    int decimated_index;
    float diff;

    // take the greater difference
    if (fabs(diff1) > fabs(diff2))
    {
        diff = diff1;
        decimated_index = 1;
    }
    else
    {
        diff = diff2;
        decimated_index = 0;
    }

    // Creates set of points between two points
    std::vector<std::vector<float> > points_to_check;
    points_to_check.push_back(Xnear);

    for (int ii = 1; ii <= fabs(diff); ii++)
    {
        std::vector<float> point;
        std::vector<float> temp;

        point.push_back(Xnear[0] + ii * diff1 / fabs(diff));
        point.push_back(Xnear[1] + ii * diff2 / fabs(diff));

        temp.push_back(point[decimated_index]);
        point[decimated_index] = floor(point[decimated_index]);

        points_to_check.push_back(point);

        if (temp[0] != point[decimated_index])
        {
            point[decimated_index]=point[decimated_index]+1;
            points_to_check.push_back(point);
        }
    }

    // returns true if one of the point in between is an obstacle
    for (int jj = 0; jj < points_to_check.size(); jj++)
    {
        std::vector<int> floatVec;
        floatVec.push_back(floor(points_to_check[jj][0]));
        floatVec.push_back(floor(points_to_check[jj][1]));
        if (map.checkObstacle(floatVec))
        {
            result = true;
        }
    }

    return result;
}

// returns the nearest node in the tree
std::vector<float> Planner::find_nearest(std::vector<float> Xrand)
{
    std::vector<float> Xnear;
    long min_distance = 1000;
    long distance;

    for (int ii = 0; ii < node.size(); ii++)
    {
        distance = calculateDistance(Xrand, node[ii]);
        if (distance < min_distance)
        {
            min_distance = distance;
            Xnear = node[ii];
        }
    }
    return Xnear;
}

// returns the new node displaced a particular branch length in the random node's direction
std::vector<float> Planner::new_node(std::vector<float> Xnear, std::vector<float> Xrand)
{
    std::vector<float> Xnew;
    float slope = (Xrand[1] - Xnear[1]) / (Xrand[0] - Xnear[0] + 0.00001);
    float adjuster = branch_length * sqrt(1 / (1 + pow(slope, 2)));

    std::vector<float> point1, point2;

    point1.push_back(Xnear[0] + adjuster);
    point1.push_back(Xnear[1] + slope * adjuster);

    point2.push_back(Xnear[0] - adjuster);
    point2.push_back(Xnear[1] - slope * adjuster);

    float distance1 = calculateDistance(Xrand, point1);
    float distance2 = calculateDistance(Xrand, point2);
    if (distance1 < distance2)
        Xnew = point1;
    else
        Xnew = point2;

    return Xnew;
}

// returns the index of nodes in neighbourhood
std::vector<int> Planner::get_neighbourhood(std::vector<float> Xnew)
{
    std::vector<int> neighbourhood;
    for (int i = 0; i < node.size(); i++)
    {
        if (calculateDistance(node[i], Xnew) < region_radius)
            neighbourhood.push_back(i);
    }
    return neighbourhood;
}

// returns the parent with least cost to come
std::vector<float> Planner::get_best_parent(std::vector<int> neighbourhood)
{
    float min = node_cost[neighbourhood[0]];
    std::vector<float> Xnear = node[neighbourhood[0]];
    int position = neighbourhood[0];
    for (int i = 1; i < neighbourhood.size(); i++)
    {
        if (min > node_cost[neighbourhood[i]])
        {
            min = node_cost[neighbourhood[i]];
            Xnear = node[neighbourhood[i]];
            position = neighbourhood[i];
        }
    }
    Xnear.push_back(position);
    // The third index is the position in the tree
    return Xnear;
}

// returns random number between min and max
int Planner::randNum(int min, int max)
{
    if(max<=min) return min;
    return rand() % max + min;
}

// returns the euclidian distance between two points
float Planner::calculateDistance(std::vector<float> first_point, std::vector<float> second_point)
{
    return (float)sqrt((double)pow(first_point[0] - second_point[0], 2) + (double)pow(first_point[1] - second_point[1], 2));
}

std::vector<geometry_msgs::PoseStamped> Planner::makePlan(std::vector<float> root, std::vector<float> target)
{
    std::vector<geometry_msgs::PoseStamped> plan;

    //Clear the content of variables
    node.clear();
    node_cost.clear();
    node_parent.clear();
    Xbeacon_old.clear();
    Xbeacon_new.clear();

    //Initialize
    node.push_back(root);
    node_cost.push_back(0);
    node_parent.push_back(-1);

    float direct_cost_old;
    float direct_cost_new;
    int n_target;

    std::vector<int> floaterVec;
    floaterVec.push_back(floor(target[0]));
    floaterVec.push_back(floor(target[1]));

    //Check whether determined target lies in obstacle
    if(map.checkObstacle(floaterVec))
    {
        std::cout<<"Target in obstacle"<< std::endl;
        return plan;
    }

    distance_to_target = branch_length + 20;

    int count = 0;

    std::vector<int> neighbourhood;
    std::vector<float> Xnew, Xnear, Xnearest, Xrand, parent;
    bool init_path_found = false;
    int position;

    Xnew = root;
    std::cout << "------------- Starting Search ------------- " << std::endl;
    ros::WallTime t_start, t_end;
    t_start = ros::WallTime::now();

    while (count<20000)
    {
        count++;

        if(DEBUG) std::cout << "-- Current Count :" << count << "    -- Has Obstacle :"<< hasObstacle(target, Xnew) <<std::endl;

        //Generate random point
        Xrand = (init_path_found && count % b == 0) ? get_random_point_beacon(Xbeacon_old) : get_random_point();

        //Find nearest node in tree
        Xnearest = find_nearest(Xrand);
        if(Xnearest[0]==Xrand[0] || Xnearest[1]==Xrand[1])
            continue;

        //Generate new node Xnew
        Xnew = new_node(Xnearest, Xrand);
        std::vector<int> intVec;
        intVec.push_back(floor(Xnew[0]));
        intVec.push_back(floor(Xnew[1]));

        if (map.checkObstacle(intVec)) //if new node is in obstacle, continue with next iteration
            continue;

        neighbourhood = get_neighbourhood(Xnew);
        parent = get_best_parent(neighbourhood);
        Xnear.clear();
        Xnear.push_back(parent[0]);
        Xnear.push_back(parent[1]);
        position = parent[2];

        // Add node if obstacle not in between
        if (!hasObstacle(Xnear, Xnew))
        {
            long current_no_of_nodes = node.size();

            if (DEBUG)
            {
                std::cout << "Printing nodes...\n";
                for (int cc = 0; cc < node.size(); cc++)
                {
                    std::cout << "[" << node[cc][0] << ", " << node[cc][1] << "] " << std::endl;
                }
                std::cout << "Press any key to continue...\n";
                getchar();
            }

            // Add new node to tree
            float temp_cost = node_cost[position] + calculateDistance(Xnear, Xnew);
            node.push_back(Xnew);
            node_cost.push_back(temp_cost);
            node_parent.push_back(position);

            // Rewiring
            for (int cc = 0; cc < neighbourhood.size(); cc++)
            {
                if (!hasObstacle(node[current_no_of_nodes], node[neighbourhood[cc]]) && (node_cost[current_no_of_nodes] + calculateDistance(node[neighbourhood[cc]], Xnew)) < node_cost[neighbourhood[cc]])
                {
                    // If cost from new node is cheaper switch parents
                    node_cost[neighbourhood[cc]] = node_cost[current_no_of_nodes] + calculateDistance(node[neighbourhood[cc]], Xnew);
                    node_parent[neighbourhood[cc]] = current_no_of_nodes;
                }
            }

            //Check if goal is reached
            if (!init_path_found && calculateDistance(Xnew, target) < region_radius && !hasObstacle(Xnew, target))
            {
                //Perform manual forced removal if goal is reached after maximum nodes is exceeded
                if (node.size() > max_nodes)
                {
                    int j = 0;
                    int temp = 0;
                    long chosen_index = 0;
                    while (j < 2) //Repeat twice for Xnew and target
                    {
                        std::vector<int> childless_node_index;
                        for (int jj = 0; jj < node.size() - 1; jj++)
                        {
                            //Find all childless nodes
                            std::vector<int>::iterator it;
                            it = std::find(node_parent.begin(), node_parent.end(), jj);
                            if (it == node_parent.end())
                                childless_node_index.push_back(jj);
                        }

                        bool is_near_target = true;
                        while (is_near_target)
                        {
                            //Do not remove nodes near target
                            if(childless_node_index.size() == 1) {
                                chosen_index = childless_node_index[0]; //just delete the last remaining node
                                break;
                            }
                            int random_index = randNum(0, childless_node_index.size() - 1);
                            chosen_index = childless_node_index[random_index];
                            if (calculateDistance(node[chosen_index], target) <= goal_radius)
                            {
                                childless_node_index.erase(std::remove(childless_node_index.begin(), childless_node_index.end(), chosen_index), childless_node_index.end());
                                continue;
                            }
                            is_near_target = false;
                        }

                        if (j == 0)
                        {
                            node[chosen_index] = Xnew;
                            node_cost[chosen_index] = temp_cost;
                            node_parent[chosen_index] = position;

                            std::replace(node_parent.begin(), node_parent.end(), current_no_of_nodes, chosen_index);
                            temp = chosen_index;
                        }
                        else
                        {
                            node[chosen_index] = target;
                            node_cost[chosen_index] = temp_cost + calculateDistance(Xnew, target);
                            node_parent[chosen_index] = temp;
                            n_target = chosen_index;
                        }

                        node.pop_back();
                        node_cost.pop_back();
                        node_parent.pop_back();
                        j++;
                    }
                }
                else //Else, proceed as normal
                {
                    node.push_back(target);
                    node_cost.push_back(temp_cost + calculateDistance(Xnew, target));
                    node_parent.push_back(current_no_of_nodes);
                    n_target = node.size() - 1;
                }

                init_path_found = true;
                direct_cost_old = temp_cost + calculateDistance(Xnew, target);
            };

            //Path Optimization
            if (init_path_found)
            {
                Xbeacon_new.clear();
                Xbeacon_new.push_back(target);
                std::vector<float> Xend;
                Xend.push_back(node[n_target][0]);
                Xend.push_back(node[n_target][1]);
                int Xend_parent = node_parent[n_target];
                int temp = Xend_parent;

                //Try to directly connect visible nodes in the found solution path, starting from target
                while (Xend_parent != -1)
                {
                    if (!hasObstacle(node[Xend_parent], Xend))
                    {
                        temp = Xend_parent;
                        Xend_parent = node_parent[Xend_parent];
                        continue;
                    }
                    Xend = node[temp];
                    Xbeacon_new.push_back(Xend);
                }
                Xbeacon_new.push_back(root);

                //Calculate new direct cost
                direct_cost_new = 0;
                for (int ii = 0; ii < Xbeacon_new.size() - 1; ii++)
                    direct_cost_new += calculateDistance(Xbeacon_new[ii], Xbeacon_new[ii + 1]);

                if (direct_cost_new < direct_cost_old)
                {
                    //Update beacon nodes if new direct cost is lesser
                    direct_cost_old = direct_cost_new;
                    Xbeacon_old.clear();
                    Xbeacon_old = Xbeacon_new;
                }
            }

            //Forced Removal
            if (!init_path_found && node.size() > max_nodes) //Before solution path is obtained
            {
                std::vector<int> childless_node_index;
                long chosen_index = 0;
                for (int jj = 0; jj < node.size() - 1; jj++)
                {
                    //Find all childless nodes
                    std::vector<int>::iterator it;
                    it = std::find(node_parent.begin(), node_parent.end(), jj);
                    if (it == node_parent.end())
                        childless_node_index.push_back(jj);
                }

                bool is_near_target = true;
                while (is_near_target)
                {
                    //Do not remove nodes around target
                    if(childless_node_index.size() == 1) {
                        chosen_index = childless_node_index[0]; //just delete the last remaining node
                        break;
                    }
                    int random_index = randNum(0, childless_node_index.size() - 1);
                    chosen_index = childless_node_index[random_index];
                    if (calculateDistance(node[chosen_index], target) <= goal_radius)
                    {
                        childless_node_index.erase(std::remove(childless_node_index.begin(), childless_node_index.end(), chosen_index), childless_node_index.end());
                        continue;
                    }
                    is_near_target = false;
                }

                //Replace the removed node with Xnew
                node[chosen_index] = Xnew;
                node_cost[chosen_index] = temp_cost;
                node_parent[chosen_index] = position;

                //Update nodes having Xnew as parent
                std::replace(node_parent.begin(), node_parent.end(), current_no_of_nodes, chosen_index);

                //Remove last element of node
                node.pop_back();
                node_cost.pop_back();
                node_parent.pop_back();
            }
            else if (init_path_found && node.size() > max_nodes) //After solution path is obtained
            {
                std::vector<int> childless_node_index;
                long chosen_index = 0;
                for (int jj = 0; jj < node.size() - 1; jj++)
                {
                    //Find all childless node, skip if it is target node
                    if (jj == n_target)
                        continue;
                    std::vector<int>::iterator it;
                    it = std::find(node_parent.begin(), node_parent.end(), jj);
                    if (it == node_parent.end())
                        childless_node_index.push_back(jj);
                }

                bool is_near_beacon = true;
                while (is_near_beacon)
                {
                    //Do not remove nodes near beacon nodes (including target)
                    bool near_beacon_detected = false;
                    if(childless_node_index.size() == 1) {
                        chosen_index = childless_node_index[0]; //just delete the last remaining node
                        break;
                    }
                    int random_index = randNum(0, childless_node_index.size() - 1);
                    chosen_index = childless_node_index[random_index];
                    for (int ii = 0; ii < Xbeacon_old.size() - 1; ii++)
                    {
                        if (calculateDistance(node[chosen_index], Xbeacon_old[ii]) <= beacon_radius)
                        {
                            childless_node_index.erase(std::remove(childless_node_index.begin(), childless_node_index.end(), chosen_index), childless_node_index.end());
                            near_beacon_detected = true;
                        }
                    }
                    if (!near_beacon_detected)
                        is_near_beacon = false;
                }
                //Replace the removed node with Xnew
                node[chosen_index] = Xnew;
                node_cost[chosen_index] = temp_cost;
                node_parent[chosen_index] = position;

                //Update nodes having Xnew as parent
                std::replace(node_parent.begin(), node_parent.end(), current_no_of_nodes, chosen_index);

                //Remove last element of node
                node.pop_back();
                node_cost.pop_back();
                node_parent.pop_back();
            }
        }

    }// end of search loop

    std::cout << "------------- Search Optimal Path ------------- " << std::endl;
    t_end = ros::WallTime::now();
    std::cout << "Time taken is " << (t_end - t_start).toNSec() * 1e-9 << "s" << std::endl;

    // Track the optimal path (beacon nodes)
    for (int ii = 0; ii < Xbeacon_old.size(); ii++)
    {
        geometry_msgs::PoseStamped pos;
        pos.pose.position.x = ((Xbeacon_old[ii][0] * 0.05) - 10.0);
        pos.pose.position.y = ((Xbeacon_old[ii][1] * 0.05) - 10.0);

        plan.push_back(pos);
    }


    //WRITE TO DATA
    std::ofstream xfile, yfile, zfile, yawfile;
    xfile.open("xfile.txt", std::ios::trunc);
    yfile.open("yfile.txt", std::ios::trunc);
    zfile.open("zfile.txt", std::ios::trunc);
    yawfile.open("yawfile.txt", std::ios::trunc);
    xfile << "easting : [";
    yfile << "northing : [";
    zfile << "height : [";
    yawfile << "heading : [";

    int len = plan.size()-1;
    for (int i = len; i >= 0; i--){
        float x1, y1, x2, y2, slope, rad;
        if (i == 0)
                rad = 0;
        else
        {
                x1 = plan[i].pose.position.x;
                y1 = plan[i].pose.position.y;
                x2 = plan[i - 1].pose.position.x;
                y2 = plan[i - 1].pose.position.y;

                if (x2 == x1)
                        rad = (y2 > y1) ? 1.5708 : -1.5708;
                else
                {
                        slope = (y2 - y1) / (x2 - x1);
                        rad = atan(slope);
                }
        }

        if(i == len){
            xfile << plan[i].pose.position.x;
            yfile << plan[i].pose.position.y;
            zfile << "1.0";
            yawfile << rad*180/M_PI;
        }
        else{
            xfile << " ," << plan[i].pose.position.x;
            yfile << " ," << plan[i].pose.position.y;
            zfile << " ,1.0";
            yawfile << " ," << rad*180/M_PI;
        }
    }

    xfile << "]\n"; yfile << "]\n"; zfile << "]\n"; yawfile << "]\n";
    xfile.close(); yfile.close(); zfile.close(); yawfile.close();

    return plan;
}
