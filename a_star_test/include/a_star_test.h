#ifndef _A_STAR_TEST_H
#define _A_STAR_TEST_H

#include <iostream>
#include <string>
#include "math.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

#include <vector>
#include <functional>
#include <set>
#include"visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "time.h"
#include "ros/console.h"


#define SEARCH_STEP   1

#define SEARCH_THRESHOLD  SEARCH_STEP * 2

struct Node{
int G;
int H;
int F;

int index;
int x_index;
int y_index;

Node * prev_node = nullptr; 

bool visiable; //0-free 100-occupy -1-unkown

};






class AStarTest{


    
public:

    //AStarTest();

    AStarTest(ros::NodeHandle* m_nh);

    ~AStarTest();

    bool get_node(geometry_msgs::PoseStamped& p_point, Node& p_node);
    bool get_node(int x_index, int y_index, Node& p_node);

    bool path_search( geometry_msgs::PoseStamped& start,  geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr p_map);

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr m_goal);

    bool is_free(Node* test_node);

    bool find_node(std::vector<Node*>nodelist, Node* t_node);

    void realsenode(std::vector<Node*>& list_node);

private:

    nav_msgs::OccupancyGrid m_map_;

    std::vector<int8_t> grid_data;
    //std::vector<uint8_t> grid_data;

    std::vector<Node*> openlist;
    std::vector<Node*> closelist;

    Node p_start ;
    Node p_goal ;

    ros::Subscriber map_sub;
    ros::Subscriber goal_sub;

    ros::Publisher m_pub;
    ros::Publisher path_pub;
    visualization_msgs::Marker m_path;

    uint32_t num_map_;

    geometry_msgs::PoseStamped start_point_;
    geometry_msgs::PoseStamped goal_point_;

    float resolution_;

};

















#endif