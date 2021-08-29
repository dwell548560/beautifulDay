#include "a_star_test.h"



AStarTest::~AStarTest()
{
    
}


void AStarTest::map_callback(const nav_msgs::OccupancyGrid::ConstPtr p_map)
{

   ROS_INFO("ENTER MAP TOPIC");

   m_map_ = *p_map;

   num_map_ = p_map->info.width * p_map->info.height;

   std::cout<<"num_map"<<num_map_<<std::endl;
   std::cout<<"p_map->info.width"<<p_map->info.width<<std::endl;
   std::cout<<"p_map->info.height"<<p_map->info.height<<std::endl;

   resolution_ = p_map->info.resolution;

   grid_data = p_map->data;
   //std::cout<<"resolution"<<resolution_<<std::endl;

  //std::cout<<"grid data"<< (int8_t)(m_map_.data[1])<<std::endl;

    // for(int i = 0; i < 8486912; i++)
    // {

    //     std::cout<<"grid data"<<i<< m_map_.data[i]<<std::endl;
    // }



}

AStarTest::AStarTest(ros::NodeHandle * m_nh)
{
    ROS_INFO("constructor function");
    ros::NodeHandle param_nh("~");

    double yaw_type;
    geometry_msgs::Quaternion quat;


    map_sub = m_nh->subscribe("map", 1000, &AStarTest::map_callback, this);//
    goal_sub = m_nh->subscribe("move_base_simple/goal", 10, &AStarTest::goal_callback, this);
    m_pub = m_nh->advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    path_pub = m_nh->advertise<nav_msgs::Path>("a_path", 10);

    param_nh.getParam("start_point_x", start_point_.pose.position.x);
    param_nh.getParam("start_point_y", start_point_.pose.position.y);
    param_nh.getParam("start_point_z", start_point_.pose.position.z);
    param_nh.getParam("angle", yaw_type);

    quat = tf::createQuaternionMsgFromYaw(yaw_type);
    start_point_.pose.orientation.x = quat.x;
    start_point_.pose.orientation.y = quat.y;
    start_point_.pose.orientation.z = quat.z;
    start_point_.pose.orientation.w = quat.w;

    std::cout<<"start_point_x"<< start_point_.pose.position.x<<std::endl;
    std::cout<<"start_point_y"<< start_point_.pose.position.y<<std::endl;
   
   // std::cout<<"angle"<<yaw_type<<std::endl;


}




void AStarTest::goal_callback(const geometry_msgs::PoseStamped::ConstPtr m_goal)
{

    nav_msgs::Path path_node;
    geometry_msgs::PoseStamped cur_pose;
    ros::Time cur_time;

    ROS_INFO("SUB GOAL");

    std::vector<geometry_msgs::PoseStamped> move_plan;

    goal_point_ = *m_goal;
   
    std::cout<<"x is"<<goal_point_.pose.position.x<<std::endl;
    std::cout<<"y is"<<goal_point_.pose.position.y<<std::endl;

    get_node(goal_point_, p_goal);

    auto search_result =  path_search(start_point_, goal_point_, move_plan);

   if(search_result == true)//准备发布path
   {
       auto it = &p_goal;//从目标点开始,通过前驱节点回到起始点 将路径打印发布出来
       while(it != nullptr)
       {
           
            cur_time = ros::Time::now();
            Node *now_node;
            now_node = it ;


            path_node.header.stamp = cur_time;
            path_node.header.frame_id = "/map";

            cur_pose.header.stamp = cur_time;
            cur_pose.pose.position.x = now_node->x_index * resolution_ + m_map_.info.origin.position.x;//根据索引求map下实际坐标
            cur_pose.pose.position.y = now_node->y_index * resolution_ + m_map_.info.origin.position.y;
            cur_pose.pose.position.z = 0;

            path_node.poses.push_back(cur_pose);
            path_pub.publish(path_node);

            it = it -> prev_node;//取出该点得父节点

            //std::cout<<"x is "<<cur_pose.pose.position.x<<std::endl;
            //std::cout<<"y is "<<cur_pose.pose.position.y<<std::endl;

       }

       realsenode(openlist);
       realsenode(closelist);//将点从列表里删除，为下一次规划准备
   }
   else
   {
       realsenode(openlist);//将点从列表里删除，为下一次规划准备
       realsenode(closelist);//将点从列表里删除，为下一次规划准备

       ROS_INFO("Search Fail !!!");
   }
   
   
}


bool AStarTest::get_node(int x_index, int y_index, Node& p_node)
{
    p_node.x_index = x_index;
    p_node.y_index = y_index;

    auto index_t = p_node.x_index + p_node.y_index * m_map_.info.width;
    //auto index_t = p_node.x_index * m_map_.info.width + p_node.y_index;
    p_node.index = index_t;
    if( m_map_.data[index_t] == 0)
    {
        p_node.visiable = 1;//free
        //ROS_INFO("Free");

    }
    else 
    {
        p_node.visiable = 0;//occupy or unkown
        //ROS_INFO("Occupy Grid");
    }



p_node.H = abs((abs(goal_point_.pose.position.y - m_map_.info.origin.position.y) / resolution_)- p_node.y_index) +\
abs((abs(goal_point_.pose.position.x - m_map_.info.origin.position.x) / resolution_)- p_node.x_index);

//std::cout<<"p_node.H"<<p_node.H<<std::endl;
}


bool AStarTest::get_node(geometry_msgs::PoseStamped& p_point, Node& p_node)
{

    p_node.x_index = abs(p_point.pose.position.x - m_map_.info.origin.position.x) / resolution_;
    p_node.y_index = abs(p_point.pose.position.y - m_map_.info.origin.position.y) / resolution_;

    auto index_t =  p_node.x_index + p_node.y_index * m_map_.info.width;
    //auto index_t = p_node.x_index * m_map_.info.width + p_node.y_index;
    p_node.index = index_t;
    if( m_map_.data[index_t] == 0)
    {
        p_node.visiable = 1;//free
        //ROS_INFO("Free");

    }
    else 
    {
        p_node.visiable = 0;//occupy or -1unkown
        //ROS_INFO("Occupy Grid");
    }




p_node.H = abs((abs(goal_point_.pose.position.y - m_map_.info.origin.position.y) / resolution_)- p_node.y_index) +\
abs((abs(goal_point_.pose.position.x - m_map_.info.origin.position.x) / resolution_)- p_node.x_index);

std::cout<<"p_node.H"<<p_node.H<<std::endl;
return 0;
}


//find is or not free grid
bool AStarTest::is_free(Node* test_node)
{
    if(test_node->visiable == 1)return true;
    else if(test_node->visiable == 0)return false;


}



bool AStarTest::find_node(std::vector<Node*>nodelist, Node* t_node)
{
    int num;
    for(auto node : nodelist )
    {
        if(node->x_index == t_node->x_index && node->y_index == t_node->y_index)
        {
            // num++;
            // std::cout<<"num"<<num<<std::endl;
            // std::cout<<"node->x_index"<<node->x_index<<std::endl;
            // std::cout<<"nt_node->x_index"<<t_node->x_index<<std::endl;
            // std::cout<<"node->y_index"<<node->y_index<<std::endl;
            // std::cout<<"t_node->y_index"<<t_node->y_index<<std::endl;
            return true;
        }
       
    }
    return false;
}



bool AStarTest::path_search( geometry_msgs::PoseStamped& start, geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    // nav_msgs::Path path_node;
    // geometry_msgs::PoseStamped cur_pose;
    // ros::Time cur_time;


    Node v_node;
    get_node(start, p_start);
    
    //std::cout<<"v_node.x_index"<<v_node.x_index<<std::endl;
    //std::cout<<"v_node.y_index"<<v_node.y_index<<std::endl;
    p_start.G = 0;
    p_start.F = p_start.G + p_start.H;
    p_start.prev_node = nullptr;

    openlist.push_back(&p_start);

    while(openlist.size() != 0)
    {
        Node* curr_ptr = openlist.front();//

        //find min F node 获取此时开发列表里F最小的节点
        for(std::vector<Node*>::iterator node = openlist.begin(); node != openlist.end(); node++)
        {
            if(curr_ptr == nullptr)
            {  
                curr_ptr = *node;
            }
            else if(curr_ptr->F >= (*node)->F)
            {
                curr_ptr = *node;

            }
        }

        //cur_time = ros::Time::now();
        //最小的F节点从开发列表里删除加入到关闭列表里
        openlist.erase(std::find(openlist.begin(), openlist.end(), curr_ptr));
        closelist.push_back(curr_ptr);

        // path_node.header.stamp = cur_time;
        // path_node.header.frame_id = "/map";

        // cur_pose.header.stamp = cur_time;
        // cur_pose.pose.position.x = curr_ptr->x_index * resolution_ + m_map_.info.origin.position.x;
        // cur_pose.pose.position.y = curr_ptr->y_index * resolution_ + m_map_.info.origin.position.y;
        // cur_pose.pose.position.z = 0;

        // path_node.poses.push_back(cur_pose);

        //path_pub.publish(path_node);


        //这里定义到达目标点的精度标准
        if(curr_ptr->index <= abs(p_goal.index + SEARCH_THRESHOLD)  &&  curr_ptr->index >= abs(p_goal.index - SEARCH_THRESHOLD))
        {


            int delta_x, delta_y;

            delta_x = p_goal.x_index - curr_ptr->x_index;
            delta_y = p_goal.y_index - curr_ptr->y_index;

            std::cout<<"delta_x is "<<delta_x<<std::endl;//计算到达精度
            std::cout<<"delta_y is"<<delta_y<<std::endl;

            p_goal.prev_node = curr_ptr;//目标点的前驱点设置为现在的点


            closelist.push_back(&p_goal);//将目标点放入关闭队列  搜索结束

            ROS_INFO("Search Finish");
            return true;
         
        }


       //开始搜索该节点周围得8个栅格，周围8个格子考虑效率 步长为5个栅格
        for(int i = -SEARCH_STEP; i < SEARCH_STEP + 1; i = i + SEARCH_STEP)
        {
            for(int j = -SEARCH_STEP; j < SEARCH_STEP + 1; j = j + SEARCH_STEP)
            {
                if(i == 0 && j == 0)// current//自己不算在内
                {
                    continue;
                }

                Node* s_node = new Node;
                //std::shared_ptr<Node> s_node( new Node) ;
                get_node((curr_ptr->x_index) + i, (curr_ptr->y_index) + j, *s_node);
               // ROS_INFO("PATH_SEARCH RUN");

                if(!is_free(s_node) || find_node(closelist, s_node))//如果本栅格不可走或在关闭列表里就结束本次for
                {
                    continue;

                }


                
                /*这里要注意，代价的计算应该与实际的步长一致，不能随便说是10，14, 因为还有一个曼哈段距离H,H的计算是根据实际步长得来的，F的值是G+H, 
                这样两个的计算就不是基于给的步长这个尺度了，所以前后左右的是一个步长，对角线的是1.4个步长（即根号2个步长）*/
                int G_cost = curr_ptr->G + ((i != 0 && j != 0) ? ((float)SEARCH_STEP * 1.4) : SEARCH_STEP);
                //std::cout<<"G_cost"<<G_cost<<std::endl;

                bool f_node = find_node(openlist, s_node);//该栅格是不是在开放列表
                //std::cout<<"F_Node is "<<f_node<<std::endl;
                if(f_node == false)//如果不在开放列表,计算F ,并将本节点设为父节点
                {
                    s_node->G = G_cost;
                    s_node->F = s_node->G + s_node->H;
                    s_node->prev_node = curr_ptr;

                    openlist.push_back(s_node);
                   // ROS_INFO("Add Node");
                }
                else if(G_cost < s_node->G)//如果已经开放列表,current_ptr这个点到达起始点得G更小，就更新G，并从新计算F，然后前驱点设置为current_ptr
                {
                    s_node->G = G_cost;
                    s_node->F = s_node->G + s_node->H;
                    s_node->prev_node = curr_ptr;
                }
            }

        }
    } 

    
    for(std::vector<Node*>::iterator node = closelist.begin(); node != closelist.end(); node++)
    {

        std::cout<<"x_index is:"<<(*node)->x_index<<"y_index is:"<<(*node)->y_index<<std::endl;

    }

    return false;   
}


//
 void AStarTest::realsenode(std::vector<Node*>& list_node)
 {
    
     

     for(auto it = list_node.begin(); it != list_node.end();it++)//注意这里不要使用++，erase本身就返回下一个的迭代器
     {
         //delete *it;

        it = list_node.erase(it);
        
     }

 }







int main(int argc, char * argv[])
{

    ros::init(argc, argv, "a_star_test");

    ros::NodeHandle nh;

    AStarTest m_a_star(&nh);

    ROS_INFO("Start ROS");
    ros::Rate rate(2);

    while(ros::ok())
    {



        

        ros::spinOnce();

        rate.sleep();
        

    }

    return 0;
}