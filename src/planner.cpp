#include "planner.h"

using namespace AStar;

Planner::Planner(){
    pubStart_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
    pubpath_ = n_.advertise<visualization_msgs::MarkerArray>("/path", 1);

    subMap_ = n_.subscribe("/map", 1, &Planner::CallbacksetMap, this);
    subGoal_ = n_.subscribe("/move_base_simple/goal", 1, &Planner::CallbacksetGoal, this);
    subStart_ = n_.subscribe("/initialpose", 1, &Planner::CallbacksetStart, this);
};
            
void Planner::CallbacksetMap(const nav_msgs::OccupancyGrid::Ptr map){
    // update the occupacy grid
    grid_ = map;
    edgeLength_ = map->info.resolution;
};

void rotation_mat(float& angle, Eigen::MatrixXd& rot_matrix){
    Eigen::MatrixXd matrix(2,2);
    matrix(0,0) = cos(angle);
    matrix(0,1) = -sin(angle);
    matrix(1,0) = sin(angle);
    matrix(1,1) = cos(angle);
    rot_matrix = matrix;
}

void rotate_counterclockwise(Eigen::VectorXd& vec, Eigen::VectorXd& vec_rotated, float angle){
    Eigen::MatrixXd rotation_matrix;
    rotation_mat(angle, rotation_matrix);
    vec_rotated = rotation_matrix*vec;
}

void Planner::CallbacksetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start_pos){
    float x = start_pos->pose.pose.position.x; // meter
    float y = start_pos->pose.pose.position.y; // meter
    float yaw = tf::getYaw(start_pos->pose.pose.orientation); // radius
    
    // publish the start without covariance for rviz
    geometry_msgs::PoseStamped startN;
    startN.pose.position = start_pos->pose.pose.position;
    startN.pose.orientation = start_pos->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();

    std::cout << "New start x:" << x << " y:" << y << " yaw:" << yaw << std::endl;
    float map_origin_x = grid_->info.origin.position.x;
    float map_origin_y = grid_->info.origin.position.y;
    float map_width = (grid_->info.width)*edgeLength_;
    float map_height = (grid_->info.height)*edgeLength_;
    float map_yaw = tf::getYaw(grid_->info.origin.orientation);
    Eigen::VectorXd v(2);
    Eigen::VectorXd v_rotateback(2);
    v << x-map_origin_x, y-map_origin_y;
    rotate_counterclockwise(v,v_rotateback,-map_yaw);
    // v_rotateback = rotation_matrix*v;
    // std::cout << "v:" << std::endl << v << std::endl;
    // std::cout << "rotation_matrix:" << std::endl << rotation_matrix << std::endl;
    // std::cout << "v_rotateback:" << std::endl << v_rotateback << std::endl;

    if ( v_rotateback(0) <= map_width && v_rotateback(0) >= 0 && 
        v_rotateback(1) <= map_height && v_rotateback(1) >= 0) {
        validStart_ = true;
        start_ = *start_pos;
        plan();
        // publish start for RViz
        pubStart_.publish(startN);
    } else {
        std::cout << "Invalid start x:" << x << " y:" << y << " yaw:" << yaw << std::endl;
    }
};

void Planner::CallbacksetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal_pos){
    //retrieving goal position
    float x = goal_pos->pose.position.x;
    float y = goal_pos->pose.position.y;
    float yaw = tf::getYaw(goal_pos->pose.orientation);

    std::cout << "New goal x:" << x << " y:" << y << " yaw:" << yaw << std::endl;
    float map_origin_x = grid_->info.origin.position.x;
    float map_origin_y = grid_->info.origin.position.y;
    float map_width = (grid_->info.width)*edgeLength_;
    float map_height = (grid_->info.height)*edgeLength_;
    float map_yaw = tf::getYaw(grid_->info.origin.orientation);
    Eigen::VectorXd v(2);
    Eigen::VectorXd v_rotateback(2);
    v << x-map_origin_x, y-map_origin_y;
    rotate_counterclockwise(v,v_rotateback,-map_yaw);

    if (v_rotateback(0) <= map_width && v_rotateback(0) >= 0 && 
        v_rotateback(1) <= map_height && v_rotateback(1) >= 0) {
        validGoal_ = true;
        goal_ = *goal_pos;
        plan();
    } else {
        std::cout << "Invalid goal x:" << x << " y:" << y << " yaw:" << yaw << std::endl;
    }
};


void Planner::plan(){
    if (validStart_ && validGoal_) {
        // Generate spot grid
        int rows = grid_->info.height;
        int cols = grid_->info.width;
        double height = (double)rows * edgeLength_;
        double width = (double)cols * edgeLength_;

        std::cout << "Map info:" << std::endl;
        std::cout << "  height(m): " << height << std::endl;
        std::cout << "  width(m): " << width << std::endl;
        std::cout << "  rows: " << rows << std::endl;
        std::cout << "  columns: " << cols << std::endl;
        std::cout << std::endl;

        float map_origin_x = grid_->info.origin.position.x;
        float map_origin_y = grid_->info.origin.position.y;
        float map_width = (grid_->info.width)*edgeLength_;
        float map_height = (grid_->info.height)*edgeLength_;
        float map_yaw = tf::getYaw(grid_->info.origin.orientation);
        Eigen::VectorXd v(2);
        Eigen::VectorXd v_rotateback(2);
        
        Spot** map = new Spot*[rows];
        for(int j = 0; j < rows; j++){
            map[j] =new Spot[cols];
            for(int i = 0; i < cols; i++){
                bool obstacle = grid_->data[cols * j + i] > 0 ? true : false;

                if(tf::getYaw(grid_->info.origin.orientation) != 0){
                    v << (0.5+i)*edgeLength_, (0.5+j)*edgeLength_;
                    rotate_counterclockwise(v,v_rotateback,map_yaw);
                    map[j][i] = Spot(i, j, v_rotateback(0)+map_origin_x, v_rotateback(1)+map_origin_y, obstacle, edgeLength_);
                }
                else{
                    map[j][i] = Spot(i, j, obstacle, edgeLength_);
                }
            }
        }
        // for(int j = rows-1; j >= 0; j--){
        //     for(int i = 0; i < cols; i++){
        //         if(map[j][i].getwall()){
        //             std::cout << "\033[31m" << map[j][i].getwall() << "\033[0m";
        //         }
        //         else{
        //             std::cout << "0";
        //         }
        //     }
        //     std::cout << std::endl;
        // }

        //retreive start 
        v << (start_.pose.pose.position.x - map_origin_x), (start_.pose.pose.position.y - map_origin_y);
        rotate_counterclockwise(v,v_rotateback,-map_yaw);
        int i_s = floor(v_rotateback(0)/edgeLength_);
        int j_s = floor(v_rotateback(1)/edgeLength_);
        Spot* start_pos = &(map[j_s][i_s]);
        std::cout << "start (i,j) =  (" << start_pos -> geti() << ", " << start_pos -> getj() << ") : " << start_pos->getwall() << std::endl;

        //retreive goal
        v << (goal_.pose.position.x - map_origin_x), (goal_.pose.position.y - map_origin_y);
        rotate_counterclockwise(v,v_rotateback,-map_yaw);
        int i_g = floor(v_rotateback(0)/edgeLength_);
        int j_g = floor(v_rotateback(1)/edgeLength_);
        Spot* goal_pos = &map[j_g][i_g];
        std::cout << "goal (i,j) = (" << goal_pos -> geti() << ", " << goal_pos -> getj() << ") : " << goal_pos->getwall() << std::endl;
        std::cout << std::endl;
        
        // Find solution
        Algorithm astar;
        bool sol = astar.Astar(start_pos, goal_pos, map, rows, cols);

        // Draw path
        if(sol){
            std::cout << "\033[33m" << "Find a solution (x,y):" << std::endl;
            visualize_path(goal_pos, map);
        }
        else{
            std::cout << "\033[31m" << "Did not find a solution" << "\033[0m" << std::endl;
        }
    }
};

void Planner::visualize_path(Spot* goal, Spot** map){
    visualize_clear();
    visualization_msgs::Marker pathNode;
    Spot* node = goal;
    int i = 0;
    while(node != nullptr){
        i++;
        std::cout << "(" << node->getx() << ", " << node->gety() << ")" << std::endl;
        pathNode.header.frame_id = "path";
        pathNode.header.stamp = ros::Time::now();
        pathNode.header.seq = i;
        pathNode.id = i;
        pathNode.type = visualization_msgs::Marker::CUBE;
        pathNode.lifetime = ros::Duration(0);
        pathNode.scale.x = edgeLength_;
        pathNode.scale.y = edgeLength_;
        pathNode.scale.z = 0.5;
        pathNode.color.r = 1.0;
        pathNode.color.g = 0.0;
        pathNode.color.b = 0.0;
        pathNode.color.a = 1.0;
        pathNode.pose.position.x = node->getx();
        pathNode.pose.position.y = node->gety();
        if(tf::getYaw(grid_->info.origin.orientation) != 0){
            pathNode.pose.orientation = grid_->info.origin.orientation;
        }
        else{
            pathNode.pose.orientation.x = 1;
        }
        path_.markers.push_back(pathNode);
        node = node->getprev();
    }
    pubpath_.publish(path_);
    std::cout << "\033[0m" << std::endl;
}

void Planner::visualize_clear(){
    visualization_msgs::Marker node;
    // CLEAR THE PATH
    path_.markers.clear();
    node.header.frame_id = "path";
    node.header.stamp = ros::Time::now();
    node.id = 0;
    node.action = 3;
    path_.markers.push_back(node);
    pubpath_.publish(path_);
    path_.markers.clear();
}