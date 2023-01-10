//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Oscar
//  WRITTEN:  2023-01-09
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "planner.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "a_star");

    AStar::Planner astar_planning;
    astar_planning.plan(); 

    ros::spin();
    return 0;
}