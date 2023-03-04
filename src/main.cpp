//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Oscar
//  WRITTEN:  2023-01-09
//###################################################

#include "planner.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "a_star");

    AStar::Planner astar_planning;
    ros::Rate loop_rate(5);
    while(ros::ok()){
        astar_planning.plan(); 
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}