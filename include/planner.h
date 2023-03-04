#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>
#include <vector>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/Lane.h>
#include "spot.h"
#include "algorithm.h"

namespace AStar{

    class Planner{
        private:
            /// The node handle
            ros::NodeHandle n_;
            /// A publisher publishing the start position for RViz
            ros::Publisher pubStart_;
            /// A subscriber for receiving map updates
            ros::Subscriber subMap_;
            /// A subscriber for receiving goal updates
            ros::Subscriber subGoal_;
            /// A subscriber for receiving start updates
            ros::Subscriber subStart_;
            // /// A listener that awaits transforms
            // tf::TransformListener listener;
            // /// A transform for moving start positions
            // tf::StampedTransform transform;
            /// A publisher publishing the path for RViz
            ros::Publisher pubpath_;
            // A publisher publishing the path for waypoint
            ros::Publisher lane_pub_;            
            /// The path produced by the A* algorithm
            visualization_msgs::MarkerArray path_;
            /// A pointer to the grid the planner runs on
            nav_msgs::OccupancyGrid::Ptr grid_;
            /// The start pose set through RViz
            geometry_msgs::PoseWithCovarianceStamped start_;
            /// The goal pose set through RViz
            geometry_msgs::PoseStamped goal_;
            /// Flags for allowing the planner to plan
            bool validStart_ = false;
            /// Flags for allowing the planner to plan
            bool validGoal_ = false;
            /// length of a cell's edge (meter)
            double edgeLength_ = 2.8;

        public:
            /// The default constructor
            Planner();

            void CallbacksetMap(const nav_msgs::OccupancyGrid::Ptr map);

            void CallbacksetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

            void CallbacksetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

            void plan();

            void visualize_path(Spot* goal, Spot** map);

            void visualize_clear();
            
            void createWayPoint(Spot* goal);
    };
}
#endif // PLANNER_H