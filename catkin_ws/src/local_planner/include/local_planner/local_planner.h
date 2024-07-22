#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>
#include <base_local_planner/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace local_planner {

class LocalPlanner : public base_local_planner::BaseLocalPlanner {
public:
    LocalPlanner();
    ~LocalPlanner();
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf::TransformListener* tf_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry base_odom_; // last odometry
    geometry_msgs::Twist last_cmd_vel_; // last cmd_vel
    double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_;

    // helper functions
    void generateTrajectory(double v, double w, double dt, std::vector<geometry_msgs::PoseStamped>& traj);
    double calculateObjectiveFunction(const std::vector<geometry_msgs::PoseStamped>& traj);
};

}

#endif
