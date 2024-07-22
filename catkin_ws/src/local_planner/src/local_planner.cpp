#include "local_planner/local_planner.h"
#include <pluginlib/class_list_macros.h> // To register the plugin
#include <tf2/utils.h> // For yaw calculations

// ... (constructor, destructor, initialize(), isGoalReached(), setPlan())

PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, base_local_planner::BaseLocalPlanner)

namespace local_planner {

bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // TODO: robot pose from odometry and calculate current robot pose from base_odom_

    // dynamic window
    double min_v = std::max(last_cmd_vel_.linear.x - max_vel_x_ * 0.1, min_vel_x_);
    double max_v = std::min(last_cmd_vel_.linear.x + max_vel_x_ * 0.1, max_vel_x_);
    double min_w = std::max(last_cmd_vel_.angular.z - max_vel_th_ * 0.1, min_vel_th_);
    double max_w = std::min(last_cmd_vel_.angular.z + max_vel_th_ * 0.1, max_vel_th_);

    // generate and evaluate trajectories within the window
    double best_score = -1.0;
    std::vector<geometry_msgs::PoseStamped> best_traj;
    // ... (iterate over possible v, w combinations)
        std::vector<geometry_msgs::PoseStamped> traj;
        generateTrajectory(v, w, 0.1, traj); // Generate trajectory
        double score = calculateObjectiveFunction(traj);
        if (score > best_score) {
            best_score = score;
            best_traj = traj;
        }
    // ...

    // set cmd_vel to the best trajectory's initial velocities
    cmd_vel.linear.x = best_traj[0].pose.position.x;
    cmd_vel.angular.z = tf2::getYaw(best_traj[0].pose.orientation);
    return true;
}

} // namespace local_planner
