#include "global_planner/global_planner.h"
#include <pluginlib/class_list_macros.h>

#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)


namespace global_planner {

GlobalPlanner::GlobalPlanner() {}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
        costmap_ros_(NULL), initialized_(false) {
    initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ros_ = costmap_ros;
        initialized_ = true;

        ros::NodeHandle private_nh("~/" + name);

        // Get parameters (adjust as needed)
        private_nh.param("step_size", step_size_, 0.5);
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.1);
        private_nh.param("num_paths_to_generate", num_paths_to_generate_, 3); 
    }
}

std::vector<PathAndScore> GlobalPlanner::generateNPaths(const geometry_msgs::PoseStamped& start,
                                                        const geometry_msgs::PoseStamped& goal) {
    std::vector<PathAndScore> paths;
    std::priority_queue<PathAndScore> pq;  // Priority queue to store potential paths

    // Initialize with starting pose and 0 cost
    pq.push({{start}, 0});

    while (!pq.empty() && paths.size() < num_paths_to_generate_) {
        PathAndScore current = pq.top(); pq.pop();

        if (isGoal(current.path.back(), goal)) {
            paths.push_back(current);
        } else {
            for (auto neighbor : getNeighbors(current.path.back())) {
                double new_score = current.score + costToNeighbor(neighbor);
                pq.push({current.path + {neighbor}, new_score + heuristic(neighbor, goal)}); 
            }
        }
    }
    return paths;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
>& plan) {

    // ... (Basic sanity checks, ensuring the planner is initialized, etc.)

    std::vector<PathAndScore> paths = generateNPaths(start, goal);

    if (paths.empty()) {
        ROS_WARN("No path found.");
        return false;
    }

    // Choose the best path based on score (or other criteria)
    auto best_path = std::min_element(paths.begin(), paths.end());
    plan = best_path->path;

    // Optionally, publish all generated paths for visualization
    // ...

    return true;
& goal) {
    return hypot(pose.pose.position.x - goal.pose.position.x,
                 pose.pose.position.y - goal.pose&> neighbors;
    
    // ... (Define how to get neighboring poses, e.g., 8-connected grid)

    for (double dx : {-step_size_, 0, step_size_}) {
        for (double dy : {-step_size_, 0, step_size_}) {
            if (dx == 0 && dy == 0) continue; new_pose = pose;
            new_pose.pose.position.x += dx;
            new_pose.pose.position.y += dy;

            // Check if new pose is valid (not in collision, within bounds)
            if (costmap_ros_->getCostmap()->worldToMap(new_pose.pose.position.x, new_pose.pose.position.y, 
                                                      new_pose.pose.position.x, new_pose.pose.position.y)) {
                neighbors.push_back(new_pose);
            }
        }
    }
    return neighbors;
}
