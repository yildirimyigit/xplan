#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <string>
#include <queue> // For priority queue in A*

namespace global_planner {

struct PathAndScore {
    std::vector<geometry_msgs::PoseStamped> path;
    double score;

    bool operator<(const PathAndScore& other) const {
        return score > other.score;
    }
};

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // BaseGlobalPlanner overwrite
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    bool initialized_;
    int num_paths_to_generate_;

    // A* Helpers
    double heuristic(const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped& goal);
    std::vector<geometry_msgs::PoseStamped> getNeighbors(const geometry_msgs::PoseStamped& pose);

    std::vector<PathAndScore> generateNPaths(const geometry_msgs::PoseStamped& start,
                                             const geometry_msgs::PoseStamped& goal);
};

}

#endif
