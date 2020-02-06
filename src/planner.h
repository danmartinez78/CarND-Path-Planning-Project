#ifndef PLANNER_CLASS
#define PLANNER_CLASS

#include "tinyfsm.hpp" // finite state machine library
#include "spline.h"    // spline library
#include "helpers.h"   // helper functions

class HighwayPlanner
{

public:
    HighwayPlanner(std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s,
    std::vector<double> map_waypoints_dx,
    std::vector<double> map_waypoints_dy) : map_waypoints_x{map_waypoints_x},
    map_waypoints_y{map_waypoints_y},
    map_waypoints_s{map_waypoints_s},
    map_waypoints_dx{map_waypoints_dx},
    map_waypoints_dy{map_waypoints_dy} {}

    ~HighwayPlanner() {}
    void SetPose(double x, double y, double s, double d, double yaw, double speed);

    struct Pose
    {
        double x = 0;
        double y = 0;
        double s = 0;
        double d = 0;
        double yaw = 0;
        double speed = 0;
        double s_dot = 0;
        double s_dot_dot = 0;
        double d_dot = 0;
        double d_dot_dot = 0;
    };

private:
    /* data */

    HighwayPlanner::Pose pose;
    // TODO: state machine

    // trajs
    tk::spline old_traj;
    tk::spline new_traj;

    // map info
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
    double max_s = 6945.554;

    // dynamic obstacles
};

void HighwayPlanner::SetPose(double x, double y, double s, double d, double yaw, double speed){
    pose.x = x;
    pose.y = y;
    pose.s = s;
    pose.d = d;
    pose.yaw = yaw;
    pose.speed = speed;
}

#endif