#ifndef HIGHWAY_PLANNER_CLASS
#define HIGHWAY_PLANNER_CLASS

#include "behavior_planner.h" // behavior planner
#include "spline.h"           // spline library
#include "helpers.h"          // helper functions

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
    bool Plan();
    std::pair<std::vector<double>, std::vector<double>> GetPlannedPath();
    void SetPrevPath(std::vector<double> last_path_x, std::vector<double> last_path_y);

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

    struct Point
    {
        double x;
        double y;
    };

    enum class State{
        KEEP_LANE, PLCL, PLCR, LCL, LCR
    };

private:
    /* data */

    HighwayPlanner::Pose pose;
    
    // TODO: simple state machine
 
    State current_state = State::KEEP_LANE;
    State next_state = State::KEEP_LANE;

    // trajs
    tk::spline prev_spline;
    tk::spline next_spline;

    // paths
    std::vector<Point> prev_path;
    std::vector<Point> next_path;

    // map info
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
    double max_s = 6945.554;

    // dynamic obstacles
    std::vector<std::vector<double>> car_poses;
    std::vector<std::vector<double>> predicted_car_poses;

    // other info
    int desired_lane = 0;
    int current_lane = 0;
    double max_jerk = 0;
    double max_acc = 0;
    double max_v = 0;

    void KeepLane();
    void ChangeLane(); 
    void PrepareLaneChange();
    void Predict();
    void Decide();

};

void HighwayPlanner::SetPose(double x, double y, double s, double d, double yaw, double speed)
{
    pose.x = x;
    pose.y = y;
    pose.s = s;
    pose.d = d;
    pose.yaw = yaw;
    pose.speed = speed;
};

bool HighwayPlanner::Plan()
{
    switch(state){
        case State::KEEP_LANE:
            // blah
            break;
        case State::PLCL:
            // blah
            break;
        case State::PLCR:
            // blah
            break;
        case State::LCL:
            // blah
            break;
        case State::LCR:
            // blah
            break;
        default:
            // blah
    }

    // double pos_x;
    // double pos_y;
    // double angle;
    // int path_size = prev_path.size();
    // next_path.clear();

    // for (int i = 0; i < path_size; ++i)
    // {
    //     Point pt;
    //     pt.x = prev_path[i].x;
    //     pt.y = prev_path[i].y;
    //     next_path.push_back(pt);
    // }

    // if (path_size == 0)
    // {
    //     pos_x = pose.x;
    //     pos_y = pose.y;
    //     angle = deg2rad(pose.yaw);
    // }
    // else
    // {
    //     pos_x = prev_path[path_size - 1].x;
    //     pos_y = prev_path[path_size - 1].y;

    //     double pos_x2 = prev_path[path_size - 2].x;
    //     double pos_y2 = prev_path[path_size - 2].y;
    //     angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
    // }

    // double dist_inc = 0.5;
    // for (int i = 0; i < 50 - path_size; ++i)
    // {
    //     Point pt;
    //     pt.x = pos_x + (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
    //     pt.y = pos_y + (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
    //     next_path.push_back(pt);
    //     pos_x += (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
    //     pos_y += (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
    // }
    // prev_path = next_path;
    // return true;
};

void HighwayPlanner::KeepLane(){
    // drive safely in current lane

};

void HighwayPlanner::ChangeLane(){
    // given target lane and current lane, plan smooth trajectory to change lanes

};

void HighwayPlanner::PrepareLaneChange(){
    // given target ane and current lane, plan trajectory to facilitate lane change

};

void HighwayPlanner::Predict(){
    // given current observations, predict motorist behavior (SIMPLE)

};

void HighwayPlanner::Decide(){
    // given current ego vehicle state and observations/prediction about surrounding cars, decide next action/state in FSM

};

std::pair<std::vector<double>, std::vector<double>> HighwayPlanner::GetPlannedPath()
{
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    for (auto pt : next_path)
    {
        next_x_vals.push_back(pt.x);
        next_y_vals.push_back(pt.y);
    }
    return std::make_pair(next_x_vals, next_y_vals);
};

void HighwayPlanner::SetPrevPath(std::vector<double> last_path_x, std::vector<double> last_path_y)
{
    this->prev_path.clear();
    int path_size = last_path_x.size();
    for (int i = 0; i < path_size; i++)
    {
        Point pt;
        pt.x = last_path_x[i];
        pt.y = last_path_y[i];
        prev_path.push_back(pt);
    }
};

#endif