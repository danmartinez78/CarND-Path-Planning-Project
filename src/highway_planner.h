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
    double max_speed_mph = 49.5;
    double max_speed_meters = max_speed_mph/2.237;

    void KeepLane();
    void ChangeLane(); 
    void PrepareLaneChange();
    void Predict();
    void PlanBehavior();
    double distance(double, double, double, double);
    std::vector<std::vector<double>> ToLocalFrame(double, double, double, std::vector<std::vector<double>>);
    std::vector<std::vector<double>> ToGlobalFrame(double, double, double, std::vector<std::vector<double>>);
    std::pair<std::vector<double>, std::vector<double>> SplitPath(std::vector<Point>);
    std::vector<Point> JoinPath(std::vector<double>, std::vector<double>);
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

std::vector<std::vector<double>> HighwayPlanner::ToLocalFrame(double x_ref, double y_ref, double yaw_ref, std::vector<std::vector<double>> path){
    std::vector<double> x_pts = path[0];
    std::vector<double> y_pts = path[1];
    int prev_path_size = x_pts.size();
    for (int i = 0;i<prev_path_size;i++){
        double shift_x = x_pts[i] - x_ref;
        double shift_y = y_pts[i] - y_ref;
        x_pts[i] = (shift_x*cos(0-yaw_ref)-shift_y*sin(0-yaw_ref));
        y_pts[i] = (shift_x*sin(0-yaw_ref)-shift_y*cos(0-yaw_ref)); 
    }
    std::vector<std::vector<double>> transformed_path;
    transformed_path.push_back(x_pts);
    transformed_path.push_back(y_pts);
    return transformed_path;
};


bool HighwayPlanner::Plan()
{
    PlanBehavior();
    switch(current_state){
        case State::KEEP_LANE:
            KeepLane();
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
            break;
    };

    return true;
};

void HighwayPlanner::KeepLane(){
    // drive safely in current lane

    // compute desired speed
    double desired_speed = max_speed_meters;
    double desired_dist_inc = desired_speed/50;

    std::vector<double> next_x;
    std::vector<double> next_y;
    int prev_path_size = prev_path.size(); // size of left over path

    // preserve remnant of previous path
    for(int i =0; i<prev_path_size;i++){
        Point pt = prev_path[i];
        next_x.push_back(pt.x);
        next_y.push_back(pt.y);
    }
    
    double starting_yaw = deg2rad(pose.yaw);
    std::vector<double> spline_pts_x;
    std::vector<double> spline_pts_y;

    if (prev_path_size < 2){
        Point pt;
        spline_pts_x.push_back(pose.x - cos(deg2rad(pose.yaw)));
        spline_pts_y.push_back(pose.y - sin(deg2rad(pose.yaw)));
        spline_pts_x.push_back(pose.x);
        spline_pts_y.push_back(pose.y);
        starting_yaw = pose.yaw;
    }else{
        Point pt, prev_pt;
        pt = prev_path[prev_path_size-1];
        prev_pt = prev_path[prev_path_size-2];
        starting_yaw = atan2(pt.y - prev_pt.y, pt.x - prev_pt.x);
        spline_pts_x.push_back(prev_pt.x);
        spline_pts_x.push_back(pt.x);
        spline_pts_y.push_back(prev_pt.y);
        spline_pts_y.push_back(pt.y);
    }

    for (int dist = 30;dist<100;dist+=30){
        vector<double> wp = getXY(pose.s+dist,(2+4*current_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        spline_pts_x.push_back(wp[0]);
        spline_pts_x.push_back(wp[1]);
    }

    // transform to local frame
    std::vector<std::vector<double>> path_to_transform;
    path_to_transform.push_back(spline_pts_x);
    path_to_transform.push_back(spline_pts_y);
    auto transformed_path = ToLocalFrame(spline_pts_x[spline_pts_x.size()-1], spline_pts_y[spline_pts_y.size()-1], starting_yaw, path_to_transform);
     
     // create spline
    tk::spline s;

    // set spline points
    s.set_points(transformed_path[0], transformed_path[1]);
    
    // interpolate points corresponding to desired speed
    double look_ahead_x = 30.0; // meters
    double look_ahead_y = s(look_ahead_x);
    double look_ahead_dist = sqrt(look_ahead_x*look_ahead_x+look_ahead_y*look_ahead_y);
    double increment = 0;


    for (int i = 0; i < 50 - prev_path_size; ++i)
    {
        double N = (look_ahead_dist)/(0.2*max_speed_meters);
        double next_x_pt = increment+look_ahead_x/N;
        double next_y_pt = s(next_x_pt);
        increment = next_x_pt;
        double x_ref = next_x_pt;
        double y_ref = next_y_pt;

        next_x_pt = (x_ref*cos(starting_yaw)-y_ref*sin(starting_yaw));
        next_y_pt = (x_ref*sin(starting_yaw)-y_ref*cos(starting_yaw));
        next_x_pt += x_ref;
        next_y_pt += y_ref;
        next_x.push_back(next_x_pt);
        next_y.push_back(next_y_pt);
    }
    next_path = JoinPath(next_x, next_y);
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

void HighwayPlanner::PlanBehavior(){
    // given current ego vehicle state and observations/prediction about surrounding cars, decide next action/state in FSM
    current_state = State::KEEP_LANE;
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
    int prev_path_size = last_path_x.size();
    for (int i = 0; i < prev_path_size; i++)
    {
        Point pt;
        pt.x = last_path_x[i];
        pt.y = last_path_y[i];
        prev_path.push_back(pt);
    }
};

std::pair<std::vector<double>, std::vector<double>> HighwayPlanner::SplitPath(std::vector<Point> path){
    std::vector<double> x_pts;
    std::vector<double> y_pts;
    for(auto pt:path){
        x_pts.push_back(pt.x);
        y_pts.push_back(pt.y);
    }
    return std::make_pair(x_pts, y_pts);
};

std::vector<HighwayPlanner::Point> HighwayPlanner::JoinPath(std::vector<double> x_pts, std::vector<double> y_pts){
    std::vector<Point> path;
    int size = x_pts.size();
    for(int i=0;i<size;i++){
        Point pt{x_pts[i], y_pts[i]};
        path.push_back(pt);
    }
    return path;
};

#endif