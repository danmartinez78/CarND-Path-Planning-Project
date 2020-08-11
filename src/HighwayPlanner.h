#ifndef HIGHWAY_PLANNER_CLASS
#define HIGHWAY_PLANNER_CLASS

#include "spline.h"  // spline library
#include <vector>
#include <math.h>
#include <iostream>

class HighwayPlanner
{

public:
    enum class BehaviorState
    {
        KEEPLANE,
        PLCL,
        PLCR,
        LCL,
        LCR
    };

    struct State
    {
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        double accel;
    };
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

    double getdeg2rad(double x) { return x * M_PI / 180; }
    double getrad2deg(double x) { return x * 180 / M_PI; }

    // Calculate distance between two points
    double calc_distance(double x1, double y1, double x2, double y2)
    {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    // Calculate closest waypoint to current x, y position
    int GetClosestWaypoint(double x, double y)
    {
        double closestLen = 100000; //large number
        int closestWaypoint = 0;

        for (int i = 0; i <  map_waypoints_x.size(); ++i)
        {
            double map_x =  map_waypoints_x[i];
            double map_y =  map_waypoints_y[i];
            double dist = calc_distance(x, y, map_x, map_y);
            if (dist < closestLen)
            {
                closestLen = dist;
                closestWaypoint = i;
            }
        }

        return closestWaypoint;
    }

    // Returns next waypoint of the closest waypoint
    int NextWaypoint(double x, double y, double theta)
    {
        int closestWaypoint = GetClosestWaypoint(x, y);

        double map_x =  map_waypoints_x[closestWaypoint];
        double map_y =  map_waypoints_y[closestWaypoint];

        double heading = atan2((map_y - y), (map_x - x));

        double angle = fabs(theta - heading);
        angle = std::min(2 * M_PI - angle, angle);

        if (angle > M_PI / 2)
        {
            ++closestWaypoint;
            if (closestWaypoint ==  map_waypoints_x.size())
            {
                closestWaypoint = 0;
            }
        }

        return closestWaypoint;
    }

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    std::vector<double> getFrenet(double x, double y, double theta)
    {
        int next_wp = NextWaypoint(x, y, theta);

        int prev_wp;
        prev_wp = next_wp - 1;
        if (next_wp == 0)
        {
            prev_wp =  map_waypoints_x.size() - 1;
        }

        double n_x =  map_waypoints_x[next_wp] -  map_waypoints_x[prev_wp];
        double n_y =  map_waypoints_y[next_wp] -  map_waypoints_y[prev_wp];
        double x_x = x -  map_waypoints_x[prev_wp];
        double x_y = y -  map_waypoints_y[prev_wp];

        // find the projection of x onto n
        double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
        double proj_x = proj_norm * n_x;
        double proj_y = proj_norm * n_y;

        double frenet_d =calc_distance(x_x, x_y, proj_x, proj_y);

        //see if d value is positive or negative by comparing it to a center point
        double center_x = 1000 -  map_waypoints_x[prev_wp];
        double center_y = 2000 -  map_waypoints_y[prev_wp];
        double centerToPos = calc_distance(center_x, center_y, x_x, x_y);
        double centerToRef = calc_distance(center_x, center_y, proj_x, proj_y);

        if (centerToPos <= centerToRef)
        {
            frenet_d *= -1;
        }

        // calculate s value
        double frenet_s = 0;
        for (int i = 0; i < prev_wp; ++i)
        {
            frenet_s += calc_distance( map_waypoints_x[i],  map_waypoints_y[i],  map_waypoints_x[i + 1],  map_waypoints_y[i + 1]);
        }

        frenet_s += calc_distance(0, 0, proj_x, proj_y);

        return {frenet_s, frenet_d};
    }

    // Transform from Frenet s,d coordinates to Cartesian x,y
    std::vector<double> getXY(double s, double d)
    {
        int prev_wp = -1;

        while (s >  map_waypoints_s[prev_wp + 1] && (prev_wp < (int)( map_waypoints_s.size() - 1)))
        {
            ++prev_wp;
        }

        int wp2 = (prev_wp + 1) %  map_waypoints_x.size();

        double heading = atan2(( map_waypoints_y[wp2] -  map_waypoints_y[prev_wp]),
                               ( map_waypoints_x[wp2] -  map_waypoints_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s -  map_waypoints_s[prev_wp]);

        double seg_x =  map_waypoints_x[prev_wp] + seg_s * cos(heading);
        double seg_y =  map_waypoints_y[prev_wp] + seg_s * sin(heading);

        double perp_heading = heading - M_PI / 2;

        double x = seg_x + d * cos(perp_heading);
        double y = seg_y + d * sin(perp_heading);

        return {x, y};
    }

    void SetState(double x, double y, double s, double d, double yaw, double speed);
    void Sense(std::vector<std::vector<double>> observation);
    bool Plan();
    std::pair<std::vector<double>, std::vector<double>> GetPlannedPath();
    void SetPrevPath(std::vector<double>, std::vector<double>, double, double);
    void printStatus(HighwayPlanner::State state, HighwayPlanner::BehaviorState bstate);

private:
    /* data */

    HighwayPlanner::State m_state;

    // TODO: simple state machine

    HighwayPlanner::BehaviorState m_current_planner_state = BehaviorState::KEEPLANE;
    HighwayPlanner::BehaviorState m_next_planner_state = BehaviorState::KEEPLANE;

    // paths
    std::vector<double> m_last_path_x;
    std::vector<double> m_last_path_y;
    std::vector<double> m_next_path_x;
    std::vector<double> m_next_path_y;
    double end_path_s;
    double end_path_d;
    int prev_size = 0;

    // map info
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
    double max_s = 6945.554;

    // dynamic obstacles
    std::vector<HighwayPlanner::State> car_poses; //index 0 left lane, 1 center lane, 2 right lane
    std::vector<HighwayPlanner::State> predicted_car_poses;

    // other info
    int desired_lane = 1;
    int current_lane = 1;
    double max_jerk = 0;
    double max_acc = .224/2.237;
    double max_speed_mph = 49.5;
    double max_speed_meters = max_speed_mph / 2.237;
    double target_speed = 0.01;
    double dt = 0.02;
    double time_elapsed = 0;
    double hole_s = 0;
    double num_points = 50;
    double avoid_distance = 30;

    void KeepLane();
    void ChangeLane();
    void PrepareLaneChange();
    void Predict();
    void PlanBehavior();
    std::vector<std::vector<double>> ToLocalFrame(double, double, double, std::vector<double>, std::vector<double>);
    std::vector<std::vector<double>> ToGlobalFrame(double, double, double, std::vector<double>, std::vector<double>);
};

#endif