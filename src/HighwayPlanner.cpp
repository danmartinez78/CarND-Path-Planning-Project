#include "HighwayPlanner.h"

void HighwayPlanner::SetState(double x, double y, double s, double d, double yaw, double speed){
    this->m_state.x = x;
    this->m_state.y = y;
    this->m_state.s = s;
    this->m_state.d = d;
    this->m_state.yaw = yaw;
    this->m_state.accel = (this->m_state.speed - speed)/dt;
    this->m_state.speed = speed;
}

void HighwayPlanner::SetPrevPath(std::vector<double> last_path_x, std::vector<double> last_path_y){
    this->m_last_path_x = last_path_x;
    this->m_last_path_y = last_path_y;
}

std::pair<std::vector<double>, std::vector<double>> HighwayPlanner::GetPlannedPath(){
    return std::make_pair(m_next_path_x, m_next_path_y);
}

double HighwayPlanner::distance(double x1, double y1, double x2, double y2){
    return 0.0;
}

void HighwayPlanner::Predict(){
    double time_to_predict = m_last_path_x.size()*0.02; // TODO check this
    predicted_car_poses.clear();
    for(auto car:this->car_poses){
        double x = car[1];
        double y = car[2];
        double vx = car[3];
        double vy = car[4];
        double speed = sqrt(vx*vx + vy*vy);
        double s = car[5];
        s += time_to_predict * speed;
        double d = car[6];
        std::vector<double> pred_car{s, d, speed};
    }
}

void HighwayPlanner::PlanBehavior(){
    // if keep lane, compare average speeds in lane, if fastest lane is not mine, if hole exists, transition to planning a lane change, else keep lane
    // if planning lane change, if next to hole and speed is matched, transition to lane change, else, keep preparing
    // if lane change, check to see if in desired lane, transition to keep lane, else keep changing lane
    switch(m_current_planner_state){
        case HighwayPlanner::BehaviorState::KEEPLANE:
            // TODO: if keep lane, compare average speeds in lane, if fastest lane is not mine, if hole exists, transition to planning a lane change, else keep lane
            
            break;
        case HighwayPlanner::BehaviorState::PLCL:
            // TODO: if planning lane change, if next to hole and speed is matched, transition to lane change, else, keep preparing

            break;
        case HighwayPlanner::BehaviorState::PLCR:
            // TODO: if planning lane change, if next to hole and speed is matched, transition to lane change, else, keep preparing

            break;
        case HighwayPlanner::BehaviorState::LCL:
            // if lane change, check to see if in desired lane, transition to keep lane, else keep changing lane
            if (current_lane = desired_lane){
                m_next_planner_state = HighwayPlanner::BehaviorState::KEEPLANE;
            }
            break;
        case HighwayPlanner::BehaviorState::LCR:
            // if lane change, check to see if in desired lane, transition to keep lane, else keep changing lane
            if (current_lane = desired_lane){
                m_next_planner_state = HighwayPlanner::BehaviorState::KEEPLANE;
            }            
            break;
        default:
            KeepLane();
    }
}

void HighwayPlanner::KeepLane(){
    // look forward and plan station keeping traj
    // smooth by using part of last traj
    // compute remaining traj with desired speed

}

void HighwayPlanner::PrepareLaneChange(){
    // hole in desired lane -> get the dist
    // plan speed to match
    
}

void HighwayPlanner::ChangeLane(){
    // maintain speed 
    // switch lane
    
}

bool HighwayPlanner::Plan(){
    Predict();
    PlanBehavior();
    switch(m_next_planner_state){
        case HighwayPlanner::BehaviorState::KEEPLANE:
            desired_lane = current_lane;
            // plan smooth station keeping
            KeepLane();
            break;
        case HighwayPlanner::BehaviorState::PLCL:
            desired_lane = current_lane - 1;
            // plan speed to align with hole in left lane
            PrepareLaneChange();
            break;
        case HighwayPlanner::BehaviorState::PLCR:
            desired_lane = current_lane + 1;
            // plan speed to align with hole in right lane
            PrepareLaneChange();
            break;
        case HighwayPlanner::BehaviorState::LCL:
            // plan smooth change to left lane
            ChangeLane();
            break;
        case HighwayPlanner::BehaviorState::LCR:
            // plan smooth change to right lane
            ChangeLane();
            break;
        default:
            KeepLane();
    }
}
