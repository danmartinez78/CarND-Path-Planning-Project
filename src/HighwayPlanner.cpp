#include "HighwayPlanner.h"

void HighwayPlanner::SetState(double x, double y, double s, double d, double yaw, double speed){
    this->m_state.x = x;
    this->m_state.y = y;
    this->m_state.s = s;
    this->m_state.d = d;
    this->m_state.yaw = getdeg2rad(yaw);
    this->m_state.accel = (this->m_state.speed - speed)/dt;
    this->m_state.speed = speed;
}

void HighwayPlanner::SetPrevPath(std::vector<double> last_path_x, std::vector<double> last_path_y){
    m_last_path_x.clear();
    m_last_path_y.clear();
    this->m_last_path_x = last_path_x;
    this->m_last_path_y = last_path_y;
    this->prev_size = last_path_x.size();
}

std::pair<std::vector<double>, std::vector<double>> HighwayPlanner::GetPlannedPath(){
    return std::make_pair(m_next_path_x, m_next_path_y);
}

void HighwayPlanner::Sense(std::vector<std::vector<double>> observation) { 
    this->car_poses = observation; }

void HighwayPlanner::Predict(){
    double time_to_predict = prev_size*0.02; // TODO: check this
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
    double ref_x = m_state.x;
    double ref_y = m_state.y;
    double ref_yaw = m_state.yaw;
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    if(prev_size < 2){
        // get a tangent segment
        double prev_x = ref_x - std::cos(ref_yaw);
        double prev_y = ref_y - std::sin(ref_yaw);
        ptsx.push_back(prev_x);
        ptsx.push_back(ref_x);
        ptsy.push_back(prev_y);
        ptsy.push_back(ref_y);
    }
    else{
        ref_x = m_last_path_x[prev_size-1];
        ref_y = m_last_path_y[prev_size-1];
        double ref_x_prev = m_last_path_x[prev_size-2];
        double ref_y_prev = m_last_path_y[prev_size-2];
        ref_yaw = std::atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    std::vector<double> next_wp0 = getXY(m_state.s + 30, (2+4*current_lane));
    std::vector<double> next_wp1 = getXY(m_state.s + 60, (2+4*current_lane));
    std::vector<double> next_wp2 = getXY(m_state.s + 90, (2+4*current_lane));

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0;i<ptsx.size();i++){
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * std::cos(0-ref_yaw)-shift_y*std::sin(0-ref_yaw));
        ptsy[i] = (shift_x * std::sin(0-ref_yaw)+shift_y*std::cos(0-ref_yaw));
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);
    m_next_path_x.clear();
    m_next_path_y.clear();
    
    for(int i = 0; i<prev_size;i++){
        m_next_path_x.push_back(m_last_path_x[i]);
        m_next_path_y.push_back(m_last_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_d = std::sqrt((target_x*target_x)+(target_y*target_y));

    double x_add_on = 0;
    for (int i = 0; i <=num_points-prev_size;i++){
        double N = (target_d/(dt*target_speed_meters));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref*std::cos(ref_yaw)-y_ref*std::sin(ref_yaw));
        y_point = (x_ref*std::sin(ref_yaw)+y_ref*std::cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        m_next_path_x.push_back(x_point);
        m_next_path_y.push_back(y_point);
    }
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
    printStatus();
    //Predict();
    //PlanBehavior();
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
            return false;
    }
    return true;
}

std::vector<std::vector<double>> HighwayPlanner::ToLocalFrame(double ref_x, double ref_y, double ref_yaw, std::vector<double> ptsx, std::vector<double> ptsy){
    for (int i = 0;i<ptsx.size();i++){
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * std::cos(0-ref_yaw)-shift_y*std::sin(0-ref_yaw));
        ptsy[i] = (shift_x * std::sin(0-ref_yaw)+shift_y*std::cos(0-ref_yaw));
    }
    std::vector<std::vector<double>> xformed_pts{ptsx, ptsy};
    return xformed_pts;
}

void HighwayPlanner::printStatus(){
    std::cout << "\nSTATUS------------------\n";
    std::cout << "x,y,yaw: (" << m_state.x << "," << m_state.y << "," << HighwayPlanner::getrad2deg(m_state.yaw) << ")\n";
    std::cout << "s,d: (" << m_state.d << "," << m_state.d << ")\n";
    std::cout << "speed: " << m_state.speed << "\n";
    std::cout << "Behavior: " ;
    switch (m_current_planner_state)
    {
    case HighwayPlanner::BehaviorState::KEEPLANE:
        std::cout << "KEEPLANE\n";
        break;
    case HighwayPlanner::BehaviorState::PLCL:
        std::cout << "PLCL\n";
        break;
    case HighwayPlanner::BehaviorState::PLCR:
        std::cout << "PLCR\n";
        break;
    case HighwayPlanner::BehaviorState::LCL:
        std::cout << "LCL\n";
        break;
    case HighwayPlanner::BehaviorState::LCR:
        std::cout << "LCR\n";
        break;
    default:
        break;
    }
}

