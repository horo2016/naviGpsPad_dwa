//
// Created by pang on 2020/6/13.
//

#include "dwa.h"
#include<cmath>
#include<iostream>
#include<vector>
#include<array>
#include<cmath>
#include <limits>

Dwa::Dwa(const State& start, const Point& goal,  const Obstacle& obs, const Config& config):
cur_x_(start), goal_(goal), obs_(obs), config_(config)
{

}

bool Dwa::stepOnceToGoal(std::vector<State>* best_trajectry, State* cur_state,Obstacle *cur_obs) 
{
     
	
    Window dw = calc_dynamic_window(cur_x_, config_);
	 

    Traj ltraj = calc_final_input(cur_x_, calculated_u, dw, config_, goal_, obs_);
//    printf("control (v,w) (%.1f,%.1f)\n ",calculated_u.v_,calculated_u.w_);

    //
    *best_trajectry = ltraj;
    *cur_state = cur_x_;
	*cur_obs = obs_;
    if (std::sqrt(std::pow((cur_x_.x_ - goal_.x_), 2) + std::pow((cur_x_.y_ - goal_.y_), 2)) <= config_.goal_min_dis){
       return true;
    }
    return false;
}


State Dwa::motion(State x, Control u, float dt){
    x.theta_ += u.w_ * dt;
    x.x_ += u.v_ * std::cos(x.theta_) * dt;
    x.y_ += u.v_ * std::sin(x.theta_) * dt;
    x.v_ = u.v_;
    x.w_ = u.w_;
    return x;
};

Window Dwa::calc_dynamic_window(State x, Config config){

    return {
            std::max((x.v_ - config.max_accel * config.dt), config.min_speed),
            std::min((x.v_ + config.max_accel * config.dt), config.max_speed),
            std::max((x.w_ - config.max_dyawrate * config.dt), -config.max_yawrate),
            std::min((x.w_ + config.max_dyawrate * config.dt), config.max_yawrate)
    };

//    return {
//            (x.v_ - config.max_accel * config.dt),
//            (x.v_ + config.max_accel * config.dt),
//            (x.w_ - config.max_dyawrate * config.dt),
//            (x.w_ + config.max_dyawrate * config.dt)
//    };
};


Traj Dwa::calc_trajectory(State x, float v, float w, Config config){

    Traj traj;
    traj.push_back(x);
    float time = 0.0;
   //这里实现预测：预测时间为predicattime 3，每一次执行为config.dt 时间 ，保留的轨迹点waypoint_cnts = predicat /config.dt
  //实际中 config.dt 应该与地盘执行时间返回时间保持一致
    while (time <= config.predict_time){
        x = motion(x, Control{v, w}, config.dt);
        traj.push_back(x);
        time += config.dt;
    }
    return traj;
};


float Dwa::calc_obstacle_cost(Traj traj, Obstacle ob, Config config){
    // calc obstacle cost inf: collistion, 0:free
    int skip_n = 2;
    float minr = std::numeric_limits<float>::max();

    for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
        for (unsigned int i=0; i< ob.size(); i++){
            float ox = ob[i].x_;
            float oy = ob[i].y_;
            float dx = traj[ii].x_ - ox;
            float dy = traj[ii].y_ - oy;

            float r = std::sqrt(dx*dx + dy*dy);
            if (r <= config.obstacle_min_dis){//小于障碍物距离
                return std::numeric_limits<float>::max();//有障碍物时返回最大损耗 
            }

            if (minr >= r){
                minr = r;
            }
        }
    }

    return 1.0 / minr;// 里障碍物越近 返回值越大
};

float Dwa::calc_to_goal_cost(Traj traj, Point goal, Config config){

    float goal_magnitude = std::sqrt(goal.x_*goal.x_ + goal.y_*goal.y_);
    float traj_magnitude = std::sqrt(std::pow(traj.back().x_, 2) + std::pow(traj.back().y_, 2));
    float dot_product = (goal.x_ * traj.back().x_) + (goal.y_ * traj.back().y_);
    float error = dot_product / (goal_magnitude * traj_magnitude);
    float error_angle = std::acos(error);
    float cost = config.to_goal_cost_gain * error_angle;

    return cost;
};

Traj Dwa::calc_final_input(
        State x, Control& u,
        Window dw, Config config, Point goal,
        std::vector<Point>ob){

    float min_cost = 10000.0;
    Control min_u = u;
    min_u.v_ = 0.0;
    Traj best_traj;

    // capture the start time
    clock_t         start, stop;
    start = clock();

    // evalucate all trajectory with sampled input in dynamic window
    int traj_cnt = 0;
    for (float v=dw.min_v_; v<=dw.max_v_; v+=config.v_reso){
        for (float w=dw.min_w_; w<=dw.max_w_; w+=config.yawrate_reso){

            Traj traj = calc_trajectory(x, v, w, config);//模拟计算出 轨迹来

            float to_goal_cost = calc_to_goal_cost(traj, goal, config);
            float speed_cost = config.speed_cost_gain * (config.max_speed - traj.back().v_);
            float ob_cost = calc_obstacle_cost(traj, ob, config);
			// by lide 增加比重
		    // 航向得分的比重、速度得分的比重、障碍物距离得分的比重 
            //evalParam = [];
            float final_cost = 1.5*to_goal_cost + 2.5*speed_cost + ob_cost;
            //计算总的损耗  损耗越小path better
            if (final_cost  <= min_cost){
                min_cost = final_cost;
                min_u = Control{v, w};
                best_traj = traj;
            }
            traj_cnt ++;
        }
    }

    stop = clock();
    float   elapsedTime = (float)(stop - start) /
                          (float)CLOCKS_PER_SEC * 1000.0f;
    printf( "Time to generate:  %3.1f ms\n", elapsedTime );

    u = min_u;
    return best_traj;
};



