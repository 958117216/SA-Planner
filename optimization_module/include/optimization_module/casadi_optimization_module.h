#pragma once

#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <Eigen/Eigen>
#include <string.h>
#include <casadi/casadi.hpp>

#include "optimization_module/termcolor.hpp"

#include "bspline_module/conversion_BS2MV.h"
#include "swin_mapping/swin_map.h"
#include "visualization_module/planning_visualization.h"

#include "optimization_module/SaveData.hpp"

using namespace casadi;
using namespace termcolor;

//优化对象为位置或偏航角
class casadi_optimization{
public:
    casadi_optimization(const SWINMap::Ptr &map,const PlanningVisualization::Ptr& visualization);
    ~casadi_optimization(){}
    //主要接口, 进行轨迹优化
   bool traj_optimization_soft(const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
                                      const Eigen::RowVectorXd &step,const double &delta_t,const Eigen::MatrixXd &pos_goal);
   
    void UpdateStepAndCollision_Constraints(Eigen::MatrixXd ctl_points);
    bool single_traj_optimization(Eigen::MatrixXd & IterationCtrlPoints);

    double GetStep(const double dist)  //From UUCtrlAstar
    {
        double mini=0.2*delta_t_;
        double margin_=0.3;
        double max_vel_=2.0;
        if(dist<=margin_) return mini;
        else if(dist>margin_&&dist<=(margin_+max_vel_*delta_t_))
        {
            double k=(max_vel_-mini)/max_vel_*delta_t_;
            double b=mini-margin_*k;
            return (k*dist+b);
        }
        else return (max_vel_*delta_t_);
    }

private:
    int IterationNum_;
    double delta_t_;//B样条曲线时间间隔
    int N_;  //B样条曲线的N N+1=控制点的数量
    int num_of_segments_; //曲线段数  是总数
    Eigen::MatrixXd traj_goal_; //目标点,用作目标函数
    double Vmax_; //速度约束
    double Amax_; //加速度约束

    //会随着迭代改变
    Eigen::RowVectorXd  initial_FixedStep_;
    std::vector<std::vector<Eigen::Vector4d>> CollisionPlanes_; //碰撞平面 元素矩阵:4x1(碰撞平面数量，初始为1),外层vector对应控制点数，内层对应每个控制点的平面数量
    vector<Eigen::Vector3d> obsPoints_;
    vector<Eigen::Vector3d> ctrlPoints_;

    // Eigen::MatrixXd initial_Fix_q_;//B样条曲线控制点 行q_dim_,列N_+1

   conversion_BS2MV BS2MV;


    SWINMap::Ptr map_module_;
    PlanningVisualization::Ptr visualization_;
    //*casadi
    //转换矩阵
    DM p_jerk_bs_basis_matrix_;
    DM p_bs_pos2vel_vel_bs2mv_;
    DM p_bs_pos2vel_bs_vel2acc_;
    DM p_traj_goal_;
    DM p_esdf_map_;
    Function nlp_solver_;
};