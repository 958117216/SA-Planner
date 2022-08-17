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

class casadi_optimizationV2
{
public:

    casadi_optimizationV2(ros::NodeHandle& nh,const SWINMap::Ptr &map,const PlanningVisualization::Ptr& visualization);
    ~casadi_optimizationV2(){}

    void BuildNLP();

     //主要接口, 进行轨迹优化
   bool traj_optimization(const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
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

    int FixedCPNum_;//构建NLP问题固定数量
    int FixedN_;  //B样条曲线的N N+1=控制点的数量
    int Fixednum_of_segments_; //曲线段数  是总数
    double Vmax_; //速度约束
    double Amax_; //加速度约束

    //根据优化问题而定
    double delta_t_;//B样条曲线时间间隔

    int valid_CtrlP_;//实际优化问题中有效的控制点数量
    int valid_num_of_segments_;
    int valid_N_; 

    Eigen::MatrixXd traj_goal_; //目标点,用作目标函数


    //会随着迭代改变
    int IterNumNow_; //从0开始，<IterationNum_ 循环继续
    Eigen::RowVectorXd  Step_;
    std::vector<Eigen::MatrixXd> CollisionPlanes_;// vector 元素个数为: IterationNum_     每个元素为 4,(valid_N_-4)
   
   //用来rviz显示的变量
    vector<Eigen::Vector3d> obsPoints_;
    vector<Eigen::Vector3d> ctrlPoints_;



   conversion_BS2MV BS2MV;


    SWINMap::Ptr map_module_;
    PlanningVisualization::Ptr visualization_;
   
    //*casadi

    casadi::Function nlp_solver_;
    //x
    casadi::SX q;
    //g
    //速度约束和加速度约束 均以轨迹的段数划分，每一段为一行，将该段的所有约束都放入这一行
    casadi::SX vel_constraints;
    casadi::SX acc_constraints;
    //步长约束和碰撞约束 均以控制点划分，在这里直接去除永远不会考虑的点
    casadi::SX step_constraints;//将end约束加入其中 step=0
    casadi::SX collision_constraints;

    //p
    casadi::SX step; //每个节点的step 目的是对应搜索的结果
    casadi::SX traj_goal;
    casadi::SX collision_Planes;

    casadi::SX coefficient_jerk;//将有效段设为1 无效段为0
    casadi::SX coefficient_goal;//将qn-2设为1 其余为0

    casadi::SX jerk_bs_basis_matrix;//纯系数
    casadi::SX bs_pos2vel_vel_bs2mv;
    casadi::SX bs_pos2vel_bs_vel2acc;


    //转换矩阵
    DM p_jerk_bs_basis_matrix_;

    DM p_bs_pos2vel_vel_bs2mv_;
    DM p_bs_pos2vel_bs_vel2acc_;
    DM p_traj_goal_;


};




