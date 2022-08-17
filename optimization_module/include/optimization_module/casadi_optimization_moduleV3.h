/**
 * @file casadi_optimization_moduleV3.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief  与V2不同，此版本会提前构建多个求解问题，根据需求选取指定数量的求解问题即可
 * @version 0.1
 * @date 2022-06-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
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



class casadi_optimizationV3
{

public:
    casadi_optimizationV3(ros::NodeHandle& nh,const SWINMap::Ptr &map,const PlanningVisualization::Ptr& visualization);
    ~casadi_optimizationV3(){}

    void BuildNLP();

     //主要接口, 进行轨迹优化
   bool traj_optimization(const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
                                      const Eigen::RowVectorXd &step,const double &delta_t,const Eigen::MatrixXd &pos_goal);
   
    void UpdateStepAndCollision_Constraints(Eigen::MatrixXd ctl_points);
    bool single_traj_optimization(Eigen::MatrixXd & IterationCtrlPoints);

    double GetStep(const double dist)  //From UUCtrlAstar
    {
        double mini=reduced_vel_*delta_t_;
        double max_vel_=Vmax_;
        if(dist<=margin_) return mini;
        else if(dist>margin_&&dist<=(margin_+max_vel_*delta_t_))
        {
            double k=(max_vel_-mini)/max_vel_*delta_t_;
            double b=mini-margin_*k;
            return (k*dist+b);
        }
        else return (max_vel_*delta_t_);
    }
    double GetStepEASA(const vector<Eigen::Vector3d>& LocalCPs)
    {
        Eigen::Vector3d pos;
        // pos=0.1667*LocalCPs[0]+0.6667*LocalCPs[1]+0.1667*LocalCPs[2];
        pos=LocalCPs[2];
        double dist=map_module_->getDistance(pos);
        if(dist<reduced_threshold_)
        {
            Eigen::Vector3d qv0,qv1,vel;
            qv0=(LocalCPs[1]-LocalCPs[0])/delta_t_;
            qv1=(LocalCPs[2]-LocalCPs[1])/delta_t_;
            vel=0.5*qv0+0.5*qv1;
            // vel=qv1;
            Eigen::Vector3d dist_grad;
            dist_grad=-1*map_module_->getGradient(pos);

            double vmin=reduced_vel_;
            double vmax=Vmax_;
            double k=(vmin-vmax)/2,b=(vmax+vmin)/2;

            if(dist_grad.norm()<1e-4||vel.norm()<1e-4)
            {
                return b*delta_t_;
            }
            else
            {
                return (k*(vel.dot(dist_grad) / vel.norm() / dist_grad.norm())+b)*delta_t_;
            }

        }
        else    return (Vmax_*delta_t_);
    }

private:

    int IterationNum_;
    int MaxCPNum_; 

    int CPNum_;//构建NLP问题固定数量
    int N_;  //B样条曲线的N N+1=控制点的数量
    int num_of_segments_; //曲线段数  是总数
    double Vmax_; //速度约束
    double Amax_; //加速度约束
    double margin_;
    double reduced_threshold_,reduced_vel_;
    bool show_rviz_;
    //根据优化问题而定
    double delta_t_;//B样条曲线时间间隔

    Eigen::MatrixXd traj_goal_; //目标点,用作目标函数
    Eigen::Vector3d map_min_pos_, map_max_pos_;

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
    std::vector<casadi::Function>  nlp_solvers_;
    //转换矩阵  可提前定义的，设为类成员变量
    DM p_jerk_bs_basis_matrix_;
    DM p_bs_pos2vel_vel_bs2mv_;
    DM p_bs_pos2vel_bs_vel2acc_;
};


