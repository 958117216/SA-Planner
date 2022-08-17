#pragma once

#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <Eigen/Eigen>
#include <string.h>
#include <gurobi_c++.h>


#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>

#include "bspline_module/conversion_BS2MV.h"
#include "swin_mapping/swin_map.h"
#include "path_search/uu_dirc_ctrl_astar.h"
#include "visualization_module/planning_visualization.h"
#include "optimization_module/termcolor.hpp"
#include "optimization_module/SaveData.hpp"
#include "optimization_module/solver_gurobi_utils.hpp"


using namespace termcolor;

class gurobi_optimization{
public:
    gurobi_optimization(ros::NodeHandle& nh,const SWINMap::Ptr &map,const UUDircCtrlAstar::Ptr &search,const PlanningVisualization::Ptr& visualization);
    ~gurobi_optimization(){}


    //主要接口, 进行轨迹优化
    bool traj_optimization(const Eigen::VectorXd&pos_start,const Eigen::VectorXd &vel_start,const Eigen::VectorXd &acc_start,
                                       const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
                                       const Eigen::RowVectorXd &step,const double &delta_t,const Eigen::VectorXd &pos_goal);


private:

    void addObjective();
    void addConstraints();
    void addVelConstraints();
    void addAccConstraints();
    void addStepConstraints();
    void addMICollisionConstraints();
    void addCollisionConstraints();
    void addSimpleMICollisionConstraints();
    void addEndConstraints();
    void addStartConstraints();
    bool single_traj_optimization(Eigen::MatrixXd & IterationCtrlPoints);
    void UpdateStepConstraints(Eigen::MatrixXd ctl_points);

    //Using ellipsoid decomposition
    EllipsoidDecomp3D decomp_util;
    conversion_BS2MV BS2MV;
    SWINMap::Ptr map_module_;
    UUDircCtrlAstar::Ptr search_module_;
    PlanningVisualization::Ptr visualization_;

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

    Eigen::VectorXd pos_start_;
    Eigen::VectorXd vel_start_;
    Eigen::VectorXd acc_start_;
    Eigen::VectorXd traj_goal_; //目标点,用作目标函数

    Eigen::Vector3d map_min_pos_, map_max_pos_;

    //会随着迭代改变
    int IterNumNow_; //从0开始，<IterationNum_ 循环继续
    Eigen::RowVectorXd  Step_;

    //*Ros
    ros::Publisher es_pub,poly_pub;

    //*Gurobi
    GRBEnv *env_ = new GRBEnv();
    GRBModel m_ = GRBModel(*env_);
    
    // Each q_[i] has 3 elements (x,y,z) 外层是点数，内层是维度
    std::vector<std::vector<GRBLinExpr>> q_;
    //二进制变量 b n_seg,p n_seg为轨迹段数 p为凸多面体数
    std::vector<std::vector<GRBVar>> b_;

    GRBQuadExpr control_cost_ = 0.0;
    GRBQuadExpr terminal_cost_ = 0.0;
    GRBQuadExpr v0_cost_ = 0.0;
    GRBQuadExpr a0_cost_ = 0.0;
    GRBQuadExpr step_cost_ = 0.0;
    GRBQuadExpr cost_ = 0.0;
};
