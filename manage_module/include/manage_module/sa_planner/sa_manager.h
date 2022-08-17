/**
 * @file sa_manager.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief   管理 地图模块 路径搜索模块 轨迹优化模块
 * @version 0.1
 * @date 2022-03-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#pragma once

#include <ros/ros.h>

#include "bspline_module/unclamped_uniform_bspline.h"

#include "swin_mapping/swin_map.h"
// #include "path_search/uu_ctrl_astar.h"
// #include "path_search/uu_ctrl_astar_four.h"
#include "path_search/uu_dirc_ctrl_astar.h"
#include "optimization_module/optimization_module.h"
// #include "optimization_module/casadi_optimization_module.h"
// #include "optimization_module/casadi_optimization_moduleV2.h"
// #include "optimization_module/casadi_optimization_moduleV3.h"
// #include "optimization_module/casadi_optimization_moduleV4.h"
#include "optimization_module/gurobi_optimization_module.h"

#include <decomp_ros_utils/data_ros_utils.h>
#include <decomp_util/ellipsoid_decomp.h>

#include "visualization_module/planning_visualization.h"
#include "manage_module/sa_planner/sa_traj_info.hpp"
#include "manage_module/termcolor.hpp"



class SA_Manager
{
public:
    //初始化
    SA_Manager(){ }
    ~SA_Manager(){ }

    void  initManager(ros::NodeHandle& nh,const PlanningVisualization::Ptr& visualization);
    bool Replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,Eigen::Vector3d end_pt);

    void planYaw(const Eigen::Vector3d& start_yaw);
    Eigen::MatrixXd GetCorridors(Eigen::MatrixXd ctrl_pts);

public:
    PlanParameters pp_;  //struct
    LocalTrajData local_data_; 
    
    SWINMap::Ptr map_module_;

    // std::unique_ptr<UUCtrlAstar_Four>    search_module_;
    // std::unique_ptr<UUCtrlAstar>    search_module_;
    UUDircCtrlAstar::Ptr    search_module_;
    std::unique_ptr<optimization> optimization_module_; //偏航角优化
    
    // std::unique_ptr<casadi_optimization> casadi_optimization_module_;
    // std::unique_ptr<casadi_optimizationV2> casadi_optimization_moduleV2_;
    // std::unique_ptr<casadi_optimizationV3> casadi_optimization_moduleV3_;
    // std::unique_ptr<casadi_optimizationV4> casadi_optimization_moduleV4_;
    std::unique_ptr<gurobi_optimization> gurobi_optimization_module_;
    
      typedef std::unique_ptr<SA_Manager> Ptr;
private:
    void calcNextYaw(const double& last_yaw, double& yaw);
    void updateTrajInfo();  

};

