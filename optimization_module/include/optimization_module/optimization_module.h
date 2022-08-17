//
// Created by gyl on 2022/3/27.
//

#ifndef BS_MAT_OPTIMIZATION_MODULE_H
#define BS_MAT_OPTIMIZATION_MODULE_H

#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <Eigen/Eigen>
#include <nlopt.hpp>
#include <string.h>

#include "bspline_module/conversion_BS2MV.h"
#include "swin_mapping/swin_map.h"

//优化对象为位置或偏航角
class optimization{
public:
    optimization(const SWINMap::Ptr &map){
        map_module_=map;
    }
    ~optimization(){}
    //主要接口, 进行轨迹优化
    Eigen::MatrixXd traj_optimization(const Eigen::MatrixXd &ctl_points,const Eigen::RowVectorXd &step,
                                                                                const Eigen::MatrixXd &corridors,const double &delta_t,const Eigen::MatrixXd &pos_goal);
    //获得指定一组控制点的优化函数值
    double GetTraj_objfunc(const Eigen::MatrixXd &q);

    Eigen::MatrixXd yaw_optimization(const Eigen::MatrixXd &ctl_points,const double &delta_t,const vector<double> &waypoints);

private:
    void q2x(const Eigen::MatrixXd &q,std::vector<double> &x);
    template <class T>
    void x2q(const T &x,Eigen::MatrixXd &q);
    static double traj_objfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    double computeTraj_objfunc(const Eigen::MatrixXd &q, std::vector<double> &grad);
    static void traj_multiconstraints(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);
    void computeTraj_multiconstraints(unsigned n, const Eigen::MatrixXd &q, unsigned m, double *result, double* grad);

    static double yaw_objfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    double computeYaw_objfunc(const Eigen::MatrixXd &q, std::vector<double> &grad);
    
private:
    int q_dim_; //控制点维度
    Eigen::MatrixXd q0_,q1_,q2_;//初始状态控制点
    Eigen::MatrixXd q_;//B样条曲线控制点 行q_dim_,列N_+1
    double delta_t_;//B样条曲线时间间隔
    int N_;  //B样条曲线的N N+1=控制点的数量
    int num_of_segments_; //曲线段数  是总数
    Eigen::MatrixXd traj_goal_; //目标点,用作目标函数

    int variable_num_; //优化变量的数量
    std::vector<double> x_;//NLOPT形式的优化变量,包括q3,...,qn-2
    int inequality_num_; //不等式约束的数量
    double Vmax_; //速度约束
    double Amax_; //加速度约束
    std::vector<double> step_;
    Eigen::RowVectorXd  initial_FixedStep_;
    Eigen::MatrixXd initial_Fix_q_;//B样条曲线控制点 行q_dim_,列N_+1
    Eigen::MatrixXd corridors_;

    vector<double> waypoints_;

    SWINMap::Ptr map_module_;

    Eigen::MatrixXd best_q_;
    double best_cost_;
};


#endif //BS_MAT_OPTIMIZATION_MODULE_H
