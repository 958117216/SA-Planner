/**
 * @file conversion_BS2MV.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief   Conversion  from B-Spline between MINVO
 * @version 0.1
 * @date 2022-03-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <vector>
#include <ros/ros.h>

//General matrix representations for B-splines
//Refer https://github.com/mit-acl/minvo/blob/master/src/solutions/getA_MV.m

class conversion_BS2MV
{
public:
       Eigen::Matrix4d  pos_bs_basis_matrix_;
       Eigen::Matrix4d  pos_mv_basis_matrix_;
       Eigen::Matrix4d  pos_bs2mv_;//*

       Eigen::Matrix3d  vel_bs_basis_matrix_;
       Eigen::Matrix3d  vel_mv_basis_matrix_;
       Eigen::Matrix3d vel_bs2mv_;//*

        //均匀B样条曲线控制点，求导
       double delta_t_=1.0;
       Eigen::Matrix<double,4,3> bs_pos2vel_;
       Eigen::Matrix<double,3,2> bs_vel2acc_;
public:
    //https://stackoverflow.com/questions/308276/can-i-call-a-constructor-from-another-constructor-do-constructor-chaining-in-c
    conversion_BS2MV():conversion_BS2MV(1.0){}
    
    conversion_BS2MV(double delta_t):delta_t_(delta_t)
    {
        // std::cout<<"delta_t_:"<<delta_t_<<std::endl;
        pos_bs_basis_matrix_<<
            -0.166667, 0.500000, -0.500000, 0.166667,
            0.500000, -1.000000, 0.000000, 0.666667,
            -0.500000, 0.500000, 0.500000, 0.166667,
            0.166667, 0.000000, 0.000000, 0.000000;
        //时间间隔为[0,1]的3次MINVO的系数矩阵
        pos_mv_basis_matrix_<<
            -3.4416309793565660335445954842726,  6.9895482693324069156659561485867, -4.4622887879670974919932291413716,                  0.91437149799125659734933338484986,
            6.6792587678886103930153694818728, -11.845989952130473454872117144987,  5.2523596862506065630071816485724, -0.000000000000000055511151231257827021181583404541,
            -6.6792587678886103930153694818728,  8.1917863515353577241739913006313, -1.5981560856554908323090558042168,                 0.085628502008743445639282754200394,
            3.4416309793565660335445954842726,  -3.335344668737291184967830304231, 0.80808518737198176129510329701588, -0.000000000000000012522535092207576212786079850048;

        Eigen::Vector4d pos_transform;
        pos_transform<<pow(1/delta_t_,3),pow(1/delta_t_,2),pow(1/delta_t_,1),1;
        pos_mv_basis_matrix_=pos_mv_basis_matrix_*pos_transform.asDiagonal();

        pos_bs2mv_=pos_bs_basis_matrix_*pos_mv_basis_matrix_.inverse();

        vel_bs_basis_matrix_<<
            0.5,-1.0,0.5,
            -1.0,1.0,0.5,
            0.5,0.0,0.0;
        //时间间隔为[0,1]的2次MINVO的系数矩阵
        vel_mv_basis_matrix_<<
            1.4999999992328318931811281800037, -2.3660254034601951866889635311964,   0.9330127021136816189983420599674,
            -2.9999999984656637863622563600074,  2.9999999984656637863622563600074,                                   0,
            1.4999999992328318931811281800037, -0.6339745950054685996732928288111, 0.066987297886318325490506708774774;

        Eigen::Vector3d vel_transform;
        vel_transform<<pow(1/delta_t_,2),pow(1/delta_t_,1),1;
        vel_mv_basis_matrix_=vel_mv_basis_matrix_*vel_transform.asDiagonal();

        vel_bs2mv_=vel_bs_basis_matrix_*vel_mv_basis_matrix_.inverse();

        bs_pos2vel_<<
        -1,   0,  0, 
          1, -1,  0,
          0,  1, -1, 
          0,  0,   1;
       bs_pos2vel_/=delta_t_;
        
        bs_vel2acc_<<
        -1,   0, 
          1, -1, 
          0,  1;
        bs_vel2acc_/=delta_t_;
    }


    ~conversion_BS2MV(){}
};