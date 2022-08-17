#include "optimization_module/casadi_optimization_moduleV2.h"



casadi_optimizationV2::casadi_optimizationV2(ros::NodeHandle& nh,const SWINMap::Ptr &map,const PlanningVisualization::Ptr& visualization)
{
    map_module_=map;
    visualization_=visualization;

    nh.param("casadi_optV2/max_vel",Vmax_,2.0);
    nh.param("casadi_optV2/max_acc",Amax_,2.0);
    nh.param("casadi_optV2/IterationNum",IterationNum_,1);
    nh.param("casadi_optV2/FixedCPNum",FixedCPNum_,20);


    FixedN_=FixedCPNum_-1;
    Fixednum_of_segments_=FixedCPNum_-3;

    p_jerk_bs_basis_matrix_=casadi::DM::zeros(4,1);
    Eigen::Matrix4d  pos_bs_basis_matrix;
    pos_bs_basis_matrix<<
        -0.166667, 0.500000, -0.500000, 0.166667,
        0.500000, -1.000000, 0.000000, 0.666667,
        -0.500000, 0.500000, 0.500000, 0.166667,
        0.166667, 0.000000, 0.000000, 0.000000;
    Eigen::Matrix<double, 4, 1> tmp;
    tmp << 6.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, 4, 1> jerk_bs_basis_matrix_temp=pos_bs_basis_matrix*tmp;
    std::copy(jerk_bs_basis_matrix_temp.data(), jerk_bs_basis_matrix_temp.data() + jerk_bs_basis_matrix_temp.size(),
            p_jerk_bs_basis_matrix_.ptr());

    BuildNLP();
}


void casadi_optimizationV2::BuildNLP()
{

    //构建NLP
    //x
    q = SX::sym("q",3,FixedCPNum_);
    //g
    //速度约束和加速度约束 均以轨迹的段数划分，每一段为一行，将该段的所有约束都放入这一行
    vel_constraints=SX::sym("vel_constraints",Fixednum_of_segments_-2,9);
    acc_constraints=SX::sym("acc_constraints",(Fixednum_of_segments_-2),3);
    //步长约束和碰撞约束 均以控制点划分，在这里直接去除永远不会考虑的点
    step_constraints=SX::sym("step_constraints",FixedN_-2,1);//将end约束加入其中 step=0
    collision_constraints=SX::sym("collision_constraints",FixedN_-4,IterationNum_);

    //p
    step=SX::sym("step",FixedN_+1,1); //每个节点的step 目的是对应搜索的结果
    traj_goal=SX::sym("traj_goal",3,1);
    collision_Planes=SX::sym("collision_Planes",4,(FixedN_-4)*IterationNum_);

    coefficient_jerk=SX::sym("coefficient_jerk",Fixednum_of_segments_,1);//将有效段设为1 无效段为0
    coefficient_goal=SX::sym("coefficient_goal",FixedN_-4,1);//将qn-2设为1 其余为0

    jerk_bs_basis_matrix= SX::sym("jerk_bs_basis_matrix",4,1);//纯系数
    bs_pos2vel_vel_bs2mv=SX::sym("bs_pos2vel_vel_bs2mv",4,3);
    bs_pos2vel_bs_vel2acc=SX::sym("bs_pos2vel_bs_vel2acc",4,2);

    //构建vel约束
    for (int i = 0; i <Fixednum_of_segments_-2; i++)
    {
        vel_constraints(Slice(i,i+1),Slice(0,3))=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(0,1)).T();
        vel_constraints(Slice(i,i+1),Slice(3,6))=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(1,2)).T();
        vel_constraints(Slice(i,i+1),Slice(6,9))=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(2,3)).T();
    }
    //构建acc约束
    for (int i = 0; i < Fixednum_of_segments_-2; ++i)
    {
        acc_constraints(Slice(i,i+1), Slice()) =mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_bs_vel2acc)(Slice(),Slice(1,2)).T();
    }
    //构建step约束
    for (int k = 3; k <= FixedN_; ++k)
    {
        step_constraints(k-3)=sumsqr(q(Slice(),Slice(k,k+1))-q(Slice(),Slice(k-1,k)))-step(k)*step(k);
    }
    //构建collision约束
    for (int i = 0; i < IterationNum_; ++i) //第i次迭代
    {
        for (int k = 3; k <= FixedN_-2; ++k)
        {
            collision_constraints(k-3,i)=mtimes(q(Slice(),Slice(k,k+1)).T(),collision_Planes(Slice(0,3),Slice((k-3)+(FixedN_-4)*i,(k-3)+(FixedN_-4)*i+1)))+collision_Planes(3,(k-3)+(FixedN_-4)*i);
        }
    }

    casadi::SX f=0;
    //构建目标函数
    // qn-2与终点 的代价

    for (int k = 3; k <= FixedN_-2; ++k)
    {
        f+=10*coefficient_goal(k-3)*sumsqr(q(Slice(),Slice(k,k+1))-traj_goal);
    }
    //Jerk^2
    for (int i = 0; i <Fixednum_of_segments_; i++)
    {
        f+=coefficient_jerk(i)*sumsqr(mtimes(q(Slice(),Slice(i,i+4)),jerk_bs_basis_matrix));
    }
    //    std::cout <<"f is "<< f<<std::endl;


    casadi::SX x = reshape(q,q.numel(),1);
    casadi::SX p = SX::vertcat({step, traj_goal,
                                reshape(collision_Planes,collision_Planes.numel(),1),
                                coefficient_jerk,coefficient_goal,jerk_bs_basis_matrix,
                                reshape(bs_pos2vel_vel_bs2mv,bs_pos2vel_vel_bs2mv.numel(),1),
                                reshape(bs_pos2vel_bs_vel2acc,bs_pos2vel_bs_vel2acc.numel(),1)});

    casadi::SX g = SX::vertcat({reshape(vel_constraints,vel_constraints.numel(),1),
                                reshape(acc_constraints,acc_constraints.numel(),1),
                                step_constraints,
                                reshape(collision_constraints,collision_constraints.numel(),1)});

    SXDict nlp = {{"x", x},{"p", p},{"f", f},{"g",g}};
    Dict nlp_config_={{"ipopt", Dict({  {"linear_solver", "ma27"},{"max_iter", 50},{"print_level", 5}  })}}; //
    nlp_solver_ = nlpsol("nlp_solver_", "ipopt", nlp,nlp_config_);

    std::cout << repr(nlp_solver_)<<std::endl;
}


bool casadi_optimizationV2::traj_optimization(const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
                                      const Eigen::RowVectorXd &step,const double &delta_t,const Eigen::MatrixXd &pos_goal)
{
    IterNumNow_=0;
    //获得问题信息
    delta_t_=delta_t;
    BS2MV=conversion_BS2MV(delta_t);

    p_bs_pos2vel_vel_bs2mv_=casadi::DM::zeros(4,3);
    Eigen::Matrix<double, 4, 3> bs_pos2vel_vel_bs2mv_tmp=BS2MV.bs_pos2vel_*BS2MV.vel_bs2mv_;
    std::copy(bs_pos2vel_vel_bs2mv_tmp.data(),bs_pos2vel_vel_bs2mv_tmp.data()+bs_pos2vel_vel_bs2mv_tmp.size(),
            p_bs_pos2vel_vel_bs2mv_.ptr());

    p_bs_pos2vel_bs_vel2acc_=casadi::DM::zeros(4,2);
    Eigen::Matrix<double, 4, 2> bs_pos2vel_bs_vel2acc_tmp=BS2MV.bs_pos2vel_*BS2MV.bs_vel2acc_;
    std::copy(bs_pos2vel_bs_vel2acc_tmp.data(),bs_pos2vel_bs_vel2acc_tmp.data()+bs_pos2vel_bs_vel2acc_tmp.size(),
            p_bs_pos2vel_bs_vel2acc_.ptr());

    //获得问题信息
    valid_CtrlP_=ctl_points.cols();
    valid_N_=valid_CtrlP_-1;
    valid_num_of_segments_=valid_CtrlP_-3;

    traj_goal_=pos_goal;

    //会随着迭代而改变的问题信息
    CollisionPlanes_.clear();
    Eigen::MatrixXd EmptyPlane; 
    EmptyPlane.resize(4,valid_N_-4);
    CollisionPlanes_.assign(IterationNum_,EmptyPlane);


    Step_=step;

    //优化的控制点    
    opt_ctl_points=ctl_points;

    //清空rviz变量
    obsPoints_.clear();
    ctrlPoints_.clear();

    bool result;
    while(IterNumNow_<IterationNum_)
    {
        std::cout << "第"<<IterNumNow_<<"次迭代************************************************************"<<std::endl;
        UpdateStepAndCollision_Constraints(opt_ctl_points);
        result=single_traj_optimization(opt_ctl_points);
        IterNumNow_++;
    }

    if(result==true) return true;
    else return false;
}


void casadi_optimizationV2::UpdateStepAndCollision_Constraints(Eigen::MatrixXd ctl_points)
{
    for (int k = 3; k <=valid_N_-2; ++k)// valid_N_-4
    {
        Eigen::Vector3d q=ctl_points.col(k);
        ctrlPoints_.push_back(q);

        //此处是否要加判断是否在地图内
        //步长由父节点位置决定
        Eigen::Vector3d q_father=ctl_points.col(k-1);
        double dist=map_module_->getDistance(q_father); 
        Step_[k]=std::max(Step_[k],GetStep(dist));
        
        Eigen::Vector3d gradient=map_module_->getGradient(q);
        // std::cout <<"ctlPoint:"<<q.transpose() << std::endl;
        // std::cout <<"dis:"<<dist << std::endl;
        // std::cout <<"gradient:"<<gradient.transpose() << std::endl;
        gradient.normalize();
        //    std::cout <<"gradient normalize:"<<gradient.transpose() << std::endl;     
       Eigen::Vector3d obsPoint= q-(dist-0.25)*gradient;
        //    std::cout <<"obsPoint:"<<obsPoint.transpose() << std::endl;
        //     std::cout <<"obsPoint distance:"<<map_module_->getDistance(obsPoint) << std::endl;
       obsPoints_.push_back(obsPoint);
        
        double d=gradient.transpose()*obsPoint;
        CollisionPlanes_[IterNumNow_].col(k-3)=Eigen::Vector4d(gradient(0),gradient(1),gradient(2),-d);
    }

    //会先删除此topic展示的所有对象
    visualization_->drawCollisionPlaneArrows(ctrlPoints_,obsPoints_,0.1,Eigen::Vector4d(0, 1, 1, 1));
    visualization_->drawCollisionPlane(obsPoints_,0.2,Eigen::Vector4d(0, 1, 0, 1));
}


bool casadi_optimizationV2::single_traj_optimization(Eigen::MatrixXd & IterationCtrlPoints)
{

    //========把那该死的数据填进去========================================================================

    //       lbx[60],ubx[60],lbg[242],ubg[242]
    casadi::DM lb_q,lb_vel_constraints,lb_acc_constraints,lb_step_constraints,lb_collision_constraints;
    casadi::DM ub_q,ub_vel_constraints,ub_acc_constraints,ub_step_constraints,ub_collision_constraints;

    //注意此处可引入AABB约束
    lb_q=-inf*casadi::DM::ones(q.rows(),q.columns());
    ub_q=+inf*casadi::DM::ones(q.rows(),q.columns());

    for(int i=0;i<3;i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            lb_q(3*i+j) = ub_q(3*i+j) = IterationCtrlPoints(j,i);
        }
    }

    for(int i=valid_N_+1;i<q.columns();i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            lb_q(3*i+j) = ub_q(3*i+j) = 0;
        }
    }

    //size valid_num_of_segments_-2,9
    lb_vel_constraints= -inf*casadi::DM::ones(vel_constraints.rows(),vel_constraints.columns());
    DM lb_vel_constraints_valid= -2*casadi::DM::ones(valid_num_of_segments_-2,vel_constraints.columns());
    lb_vel_constraints.set(lb_vel_constraints_valid,true,Slice(0,valid_num_of_segments_-2),Slice());
    ub_vel_constraints= +inf*casadi::DM::ones(vel_constraints.rows(),vel_constraints.columns());
    DM ub_vel_constraints_valid= +2*casadi::DM::ones(valid_num_of_segments_-2,vel_constraints.columns());
    ub_vel_constraints.set(ub_vel_constraints_valid,true,Slice(0,valid_num_of_segments_-2),Slice());
        // std::cout<<"lb_vel_constraints:\n"<<lb_vel_constraints<<std::endl;
        // std::cout<<"ub_vel_constraints:\n"<<ub_vel_constraints<<std::endl;

    //(valid_num_of_segments_-2),3
    lb_acc_constraints= -inf*casadi::DM::ones(acc_constraints.rows(),acc_constraints.columns());
    DM lb_acc_constraints_valid= -2*casadi::DM::ones(valid_num_of_segments_-2,acc_constraints.columns());
    lb_acc_constraints.set(lb_acc_constraints_valid,true,Slice(0,valid_num_of_segments_-2),Slice());
    ub_acc_constraints= +inf*casadi::DM::ones(acc_constraints.rows(),acc_constraints.columns());
    DM ub_acc_constraints_valid= +2*casadi::DM::ones(valid_num_of_segments_-2,acc_constraints.columns());
    ub_acc_constraints.set(ub_acc_constraints_valid,true,Slice(0,valid_num_of_segments_-2),Slice());
    //    std::cout<<"lb_acc_constraints:\n"<<lb_acc_constraints<<std::endl;
    //    std::cout<<"ub_acc_constraints:\n"<<ub_acc_constraints<<std::endl;

    lb_step_constraints= -inf*casadi::DM::ones(step_constraints.rows(),step_constraints.columns());
    ub_step_constraints= +inf*casadi::DM::ones(step_constraints.rows(),step_constraints.columns());
    DM ub_step_constraints_valid = casadi::DM::zeros(valid_N_-2,step_constraints.columns());
    ub_step_constraints.set(ub_step_constraints_valid, true,Slice(0,valid_N_-2),Slice());
    //    std::cout<<"lb_step_constraints:\n"<<lb_step_constraints<<std::endl;
    //    std::cout<<"ub_step_constraints:\n"<<ub_step_constraints<<std::endl;

    //N_-4,IterationNum_
    lb_collision_constraints= -inf*casadi::DM::ones(collision_constraints.rows(),collision_constraints.columns());
    ub_collision_constraints= +inf*casadi::DM::ones(collision_constraints.rows(),collision_constraints.columns());
    DM lb_collision_constraints_valid = casadi::DM::zeros(valid_N_-4,IterationNum_);
    lb_collision_constraints.set(lb_collision_constraints_valid,true,Slice(0,valid_N_-4),Slice(0,IterationNum_));
    //    std::cout<<"lb_collision_constraints:\n"<<lb_collision_constraints<<std::endl;
    //    std::cout<<"ub_collision_constraints:\n"<<ub_collision_constraints<<std::endl;

    //p
    casadi::DM p_step,p_traj_goal,p_collision_Planes,p_coefficient_jerk,p_coefficient_goal;

    //FixedN_+1,1 存储有效步长，无效步长为-100
    p_step=-100*casadi::DM::ones(step.rows(),step.columns());
    DM p_step_valid=casadi::DM::zeros(valid_N_+1,1);
    std::copy(Step_.data(),Step_.data()+Step_.size()-2,p_step_valid.ptr());
    p_step.set(p_step_valid,true,Slice(0,valid_N_+1),Slice());
    //    std::cout<<"p_step:\n"<<p_step<<std::endl;

    //存储终点
    p_traj_goal= casadi::DM::zeros(3,1);
    std::copy(traj_goal_.data(),traj_goal_.data()+traj_goal_.size(),p_traj_goal.ptr());

    //size: 4,(FixedN_-4)*IterationNum_
    p_collision_Planes =casadi::DM::zeros(collision_Planes.rows(),collision_Planes.columns());
    for (int i = 0; i <= IterNumNow_; ++i) {
        DM p_collision_Planes_valid = casadi::DM::ones(4,valid_N_-4);
        std::copy(CollisionPlanes_[i].data(),CollisionPlanes_[i].data()+CollisionPlanes_[i].size(),p_collision_Planes_valid.ptr());
        p_collision_Planes.set(p_collision_Planes_valid,true,Slice(),Slice(i*(FixedN_-4),i*(FixedN_-4)+valid_N_-4));
    }
    //    std::cout<<"p_collision_Planes:\n"<<p_collision_Planes<<std::endl;

    //size:Fixednum_of_segments_,1
    p_coefficient_jerk=casadi::DM::zeros(coefficient_jerk.rows(),coefficient_jerk.columns());
    DM p_coefficient_jerk_valid=casadi::DM::ones(valid_num_of_segments_,1);
    p_coefficient_jerk.set(p_coefficient_jerk_valid,true,Slice(0,valid_num_of_segments_));
    //    std::cout<<"p_coefficient_jerk:\n"<<p_coefficient_jerk<<std::endl;

    //size:FixedN_-4,1
    p_coefficient_goal=casadi::DM::zeros(coefficient_goal.rows(),coefficient_goal.columns());
    p_coefficient_goal(valid_N_-4-1)=1;
    //    std::cout<<"p_coefficient_goal:\n"<<p_coefficient_goal<<std::endl;


    //组织计算好的参数
    DMDict arg, res;
    arg["lbx"] = reshape(lb_q,lb_q.numel(),1);
    arg["ubx"] = reshape(ub_q,ub_q.numel(),1);
    arg["lbg"] = DM::vertcat( {reshape(lb_vel_constraints,lb_vel_constraints.numel(),1),
                                  reshape(lb_acc_constraints,lb_acc_constraints.numel(),1),
                                  lb_step_constraints,
                                  reshape(lb_collision_constraints,lb_collision_constraints.numel(),1)} );
    arg["ubg"] = DM::vertcat( {reshape(ub_vel_constraints,ub_vel_constraints.numel(),1),
                                  reshape(ub_acc_constraints,ub_acc_constraints.numel(),1),
                                  ub_step_constraints,
                                  reshape(ub_collision_constraints,ub_collision_constraints.numel(),1)} );
    arg["p"] = DM::vertcat({p_step, p_traj_goal,
                                reshape(p_collision_Planes,p_collision_Planes.numel(),1),
                                p_coefficient_jerk,p_coefficient_goal,p_jerk_bs_basis_matrix_,
                                reshape(p_bs_pos2vel_vel_bs2mv_,bs_pos2vel_vel_bs2mv.numel(),1),
                                reshape(p_bs_pos2vel_bs_vel2acc_,bs_pos2vel_bs_vel2acc.numel(),1)}  );

    DM x0=casadi::DM::zeros(3,FixedCPNum_);
    std::copy(IterationCtrlPoints.data(),IterationCtrlPoints.data()+IterationCtrlPoints.size(),x0.ptr());

    arg["x0"] = reshape(x0,x0.numel(),1);


    res = nlp_solver_(arg);//求解




    if(nlp_solver_.stats()["success"])
    {
        // Conversion DM <--> Eigen:  https://github.com/casadi/casadi/issues/2563
        DM opt = res.at("x");
        auto vector_x = static_cast<std::vector<double>>(opt);
        IterationCtrlPoints=Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>>(vector_x.data(),IterationCtrlPoints.rows(),IterationCtrlPoints.cols());
        std::cout<< nlp_solver_.stats()["return_status"]<<std::endl;
        std::cout <<green<<"IPOPT found a solution" <<reset<< std::endl;
        return true;
    }
    else
    {
        std::cout << red<<"IPOPT failed to find a solution" <<reset<< std::endl;
        // SaveData::saveData("/home/gyl/failure.csv",IterationCtrlPoints);
        // std::cout << red<<"错误结果已存储" <<reset<< std::endl;
        IterationCtrlPoints=IterationCtrlPoints;
        return false;
    }
    
}








