#include"optimization_module/casadi_optimization_moduleV4.h"


casadi_optimizationV4::casadi_optimizationV4(ros::NodeHandle& nh,const SWINMap::Ptr &map,const PlanningVisualization::Ptr& visualization)
{
    map_module_=map;
    visualization_=visualization;

    nh.param("casadi_optV4/max_vel",Vmax_,2.0);
    nh.param("casadi_optV4/max_acc",Amax_,2.0);
    nh.param("casadi_optV4/IterationNum",IterationNum_,1);
    nh.param("casadi_optV4/MaxCPNum",MaxCPNum_,20);
    nh.param("casadi_optV4/margin",margin_,0.2);
    nh.param("casadi_optV4/reduced_threshold",reduced_threshold_,2.0);
    nh.param("casadi_optV4/reduced_vel",reduced_vel_,0.4);
    nh.param("casadi_optV4/show_rviz",show_rviz_,true);

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

    std::cout << "opt 已初始化完成！"<<std::endl;
}

void casadi_optimizationV4::BuildNLP()
{
    for (int v=6; v<=MaxCPNum_ ;v++)
    {
        //构建NLP
        CPNum_=v;
        N_=CPNum_-1;  
        num_of_segments_=CPNum_-3; //CPNum_-p

        //x
        casadi::SX q = SX::sym("q",3,CPNum_);
        //g
        //速度约束和加速度约束 均以轨迹的段数划分，每一段为一行，将该段的所有约束都放入这一行,均不包含最后两点的减速过程
        casadi::SX vel_constraints=SX::sym("vel_constraints",num_of_segments_-2,9);
        casadi::SX acc_constraints=SX::sym("acc_constraints",num_of_segments_-2,3);
        //步长约束均以控制点划分，在这里直接去除永远不会考虑的点
        casadi::SX step_constraints=SX::sym("step_constraints",N_-4,1);
        casadi::SX end_constraints=SX::sym("end_constraints",6,1);
        //碰撞约束，对每一段轨迹的最小凸包点进行约束
        casadi::SX collision_constraints=SX::sym("collision_constraints",4*num_of_segments_,IterationNum_);
        casadi::SX start_constraints=SX::sym("start_constraints",3,1);

        //p
        casadi::SX step=SX::sym("step",N_+1,1); //每个节点的step, N_+1的目的是对应搜索的结果
        casadi::SX traj_goal=SX::sym("traj_goal",3,1);
        casadi::SX collision_Planes=SX::sym("collision_Planes",4,num_of_segments_*IterationNum_);
        casadi::SX initial_pos=SX::sym("initial_pos",3,1);
        casadi::SX initial_vel=SX::sym("initial_vel",3,1);
        casadi::SX initial_acc=SX::sym("initial_acc",3,1);
        casadi::SX deltaT=SX::sym("deltaT",1,1);

        casadi::SX jerk_bs_basis_matrix= SX::sym("jerk_bs_basis_matrix",4,1);//纯系数
        casadi::SX bs_pos2vel_vel_bs2mv=SX::sym("bs_pos2vel_vel_bs2mv",4,3);
        casadi::SX bs_pos2vel_bs_vel2acc=SX::sym("bs_pos2vel_bs_vel2acc",4,2);
        casadi::SX pos_bs2mv_=SX::sym("pos_bs2mv_",4,4);

        //构建vel约束
        for (int i = 0; i <num_of_segments_-2; i++)
        {
            vel_constraints(Slice(i,i+1),Slice(0,3))=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(0,1)).T();
            vel_constraints(Slice(i,i+1),Slice(3,6))=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(1,2)).T();
            vel_constraints(Slice(i,i+1),Slice(6,9))=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(2,3)).T();
        }
        //构建acc约束
        for (int i = 0; i < num_of_segments_-2; ++i)
        {
            acc_constraints(Slice(i,i+1), Slice()) =mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_bs_vel2acc)(Slice(),Slice(1,2)).T();
        }
        //构建step约束
        for (int k = 3; k <= N_-2; ++k)
        {
            step_constraints(k-3)=sumsqr(q(Slice(),Slice(k,k+1))-q(Slice(),Slice(k-1,k)))-step(k)*step(k);
        }
        //构建end约束
        end_constraints(Slice(0,3),Slice())=q(Slice(),Slice(N_-1,N_-1+1))-q(Slice(),Slice(N_-2,N_-2+1));
        end_constraints(Slice(3,6),Slice())=q(Slice(),Slice(N_,N_+1))-q(Slice(),Slice(N_-1,N_-1+1));

        //构建collision约束
        for (int i = 0; i < IterationNum_; ++i) //第i次迭代
        {
            for (int k = 0; k <num_of_segments_; ++k)
            {
               collision_constraints(Slice(k*4,k*4+4),Slice(i,i+1))=mtimes(mtimes(q(Slice(),Slice(k,k+4)),pos_bs2mv_).T(),collision_Planes(Slice(0,3),Slice(k+num_of_segments_*i,k+num_of_segments_*i+1)))
                                                                                                                            +collision_Planes(3,k+num_of_segments_*i);
            }
        }

        //构建start约束
        start_constraints=(0.166666667*q(Slice(),Slice(0,1))+0.666666667*q(Slice(),Slice(1,2))+0.166666667*q(Slice(),Slice(2,3)))-initial_pos;

        casadi::SX f=0;
        //构建目标函数
        // qn-2与终点 的代价
        f+=10*sumsqr(q(Slice(),Slice(N_-2,N_-2+1))-traj_goal);

        //Jerk^2
        for (int i = 0; i <num_of_segments_; i++)
        {
            f+=sumsqr(mtimes(q(Slice(),Slice(i,i+4)),jerk_bs_basis_matrix));
        }
        //速度
        f+=100*sumsqr(((q(Slice(),Slice(2,3))-q(Slice(),Slice(0,1)))/(2*deltaT))-initial_vel);
        //加速度
        f+=100*sumsqr(((q(Slice(),Slice(0,1))-2*q(Slice(),Slice(1,2))+q(Slice(),Slice(2,3)))/(deltaT*deltaT))-initial_acc);
        //    std::cout <<"f is "<< f<<std::endl;


        casadi::SX x = reshape(q,q.numel(),1);
        casadi::SX p = SX::vertcat({step, traj_goal,
                                    reshape(collision_Planes,collision_Planes.numel(),1),
                                    initial_pos,initial_vel,initial_acc,deltaT,
                                    jerk_bs_basis_matrix,
                                    reshape(bs_pos2vel_vel_bs2mv,bs_pos2vel_vel_bs2mv.numel(),1),
                                    reshape(bs_pos2vel_bs_vel2acc,bs_pos2vel_bs_vel2acc.numel(),1),
                                    reshape(pos_bs2mv_,pos_bs2mv_.numel(),1)});

        casadi::SX g = SX::vertcat({reshape(vel_constraints,vel_constraints.numel(),1),
                                    reshape(acc_constraints,acc_constraints.numel(),1),
                                    step_constraints,end_constraints,
                                    reshape(collision_constraints,collision_constraints.numel(),1),
                                    start_constraints});

        SXDict nlp = {{"x", x},{"p", p},{"f", f},{"g",g}};
        Dict nlp_config_={{"ipopt", Dict({  {"linear_solver", "ma27"},{"max_iter", 50},{"print_level", 0}  })}}; //
        casadi::Function  nlp_solver_ = nlpsol("nlp_solver_", "ipopt", nlp,nlp_config_);

        nlp_solvers_.push_back(nlp_solver_);
        std::cout << repr(nlp_solver_)<<std::endl;
    }

}


bool casadi_optimizationV4::traj_optimization(const Eigen::MatrixXd &pos_start,const Eigen::MatrixXd &vel_start,const Eigen::MatrixXd &acc_start,const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
                                      const Eigen::RowVectorXd &step,const double &delta_t,const Eigen::MatrixXd &pos_goal)
{
    std::cout << "casadi_optimizationV4 优化"<<std::endl;

    IterNumNow_=0;
    map_module_->GetMapRange(map_min_pos_, map_max_pos_);

    //获得问题信息
    p_initial_pos_=casadi::DM::zeros(3,1);
    std::copy(pos_start.data(),pos_start.data()+pos_start.size(),p_initial_pos_.ptr());

    p_initial_vel_=casadi::DM::zeros(3,1);
    std::copy(vel_start.data(),vel_start.data()+vel_start.size(),p_initial_vel_.ptr());

    p_initial_acc_=casadi::DM::zeros(3,1);
    std::copy(acc_start.data(),acc_start.data()+acc_start.size(),p_initial_acc_.ptr());

    delta_t_=delta_t;
    p_deltaT_=delta_t_*casadi::DM::ones(1,1);

   BS2MV=conversion_BS2MV(delta_t);
   p_pos_bs2mv_=casadi::DM::zeros(4,4);
    std::copy(BS2MV.pos_bs2mv_.data(),BS2MV.pos_bs2mv_.data()+BS2MV.pos_bs2mv_.size(),
            p_pos_bs2mv_.ptr());

    p_bs_pos2vel_vel_bs2mv_=casadi::DM::zeros(4,3);
    Eigen::Matrix<double, 4, 3> bs_pos2vel_vel_bs2mv_tmp=BS2MV.bs_pos2vel_*BS2MV.vel_bs2mv_;
    std::copy(bs_pos2vel_vel_bs2mv_tmp.data(),bs_pos2vel_vel_bs2mv_tmp.data()+bs_pos2vel_vel_bs2mv_tmp.size(),
            p_bs_pos2vel_vel_bs2mv_.ptr());

    p_bs_pos2vel_bs_vel2acc_=casadi::DM::zeros(4,2);
    Eigen::Matrix<double, 4, 2> bs_pos2vel_bs_vel2acc_tmp=BS2MV.bs_pos2vel_*BS2MV.bs_vel2acc_;
    std::copy(bs_pos2vel_bs_vel2acc_tmp.data(),bs_pos2vel_bs_vel2acc_tmp.data()+bs_pos2vel_bs_vel2acc_tmp.size(),
            p_bs_pos2vel_bs_vel2acc_.ptr());

    //存储终点
     traj_goal_=pos_goal;
    p_traj_goal_= casadi::DM::zeros(3,1);
    std::copy(traj_goal_.data(),traj_goal_.data()+traj_goal_.size(),p_traj_goal_.ptr());

    //获得问题信息
    CPNum_=ctl_points.cols();
    N_=CPNum_-1;
    num_of_segments_=CPNum_-3;


    //会随着迭代而改变的问题信息
    CollisionPlanes_.clear();
    Eigen::MatrixXd EmptyPlane; 
    EmptyPlane.resize(4,num_of_segments_);
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


void casadi_optimizationV4::UpdateStepAndCollision_Constraints(Eigen::MatrixXd ctl_points)
{
    for (int k = 3; k <=N_; ++k)// N_-4
    {
        Eigen::Vector3d q=ctl_points.col(k);
        ctrlPoints_.push_back(q);

        //此处是否要加判断是否在地图内

        //*更新步长约束
        //1.步长由父节点位置决定
        // Eigen::Vector3d q_father=ctl_points.col(k-1);
        // double dist=map_module_->getDistance(q_father); 
        // Step_[k]=std::max(Step_[k],GetStep(dist));

        //2.
        vector<Eigen::Vector3d> LocalCPs;
        LocalCPs.push_back(ctl_points.col(k-3));
        LocalCPs.push_back(ctl_points.col(k-2));
        LocalCPs.push_back(ctl_points.col(k-1));
        Step_[k]=std::max(Step_[k],GetStepEASA(LocalCPs));

       

        //*更新碰撞平面
        Eigen::Matrix<double,3,4> bs_cpts,mv_cpts;
        bs_cpts.col(0)=ctl_points.col(k-3);
        bs_cpts.col(1)=ctl_points.col(k-2);
        bs_cpts.col(2)=ctl_points.col(k-1);
        bs_cpts.col(3)=ctl_points.col(k-0);

        mv_cpts=bs_cpts*BS2MV.pos_bs2mv_;

        std::vector<double> temp_ESDF;
        for(int i=0;i<4;i++)
        {
            Eigen::Vector3d q_temp=mv_cpts.col(i);
            temp_ESDF.push_back(map_module_->getDistance(q_temp));
        }
        int minPosition = min_element(temp_ESDF.begin(),temp_ESDF.end()) - temp_ESDF.begin();
        q=mv_cpts.col(minPosition);

        Eigen::Vector3d gradient=map_module_->getGradient(q);
        double dist_q=map_module_->getDistance(q); 
        map_module_->evaluateEDTWithGrad(q,dist_q,gradient);
        gradient=map_module_->getGradient(q);
        
        // std::cout <<"ctlPoint:"<<q.transpose() << std::endl;
        // std::cout <<"dis:"<<dist << std::endl;
        // std::cout <<"gradient:"<<gradient.transpose() << std::endl;
        gradient.normalize();
        //    std::cout <<"gradient normalize:"<<gradient.transpose() << std::endl;     
       Eigen::Vector3d obsPoint= q-(dist_q-margin_)*gradient;
        //    std::cout <<"obsPoint:"<<obsPoint.transpose() << std::endl;
        //     std::cout <<"obsPoint distance:"<<map_module_->getDistance(obsPoint) << std::endl;
       obsPoints_.push_back(obsPoint);
        
        double d=gradient.transpose()*obsPoint;
        // CollisionPlanes_[0].col(k-3)=Eigen::Vector4d(gradient(0),gradient(1),gradient(2),-d);
         CollisionPlanes_[IterNumNow_].col(k-3)=Eigen::Vector4d(gradient(0),gradient(1),gradient(2),-d);
    }

    //会先删除此topic展示的所有对象
    if(show_rviz_)
    {
        visualization_->drawCollisionPlaneArrows(ctrlPoints_,obsPoints_,0.05,Eigen::Vector4d(0, 1, 1, 1));
        visualization_->drawCollisionPlane(obsPoints_,0.1,Eigen::Vector4d(0, 1, 0, 1));
    }

}


bool casadi_optimizationV4::single_traj_optimization(Eigen::MatrixXd & IterationCtrlPoints)
{

    //========把那该死的数据填进去========================================================================
    casadi::DM lb_q,lb_vel_constraints,lb_acc_constraints,lb_step_constraints,lb_end_constraints,lb_collision_constraints,lb_start_constraints;
    casadi::DM ub_q,ub_vel_constraints,ub_acc_constraints,ub_step_constraints,ub_end_constraints,ub_collision_constraints,ub_start_constraints;

    //注意此处引入AABB约束
    lb_q=-inf*casadi::DM::ones(3,CPNum_);
    ub_q=+inf*casadi::DM::ones(3,CPNum_);

    for(int i=0;i<3;i++)
    {
        casadi::DM temp=map_min_pos_(i)*casadi::DM::ones(1,CPNum_);
        lb_q.set(temp,true,Slice(i,i+1),Slice());
    }
    for(int i=0;i<3;i++)
    {
        casadi::DM temp=map_max_pos_(i)*casadi::DM::ones(1,CPNum_);
        ub_q.set(temp,true,Slice(i,i+1),Slice());
    }
    //固定初始p个控制点
    // for(int i=0;i<3;i++)
    // {
    //     for (int j = 0; j < 3; ++j)
    //     {
    //         lb_q(3*i+j) = ub_q(3*i+j) = IterationCtrlPoints(j,i);
    //     }
    // }


    //size num_of_segments_-2,9
    lb_vel_constraints= -Vmax_*casadi::DM::ones(num_of_segments_-2,9);
    ub_vel_constraints= +Vmax_*casadi::DM::ones(num_of_segments_-2,9);

        // std::cout<<"lb_vel_constraints:\n"<<lb_vel_constraints<<std::endl;
        // std::cout<<"ub_vel_constraints:\n"<<ub_vel_constraints<<std::endl;

    //num_of_segments_-2,3
    lb_acc_constraints= -Amax_*casadi::DM::ones(num_of_segments_-2,3);
    ub_acc_constraints= +Amax_*casadi::DM::ones(num_of_segments_-2,3);

    //    std::cout<<"lb_acc_constraints:\n"<<lb_acc_constraints<<std::endl;
    //    std::cout<<"ub_acc_constraints:\n"<<ub_acc_constraints<<std::endl;

    //N_-4,1
    lb_step_constraints= -inf*casadi::DM::ones(N_-4,1);
    ub_step_constraints= casadi::DM::zeros(N_-4,1);
    //    std::cout<<"lb_step_constraints:\n"<<lb_step_constraints<<std::endl;
    //    std::cout<<"ub_step_constraints:\n"<<ub_step_constraints<<std::endl;
    lb_end_constraints=DM::zeros(6,1);
    ub_end_constraints=DM::zeros(6,1);

    // lb_end_constraints=1e-2*DM::ones(6,1);
    // ub_end_constraints=1e-2*DM::ones(6,1);
    //N_-4,IterationNum_
    //4*num_of_segments_,IterationNum_
    lb_collision_constraints= casadi::DM::zeros(4*num_of_segments_,IterationNum_);
    ub_collision_constraints= +inf*casadi::DM::ones(4*num_of_segments_,IterationNum_);
    //    std::cout<<"lb_collision_constraints:\n"<<lb_collision_constraints<<std::endl;
    //    std::cout<<"ub_collision_constraints:\n"<<ub_collision_constraints<<std::endl;

    lb_start_constraints=DM::zeros(3,1);
    ub_start_constraints=DM::zeros(3,1);

    //p, 此处仅填入会迭代更新的部分
    casadi::DM p_step,p_collision_Planes;

    //N_+1,1 
    p_step= casadi::DM::zeros(N_+1,1);
    std::copy(Step_.data(),Step_.data()+Step_.size(),p_step.ptr());
    //    std::cout<<"p_step:\n"<<p_step<<std::endl;

    //size: 4,num_of_segments_*IterationNum_
    p_collision_Planes =casadi::DM::zeros(4,num_of_segments_*IterationNum_);
    for (int i = 0; i <= IterNumNow_; ++i) {
        DM p_collision_Planes_valid = casadi::DM::ones(4,num_of_segments_);
        std::copy(CollisionPlanes_[i].data(),CollisionPlanes_[i].data()+CollisionPlanes_[i].size(),p_collision_Planes_valid.ptr());
        p_collision_Planes.set(p_collision_Planes_valid,true,Slice(),Slice(i*num_of_segments_,i*num_of_segments_+num_of_segments_));
    }
    //    std::cout<<"p_collision_Planes:\n"<<p_collision_Planes<<std::endl;


    //组织计算好的参数
    DMDict arg, res;
    arg["lbx"] = reshape(lb_q,lb_q.numel(),1);
    arg["ubx"] = reshape(ub_q,ub_q.numel(),1);
    arg["lbg"] = DM::vertcat( {reshape(lb_vel_constraints,lb_vel_constraints.numel(),1),
                                  reshape(lb_acc_constraints,lb_acc_constraints.numel(),1),
                                  lb_step_constraints,lb_end_constraints,
                                  reshape(lb_collision_constraints,lb_collision_constraints.numel(),1),
                                  lb_start_constraints} );
                            
    arg["ubg"] = DM::vertcat( {reshape(ub_vel_constraints,ub_vel_constraints.numel(),1),
                                  reshape(ub_acc_constraints,ub_acc_constraints.numel(),1),
                                  ub_step_constraints,ub_end_constraints,
                                  reshape(ub_collision_constraints,ub_collision_constraints.numel(),1),
                                  ub_start_constraints} );

    arg["p"] = DM::vertcat({p_step, p_traj_goal_,
                                reshape(p_collision_Planes,p_collision_Planes.numel(),1),
                                p_initial_pos_,p_initial_vel_,p_initial_acc_,p_deltaT_,
                                p_jerk_bs_basis_matrix_,
                                reshape(p_bs_pos2vel_vel_bs2mv_,p_bs_pos2vel_vel_bs2mv_.numel(),1),
                                reshape(p_bs_pos2vel_bs_vel2acc_,p_bs_pos2vel_bs_vel2acc_.numel(),1),
                                reshape(p_pos_bs2mv_,p_pos_bs2mv_.numel(),1)}  );


    DM x0=casadi::DM::zeros(3,CPNum_);
    std::copy(IterationCtrlPoints.data(),IterationCtrlPoints.data()+IterationCtrlPoints.size(),x0.ptr());

    arg["x0"] = reshape(x0,x0.numel(),1);


    res = nlp_solvers_[CPNum_-6](arg);//求解


    if( nlp_solvers_[CPNum_-6].stats()["success"])
    {
        // Conversion DM <--> Eigen:  https://github.com/casadi/casadi/issues/2563
        DM opt = res.at("x");
        auto vector_x = static_cast<std::vector<double>>(opt);
        IterationCtrlPoints=Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>>(vector_x.data(),IterationCtrlPoints.rows(),IterationCtrlPoints.cols());
        std::cout<<  nlp_solvers_[CPNum_-6].stats()["return_status"]<<std::endl;
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









