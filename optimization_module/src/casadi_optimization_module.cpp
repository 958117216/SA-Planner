#include "optimization_module/casadi_optimization_module.h"



casadi_optimization::casadi_optimization(const SWINMap::Ptr &map,const PlanningVisualization::Ptr& visualization)
{
    map_module_=map;
    visualization_=visualization;

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
}

bool casadi_optimization::traj_optimization_soft(const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
                                  const Eigen::RowVectorXd &step,const double &delta_t,const Eigen::MatrixXd &pos_goal)
{
    //迭代优化的次数
    IterationNum_=1;

    //从初始解获得问题信息
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

    //从初始解获得问题信息
    num_of_segments_=ctl_points.cols()-3;
    N_=ctl_points.cols()-1;
    traj_goal_=pos_goal;
    Vmax_=2.0;
    Amax_=2.0;
     
    //会随着迭代而改变的问题信息
    CollisionPlanes_.clear();
    std::vector<Eigen::Vector4d> EmptyPlane;
    CollisionPlanes_.assign(N_-4,EmptyPlane);

    initial_FixedStep_=step;

    //优化的控制点    
    opt_ctl_points=ctl_points;
    // SaveData::saveData("/home/gyl/initial.csv",ctl_points);
    //清空变量
    obsPoints_.clear();
    ctrlPoints_.clear();

    bool result;
    while(IterationNum_--)
    {
        std::cout << "倒数第"<<IterationNum_<<"次迭代************************************************************"<<std::endl;
        UpdateStepAndCollision_Constraints(opt_ctl_points);
        result=single_traj_optimization(opt_ctl_points);
    }

    // SaveData::saveData("/home/gyl/桌面/data/opt_ctl_points.csv",opt_ctl_points);
    if(result==true) return true;
    else return false;
}


void casadi_optimization::UpdateStepAndCollision_Constraints(Eigen::MatrixXd ctl_points)
{
    for (int k = 3; k <=N_-2; ++k)// N_-2
    {
        Eigen::Vector3d q=ctl_points.col(k);
        ctrlPoints_.push_back(q);

        //此处是否要加判断是否在地图内
        //步长由父节点位置决定
        Eigen::Vector3d q_father=ctl_points.col(k-1);
        double dist=map_module_->getDistance(q_father); 
        initial_FixedStep_[k]=std::max(initial_FixedStep_[k],GetStep(dist));

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
        CollisionPlanes_[k-3].push_back(Eigen::Vector4d(gradient(0),gradient(1),gradient(2),-d));

    }
    Eigen::MatrixXd temp_CollisionPlanes = Eigen::MatrixXd::Zero(4,CollisionPlanes_.size());;
    for(int i=0;i<CollisionPlanes_.size();i++)
    {
        for (int j = 0; j < 4; ++j)     temp_CollisionPlanes(j,i)=CollisionPlanes_[i][0][j];
    }

    /*存储相应的验证数据*/
    // SaveData::saveData("/home/gyl/桌面/data/ctrlPoints.csv",ctl_points);
    // SaveData::saveData("/home/gyl/桌面/data/step.csv",initial_FixedStep_);
    // SaveData::saveData("/home/gyl/桌面/data/CollisionPlanes.csv",temp_CollisionPlanes);

    //会先删除此topic展示的所有对象
    visualization_->drawCollisionPlaneArrows(ctrlPoints_,obsPoints_,0.1,Eigen::Vector4d(0, 1, 1, 1));
    visualization_->drawCollisionPlane(obsPoints_,0.2,Eigen::Vector4d(0, 1, 0, 1));
}

bool casadi_optimization::single_traj_optimization(Eigen::MatrixXd & IterationCtrlPoints)
{

    //构建NLP
    //x
    casadi::SX q = SX::sym("q",IterationCtrlPoints.rows(),IterationCtrlPoints.cols());
    //g
    casadi::SX vel_constraints=SX::sym("vel_constraints",(num_of_segments_-2)*9,1);
    casadi::SX acc_constraints=SX::sym("acc_constraints",3*(num_of_segments_-2),1);
    casadi::SX step_constraints=SX::sym("step_constraints",N_-4,1);
    casadi::SX end_constraints=SX::sym("end_constraints",6,1);

    int num_temp=0;   for(int i=0;i<CollisionPlanes_.size();i++)   num_temp+=CollisionPlanes_[i].size();
    casadi::SX collision_constraints=SX::sym("collision_constraints",num_temp,1);

    // std::cout << repr(collision_constraints);

    //p
    casadi::SX jerk_bs_basis_matrix= SX::sym("jerk_bs_basis_matrix",4,1);
    casadi::SX traj_goal=SX::sym("traj_goal",3,1);
    casadi::SX bs_pos2vel_vel_bs2mv=SX::sym("bs_pos2vel_vel_bs2mv",4,3);
    casadi::SX bs_pos2vel_bs_vel2acc=SX::sym("bs_pos2vel_bs_vel2acc",4,2);
    casadi::SX collision_Planes=SX::sym("collision_Planes",4,num_temp);

    //构建vel约束
    for (int i = 0; i <num_of_segments_-2; i++)
    {
        vel_constraints(Slice(i*9,i*9+3),Slice())=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(0,1));
        vel_constraints(Slice(i*9+3,i*9+6),Slice())=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(1,2));
        vel_constraints(Slice(i*9+6,i*9+9),Slice())=mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_vel_bs2mv)(Slice(),Slice(2,3));
    }
    //构建acc约束
    for (int i = 0; i < num_of_segments_-2; ++i)
    {
        acc_constraints(Slice(i * 3, i * 3 + 3), Slice()) =mtimes(q(Slice(),Slice(i,i+4)),bs_pos2vel_bs_vel2acc)(Slice(),Slice(1,2));
    }
    //构建step约束
    for (int k = 3; k <= N_-2; ++k)
    {
        step_constraints(k-3)=sumsqr(q(Slice(),Slice(k,k+1))-q(Slice(),Slice(k-1,k)))-initial_FixedStep_[k]*initial_FixedStep_[k];
    }
    //构建end约束
    end_constraints(Slice(0,3),Slice())=q(Slice(),Slice(N_-1,N_-1+1))-q(Slice(),Slice(N_-2,N_-2+1));
    end_constraints(Slice(3,6),Slice())=q(Slice(),Slice(N_,N_+1))-q(Slice(),Slice(N_-1,N_-1+1));
    //构建collision约束
    int count=0;
    for (int i = 3;i <=N_-2; ++i)// N_-2
    {
        for(int j=0;j<CollisionPlanes_[i-3].size();j++)
        {
            collision_constraints(count)= mtimes(q(Slice(),Slice(i,i+1)).T(),collision_Planes(Slice(0,3),Slice(count,count+1)))+collision_Planes(3,count);
            count++;
            // std::cout << "count"<<count;
        }
    }
   

    casadi::SX f;
    //构建目标函数
    // qn-2与终点 的代价
    f=10*sumsqr(q(Slice(),Slice(N_-2,N_-2+1))-traj_goal);
    //Jerk^2
    for (int i = 0; i <num_of_segments_; i++)
    {
        f+=mtimes(mtimes(jerk_bs_basis_matrix.T(),q(Slice(),Slice(i,i+4)).T()),mtimes(q(Slice(),Slice(i,i+4)),jerk_bs_basis_matrix));
    }
    // std::cout <<"f is "<< f;

    casadi::SX x = reshape(q,q.numel(),1);
    casadi::SX p = SX::vertcat({jerk_bs_basis_matrix, traj_goal, reshape(bs_pos2vel_vel_bs2mv,bs_pos2vel_vel_bs2mv.numel(),1),
                                                            reshape(bs_pos2vel_bs_vel2acc,bs_pos2vel_bs_vel2acc.numel(),1),reshape(collision_Planes,collision_Planes.numel(),1)});
    casadi::SX g = SX::vertcat({vel_constraints, acc_constraints, step_constraints,end_constraints,collision_constraints});

    SXDict nlp = {{"x", x},{"p", p},{"f", f},{"g",g}};
    //    ipopt.SetOption("max_cpu_time", 0.01);
//    ipopt.SetOption("max_iter", 20);
    Dict nlp_config_={{"ipopt", Dict({  {"linear_solver", "ma27"},{"max_iter", 50},{"print_level", 5}  })}};
    nlp_solver_=nlpsol("nlp_solver_", "ipopt", nlp,nlp_config_);

    //  std::cout << repr(nlp_solver_);


    //读取数据
    DM lb_q,lb_vel_constraints,lb_acc_constraints,lb_step_constraints,lb_end_constraints,lb_collision_constraints;
    DM ub_q,ub_vel_constraints,ub_acc_constraints,ub_step_constraints,ub_end_constraints,ub_collision_constraints;

    lb_q=-inf*DM::ones(q.rows(),q.columns());
    lb_vel_constraints=-Vmax_*DM::ones(vel_constraints.numel(),1);
    lb_acc_constraints=-Amax_*DM::ones(acc_constraints.numel(),1);
    lb_step_constraints=-inf*DM::ones(step_constraints.numel(),1);
    lb_end_constraints=DM::zeros(end_constraints.numel(),1);
    lb_collision_constraints=DM::zeros(collision_constraints.numel(),1);


    ub_q=inf*DM::ones(q.rows(),q.columns());
    ub_vel_constraints=Vmax_*DM::ones(vel_constraints.numel(),1);
    ub_acc_constraints=Amax_*DM::ones(acc_constraints.numel(),1);
    ub_step_constraints=DM::zeros(step_constraints.numel(),1);
    ub_end_constraints=DM::zeros(end_constraints.numel(),1);
    ub_collision_constraints=inf*DM::ones(collision_constraints.numel(),1);


    for(int i=0;i<3;i++)
    {
        for (int j = 0; j < 3; ++j)
        {
            lb_q(3*i+j) = ub_q(3*i+j) = IterationCtrlPoints(j,i);
        }
    }

   //存储collision planes
   DM p_collision_Planes=casadi::DM::zeros(4,collision_Planes.columns());

    int count_2=0;
    for (int i = 0;i <CollisionPlanes_.size(); ++i)
    {
        for(int j=0;j<CollisionPlanes_[i].size();++j)
        {
            for (int k= 0;k < 4;k++)
            {
                p_collision_Planes(k,count_2)=CollisionPlanes_[i][j](k);
            }
            count_2++;
        }
    }
   std::cout<<"p_collision_Planes is\n"<<p_collision_Planes<<std::endl;

    //存储终点
    p_traj_goal_= casadi::DM::zeros(3,1);
    std::copy(traj_goal_.data(),traj_goal_.data()+traj_goal_.size(),p_traj_goal_.ptr());


    DMDict arg, res;
    arg["lbx"] = reshape(lb_q,lb_q.numel(),1);
    arg["ubx"] = reshape(ub_q,ub_q.numel(),1);
    arg["lbg"] = DM::vertcat({lb_vel_constraints, lb_acc_constraints, lb_step_constraints, lb_end_constraints,lb_collision_constraints});
    arg["ubg"] = DM::vertcat({ub_vel_constraints, ub_acc_constraints, ub_step_constraints, ub_end_constraints,ub_collision_constraints});
    arg["p"] = DM::vertcat({p_jerk_bs_basis_matrix_, p_traj_goal_, reshape(p_bs_pos2vel_vel_bs2mv_,p_bs_pos2vel_vel_bs2mv_.numel(),1),
                                                    reshape(p_bs_pos2vel_bs_vel2acc_,p_bs_pos2vel_bs_vel2acc_.numel(),1),reshape(p_collision_Planes,p_collision_Planes.numel(),1)});

    DM x0(Sparsity::dense(IterationCtrlPoints.rows(),IterationCtrlPoints.cols()));
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
        SaveData::saveData("/home/gyl/failure.csv",IterationCtrlPoints);
        std::cout << red<<"错误结果已存储" <<reset<< std::endl;
        IterationCtrlPoints=IterationCtrlPoints;
        return false;
    }
    
}


