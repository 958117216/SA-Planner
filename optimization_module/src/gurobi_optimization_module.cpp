#include "optimization_module/gurobi_optimization_module.h"


gurobi_optimization::gurobi_optimization(ros::NodeHandle& nh,const SWINMap::Ptr &map,const UUDircCtrlAstar::Ptr &search,const PlanningVisualization::Ptr& visualization)
{
    map_module_=map;
    search_module_=search;
    visualization_=visualization;

    nh.param("gurobi_opt/max_vel",Vmax_,2.0);
    nh.param("gurobi_opt/max_acc",Amax_,2.0);
    nh.param("gurobi_opt/IterationNum",IterationNum_,1);
    nh.param("gurobi_opt/margin",margin_,0.2);
    nh.param("gurobi_opt/reduced_threshold",reduced_threshold_,2.0);
    nh.param("gurobi_opt/reduced_vel",reduced_vel_,0.4);
    nh.param("gurobi_opt/show_rviz",show_rviz_,true);

    es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    std::cout << "Gurobi opt 已初始化完成！"<<std::endl;
}

bool gurobi_optimization::traj_optimization(const Eigen::VectorXd&pos_start,const Eigen::VectorXd &vel_start,const Eigen::VectorXd &acc_start,
                       const Eigen::MatrixXd &ctl_points,Eigen::MatrixXd &opt_ctl_points,
                       const Eigen::RowVectorXd &step,const double &delta_t,const Eigen::VectorXd &pos_goal)
{
    //获得问题信息
    IterNumNow_=0;
    pos_start_=pos_start;
    vel_start_=vel_start;
    acc_start_=acc_start;

    CPNum_=ctl_points.cols();
    N_=CPNum_-1;
    num_of_segments_=CPNum_-3;


    Step_=step;
    delta_t_=delta_t;
    BS2MV=conversion_BS2MV(delta_t);
    traj_goal_=pos_goal;

    // Create the variables: control points
    resetCompleteModel(m_);
    // map_module_->GetMapRange(map_min_pos_, map_max_pos_);
    // std::cout << "temp_origin"<<temp_origin<<"dim"<<temp_dim<<std::endl;
    // m_.set("Method", std::to_string(0));  // verbose (1) or not (0)
    m_.set("OutputFlag", std::to_string(0));  // verbose (1) or not (0)
    // See https://www.gurobi.com/documentation/9.0/refman/cpp_parameter_examples.html
    // m_.set("TimeLimit", std::to_string(0.02));

    std::vector<std::string> coords = { "x", "y", "z" };
    q_.clear();
    for (int i = 0; i <= N_; i++)
    {
        GRBVector q_i;
        for (int j = 0; j < 3; j++)  // x,y,z
        {
            GRBVar tmp;
            tmp = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "q_" + std::to_string(i) + coords[j]);
            // tmp = m_.addVar(map_min_pos_[j], map_max_pos_[j], 0, GRB_CONTINUOUS, "q_" + std::to_string(i) + coords[j]);
            q_i.push_back(GRBLinExpr(tmp));
        }
        q_.push_back(q_i);
    }
    

    addObjective();
    addConstraints();


    bool result;
    while(IterNumNow_<IterationNum_)
    {
        std::cout << "第"<<IterNumNow_<<"次迭代************************************************************"<<std::endl;
        if(IterNumNow_!=0) UpdateStepConstraints(opt_ctl_points); //第一次不用更新步长约束
        result=single_traj_optimization(opt_ctl_points);
        IterNumNow_++;
    }

    if(result==true) return true;
    else return false;


    // m_.optimize();


    // int optimstatus = m_.get(GRB_IntAttr_Status);

    // printGurobiStatus(optimstatus);

    // int number_of_stored_solutions = m_.get(GRB_IntAttr_SolCount);


    // opt_ctl_points.resize(q_[0].size(),q_.size());


    // // See https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html#sec:StatusCodes
    // if ((optimstatus == GRB_OPTIMAL || optimstatus == GRB_TIME_LIMIT ||
    //      optimstatus == GRB_USER_OBJ_LIMIT ||                                    ///////////////
    //      optimstatus == GRB_ITERATION_LIMIT || optimstatus == GRB_NODE_LIMIT ||  ///////////////
    //      optimstatus == GRB_SOLUTION_LIMIT) &&
    //     number_of_stored_solutions > 0)

    // {
    //     std::cout << green << "Gurobi found a solution" << reset << std::endl;
    //     // copy the solution
    //     for(int i=0;i<q_.size();i++) 
    //     {
    //         for(int j=0; j < q_[0].size(); j++)
    //         {
    //             opt_ctl_points(j,i)=q_[i][j].getValue();
    //         }
    //     }

    //     std::cout << "Control_cost_=" << control_cost_.getValue() <<", Terminal cost=" << terminal_cost_.getValue() << std::endl;
    //     std::cout << "Vel0 cost=" << v0_cost_.getValue() << ", Acc0 cost=" << a0_cost_.getValue() <<std::endl;
    //     std::cout << "Total Cost=" << cost_.getValue() << std::endl;
    // }
    // else
    // {
    //     std::cout << red << "Gurobi failed to find a solution, returning" << reset << std::endl;
    //     opt_ctl_points=ctl_points;
    //     return false;
    // }

}

bool gurobi_optimization::single_traj_optimization(Eigen::MatrixXd & IterationCtrlPoints)
{

    m_.optimize();
    // m_.write("/home/gyl/ROS_Projects/GYL_ws/Prometheus/src/Modules/planning/SA_Planner/optimization_module/lp_files/model.lp");

    int optimstatus = m_.get(GRB_IntAttr_Status);
    printGurobiStatus(optimstatus);

    int number_of_stored_solutions = m_.get(GRB_IntAttr_SolCount);


    IterationCtrlPoints.resize(q_[0].size(),q_.size());


    // See https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html#sec:StatusCodes
    if ((optimstatus == GRB_OPTIMAL || optimstatus == GRB_TIME_LIMIT ||
         optimstatus == GRB_USER_OBJ_LIMIT ||                                    ///////////////
         optimstatus == GRB_ITERATION_LIMIT || optimstatus == GRB_NODE_LIMIT ||  ///////////////
         optimstatus == GRB_SOLUTION_LIMIT) &&
        number_of_stored_solutions > 0)

    {
        std::cout << green << "Gurobi found a solution" << reset << std::endl;
        // copy the solution
        for(int i=0;i<q_.size();i++) 
        {
            for(int j=0; j < q_[0].size(); j++)
            {
                IterationCtrlPoints(j,i)=q_[i][j].getValue();
            }
        }

        std::cout << "Control_cost_=" << control_cost_.getValue() <<", Terminal cost=" << terminal_cost_.getValue() << std::endl;
        if( v0_cost_.getValue()<0.1&&a0_cost_.getValue()<0.1) std::cout << green<<"Vel0 cost=" << v0_cost_.getValue() << ", Acc0 cost=" << a0_cost_.getValue() <<reset<<std::endl;
        else  std::cout << red<<"Vel0 cost=" << v0_cost_.getValue() << ", Acc0 cost=" << a0_cost_.getValue() <<reset<<std::endl;
        // std::cout << "Vel0 cost=" << v0_cost_.getValue() << ", Acc0 cost=" << a0_cost_.getValue() <<std::endl;
        std::cout << "Total Cost=" << cost_.getValue() << std::endl;
        return true;
    }
    else
    {
        std::cout << red << "Gurobi failed to find a solution, returning" << reset << std::endl;
        m_.write("/home/gyl/ROS_Projects/GYL_ws/Prometheus/src/Modules/planning/SA_Planner/optimization_module/lp_files/model.lp");
        IterationCtrlPoints=IterationCtrlPoints;
        return false;
    }

}

void gurobi_optimization::addObjective()
{
    control_cost_ = 0.0;
    terminal_cost_ = 0.0;
    cost_ = 0.0;

    Eigen::Matrix4d  pos_bs_basis_matrix;
    pos_bs_basis_matrix<<
            -0.166667, 0.500000, -0.500000, 0.166667,
            0.500000, -1.000000, 0.000000, 0.666667,
            -0.500000, 0.500000, 0.500000, 0.166667,
            0.166667, 0.000000, 0.000000, 0.000000;
    Eigen::Matrix<double, 4, 1> tmp;
    tmp << 6.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<double, -1, 1> jerk_bs_basis_matrix_temp=pos_bs_basis_matrix*tmp;// TODO (this is 4x1)
    
    for (int i = 0; i < num_of_segments_; i++)
    {

        GRBMatrix Q;
        Q.push_back(GRBVector{ q_[i][0], q_[i + 1][0], q_[i + 2][0], q_[i + 3][0] });  // row0
        Q.push_back(GRBVector{ q_[i][1], q_[i + 1][1], q_[i + 2][1], q_[i + 3][1] });  // row1
        Q.push_back(GRBVector{ q_[i][2], q_[i + 1][2], q_[i + 2][2], q_[i + 3][2] });  // row2

        control_cost_ += getNorm2(matrixMultiply(Q, eigenVector2std(jerk_bs_basis_matrix_temp)));
    }

    terminal_cost_ = getNorm2(q_[N_] - eigenVector2std(traj_goal_));

    v0_cost_ = getNorm2( (1/(2*delta_t_))*(q_[2] -q_[0]) - eigenVector2std(vel_start_));

    a0_cost_ = getNorm2( (1/(delta_t_*delta_t_))*(q_[0] -2*q_[1]+q_[2]) - eigenVector2std(acc_start_));

    //构建step约束
    // step_cost_=0;
    // for (int k = 3; k <= N_-2; ++k)
    // {
    //     step_cost_+= getNorm2(q_[k]-q_[k-1])-Step_(k)*Step_(k);
    // }


    cost_ = control_cost_ + 10.0 * terminal_cost_+ 100.0 *v0_cost_ + 100.0 * a0_cost_;
    // cost_ = control_cost_ + 10.0 * terminal_cost_;
    m_.setObjective(cost_, GRB_MINIMIZE);
}

void gurobi_optimization::addVelConstraints()
{
    Eigen::Matrix<double, -1, -1> bs_pos2vel_vel_bs2mv_=BS2MV.bs_pos2vel_*BS2MV.vel_bs2mv_;//4x3矩阵

    for (int i = 0; i < num_of_segments_-2; i++)
    {

        GRBMatrix Q;
        Q.push_back(GRBVector{ q_[i][0], q_[i + 1][0], q_[i + 2][0], q_[i + 3][0] });  // row0
        Q.push_back(GRBVector{ q_[i][1], q_[i + 1][1], q_[i + 2][1], q_[i + 3][1] });  // row1
        Q.push_back(GRBVector{ q_[i][2], q_[i + 1][2], q_[i + 2][2], q_[i + 3][2] });  // row2

        GRBMatrix Qv_MV=matrixMultiply(Q, eigenMatrix2std(bs_pos2vel_vel_bs2mv_));

        for (int j = 0; j < 3; j++)
        {
            addVectorLessEqualConstraint(m_, getColumn(Qv_MV, j), Vmax_*Eigen::Vector3d::Ones());
            addVectorGreaterEqualConstraint(m_, getColumn(Qv_MV, j), -Vmax_*Eigen::Vector3d::Ones());
        }

    }
}

void gurobi_optimization::addAccConstraints()
{
    Eigen::Matrix<double, -1, -1> bs_pos2vel_bs_vel2acc_=BS2MV.bs_pos2vel_*BS2MV.bs_vel2acc_; //4x2矩阵
    for (int i = 0; i < num_of_segments_-2; i++)
    {

        GRBMatrix Q;
        Q.push_back(GRBVector{ q_[i][0], q_[i + 1][0], q_[i + 2][0], q_[i + 3][0] });  // row0
        Q.push_back(GRBVector{ q_[i][1], q_[i + 1][1], q_[i + 2][1], q_[i + 3][1] });  // row1
        Q.push_back(GRBVector{ q_[i][2], q_[i + 1][2], q_[i + 2][2], q_[i + 3][2] });  // row2
        GRBMatrix Qa_BS=matrixMultiply(Q, eigenMatrix2std(bs_pos2vel_bs_vel2acc_));

        addVectorLessEqualConstraint(m_, getColumn(Qa_BS, 1), Amax_*Eigen::Vector3d::Ones());
        addVectorGreaterEqualConstraint(m_, getColumn(Qa_BS, 1), -Amax_*Eigen::Vector3d::Ones());
    }
}

void gurobi_optimization::addStepConstraints()
{
    //构建step约束
    for (int k = 3; k <= N_-2; ++k)
    {
        m_.addQConstr(getNorm2(q_[k]-q_[k-1])<=Step_(k)*Step_(k));
    }
}

void gurobi_optimization::addMICollisionConstraints()
{
    
    //*凸分解 获得SFC
    decomp_util.set_obs(map_module_->getObsFromESDF(0.2));
    // double rs= 1.5* Vmax_*Vmax_/(2*Amax_);
    decomp_util.set_local_bbox(Vec3f(2, 1, 1)); //安全距离 v^2/(2*a)
    vec_Vec3f path=search_module_->getDecompPath(3);
    decomp_util.dilate(path); //凸分解

  //Publish visualization msgs
  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
  es_msg.header.frame_id = "world";
  es_pub.publish(es_msg);

  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
  poly_msg.header.frame_id = "world";
  poly_pub.publish(poly_msg);


  // Convert to inequality constraints Ax < b
    std::vector<LinearConstraint3D>  SFCs;

    auto polys = decomp_util.get_polyhedrons();
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        const auto pt_inside = (path[i] + path[i + 1]) / 2;
        LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
        SFCs.push_back(cs);
    }

    //*添加无碰撞约束
    if(SFCs.size()>0)
    {
        // 添加 binary variables
        b_.clear();
        for (int n_seg = 0; n_seg < num_of_segments_; n_seg++)
        {
            std::vector<GRBVar> row;
            for (int p = 0; p < SFCs.size(); p++)  // For all the polytopes
            {
                GRBVar variable =m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_BINARY, "b" + std::to_string(n_seg) + "_" + std::to_string(p));
                row.push_back(variable);
            }
            b_.push_back(row);
        }


        Eigen::Matrix<double, -1, -1> pos_bs2mv_= BS2MV.pos_bs2mv_;//4x4矩阵
        for (int n_seg = 0; n_seg < num_of_segments_; n_seg++)
        {
            //对于任意一段轨迹，其必须位于至少一个凸多面体内
            GRBLinExpr sum = 0;
            for (int p = 0; p < SFCs.size(); p++)  // For all the polytopes
            {
                sum = sum + b_[n_seg][p];
            }
            m_.addConstr(sum>=1);

            //对于一段轨迹，其位于凸多面体内
            for (int p = 0; p < SFCs.size(); p++)  // For all the polytopes
            {  
                Eigen::MatrixXd A = SFCs[p].A();
                Eigen::VectorXd b = SFCs[p].b();
                
                int i=n_seg; // i 懒得改了

                GRBMatrix Q; //取出第i段轨迹的控制点 qi--qi+3
                Q.push_back(GRBVector{ q_[i][0], q_[i + 1][0], q_[i + 2][0], q_[i + 3][0] });  // row0
                Q.push_back(GRBVector{ q_[i][1], q_[i + 1][1], q_[i + 2][1], q_[i + 3][1] });  // row1
                Q.push_back(GRBVector{ q_[i][2], q_[i + 1][2], q_[i + 2][2], q_[i + 3][2] });  // row2

                GRBMatrix Q_MV=matrixMultiply(Q, eigenMatrix2std(pos_bs2mv_));

                 for (int j = 0; j < 4; j++)
                {
                    GRBVector temp_left = matrixMultiply(eigenMatrix2std(A),getColumn(Q_MV, j)); //凸包的第j个点在凸多面体内

                    for(int i= 0; i< b.rows(); i++) 
                    {
                        m_.addGenConstrIndicator(b_[n_seg][p], 1, temp_left[i], GRB_LESS_EQUAL, b[i]);
                    }
                }               
            }
        }
    }


}

void gurobi_optimization::addCollisionConstraints()
{
    //*凸分解 获得SFC

    decomp_util.set_obs(map_module_->getObsFromESDF(margin_));
    // double rs= 1.5* Vmax_*Vmax_/(2*Amax_);
    double rs=Vmax_*delta_t_;
    // decomp_util.set_local_bbox(Vec3f(rs, rs/2, rs/2)); //安全距离 v^2/(2*a)

    decomp_util.set_local_bbox(Vec3f(2, 2, 1)); //安全距离 v^2/(2*a)
    decomp_util.set_inflate_distance(0.1+1e-4);//屏蔽栅格的影响
    //询问控制点所属阶段 CpStage包含 q0-qn的,  CpStage[i] 为qi所属的第？阶段
    std::vector<int> CpStage=search_module_->RetrieveCtrlNodeStage();
    std::cout << "共应有"<<CpStage.back()+1<<"个凸包"<<std::endl;
    vec_Vec3f path=search_module_->getDecompPath(CpStage.back()+1);

    if(path[0]==path[1])
    {
        std::cout << "分化q2"<<std::endl;
        path[1]=(path[2]-path[0]).normalized()*0.1+path[0];
    }

    decomp_util.dilate(path); //凸分解

  //Publish visualization msgs
  if(show_rviz_)
  {
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
    es_msg.header.frame_id = "world";
    es_pub.publish(es_msg);


    // vec_E<Polyhedron3D>  polys;
    // polys.push_back(decomp_util.get_polyhedrons().back());
    // decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "world";
    poly_pub.publish(poly_msg);
  }


  // Convert to inequality constraints Ax < b
    std::vector<LinearConstraint3D>  SFCs;

    auto polys = decomp_util.get_polyhedrons();
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        const auto pt_inside = (path[i] + path[i + 1]) / 2;
        LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
        SFCs.push_back(cs);
    }

    //*添加无碰撞约束
    Eigen::Matrix<double, -1, -1> pos_bs2mv_= BS2MV.pos_bs2mv_;//4x4矩阵
    for (int i = 2; i <= N_-1; i++)
    {

        //对于一段轨迹，其位于相应的凸多面体内
        Eigen::MatrixXd A = SFCs[CpStage[i]].A();
        Eigen::VectorXd b = SFCs[CpStage[i]].b();
        
        GRBMatrix Q; //取出相应的控制点 qi-2 qi-1 qi qi+1
        Q.push_back(GRBVector{ q_[i-2][0], q_[i - 1][0], q_[i][0], q_[i + 1][0] });  // row0
        Q.push_back(GRBVector{ q_[i-2][1], q_[i - 1][1], q_[i][1], q_[i + 1][1] });  // row1
        Q.push_back(GRBVector{ q_[i-2][2], q_[i - 1][2], q_[i][2], q_[i + 1][2] });  // row2

        GRBMatrix Q_MV=matrixMultiply(Q, eigenMatrix2std(pos_bs2mv_));
        for (int j = 0; j < 4; j++)
        {
            GRBVector temp_left = matrixMultiply(eigenMatrix2std(A),getColumn(Q_MV, j)); //凸包的第j个点在凸多面体内

            for(int k= 0; k< b.rows(); k++) 
            {
                m_.addConstr(temp_left[k]<=b[k]);
            }
        }               
    }


}

void gurobi_optimization::addSimpleMICollisionConstraints()
{
    //*凸分解 获得SFC
    decomp_util.set_obs(map_module_->getObsFromESDF(0.2));
    // double rs= 1.5* Vmax_*Vmax_/(2*Amax_);
    double rs=Vmax_*delta_t_;
    decomp_util.set_local_bbox(Vec3f(rs, rs/2, rs)); //安全距离 v^2/(2*a)

    // decomp_util.set_local_bbox(Vec3f(2, 2, 1)); //安全距离 v^2/(2*a)
    //询问控制点所属阶段 CpStage包含 q0-qn的,  CpStage[i] 为qi所属的第？阶段
    std::vector<int> CpStage=search_module_->RetrieveCtrlNodeStage();
    // 最后一个控制点是朝第CpStage.back() 个目标点扩张获得的
    //一定有CpStage.back() +1个凸包
    std::cout << "希望有"<<CpStage.back()+2<<"个凸包"<<std::endl;
    vec_Vec3f path=search_module_->getDecompPath(CpStage.back()+2);

    int last_poly_th=path.size()-2;//实际有path-1个凸包，最后一个凸包为第last_poly_th个凸包
    //如果last_poly_th=CpStage.back() 说明 希望破灭，没有后续多余的关键点和凸包

    std::cout << "实际有"<<last_poly_th+1<<"个凸包"<<std::endl;
    decomp_util.dilate(path); //凸分解

  //Publish visualization msgs
  if(show_rviz_)
  {
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
    es_msg.header.frame_id = "world";
    es_pub.publish(es_msg);

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "world";
    poly_pub.publish(poly_msg);
  }

  // Convert to inequality constraints Ax < b
    std::vector<LinearConstraint3D>  SFCs;

    auto polys = decomp_util.get_polyhedrons();
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        const auto pt_inside = (path[i] + path[i + 1]) / 2;
        LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
        SFCs.push_back(cs);
    }

    //*添加无碰撞约束
    Eigen::Matrix<double, -1, -1> pos_bs2mv_= BS2MV.pos_bs2mv_;//4x4矩阵
    for (int i = 2; i <= N_-2; i++)
    {

        if(CpStage[i]==0 || CpStage[i]==last_poly_th||CpStage[i]==CpStage[i+1])
        {
            //对于一段轨迹，其位于相应的凸多面体内或相应的凸多面体们的内部
            Eigen::MatrixXd A = SFCs[CpStage[i]].A();
            Eigen::VectorXd b = SFCs[CpStage[i]].b();

            GRBMatrix Q; //取出相应的控制点 qi-2 qi-1 qi qi+1
            Q.push_back(GRBVector{ q_[i-2][0], q_[i - 1][0], q_[i][0], q_[i + 1][0] });  // row0
            Q.push_back(GRBVector{ q_[i-2][1], q_[i - 1][1], q_[i][1], q_[i + 1][1] });  // row1
            Q.push_back(GRBVector{ q_[i-2][2], q_[i - 1][2], q_[i][2], q_[i + 1][2] });  // row2

            GRBMatrix Q_MV=matrixMultiply(Q, eigenMatrix2std(pos_bs2mv_));
            for (int j = 0; j < 4; j++)
            {
                GRBVector temp_left = matrixMultiply(eigenMatrix2std(A),getColumn(Q_MV, j)); //凸包的第j个点在凸多面体内

                for(int k= 0; k< b.rows(); k++) 
                {
                    m_.addConstr(temp_left[k]<=b[k]);
                }
            }      
        }
        else//对中间部分的轨迹段 ,添加混合整数约束
        {
            GRBVar p0 =m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_BINARY, "b" + std::to_string(i-2) + "_0" );
            GRBVar p1 =m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_BINARY, "b" + std::to_string(i-2) + "_1" );
            GRBLinExpr sum =p0+p1;
            m_.addConstr(sum>=1);//局部轨迹至少在一个多面体内

            //对于一段轨迹，其位于凸多面体内
            for (int p = 0; p < 2; p++)  // For all the polytopes
            {  
                Eigen::MatrixXd A = SFCs[CpStage[i]+p].A();
                Eigen::VectorXd b = SFCs[CpStage[i]+p].b();
                

                GRBMatrix Q; //取出相应的控制点 qi-2 qi-1 qi qi+1
                Q.push_back(GRBVector{ q_[i-2][0], q_[i - 1][0], q_[i][0], q_[i + 1][0] });  // row0
                Q.push_back(GRBVector{ q_[i-2][1], q_[i - 1][1], q_[i][1], q_[i + 1][1] });  // row1
                Q.push_back(GRBVector{ q_[i-2][2], q_[i - 1][2], q_[i][2], q_[i + 1][2] });  // row2

                GRBMatrix Q_MV=matrixMultiply(Q, eigenMatrix2std(pos_bs2mv_));

                 for (int j = 0; j < 4; j++)
                {
                    GRBVector temp_left = matrixMultiply(eigenMatrix2std(A),getColumn(Q_MV, j)); //凸包的第j个点在凸多面体内

                    for(int i= 0; i< b.rows(); i++) 
                    {
                        if(p ==0)   m_.addGenConstrIndicator(p0, 1, temp_left[i], GRB_LESS_EQUAL, b[i]);
                        if(p ==1)   m_.addGenConstrIndicator(p1, 1, temp_left[i], GRB_LESS_EQUAL, b[i]);
                    }
                }               
            }

            //对于一段轨迹，其位于相应的凸多面体内或相应的凸多面体们的内部
            // Eigen::MatrixXd A = SFCs[CpStage[i+1]].A();
            // Eigen::VectorXd b = SFCs[CpStage[i+1]].b();

            // GRBMatrix Q; //取出相应的控制点 qi-2 qi-1 qi qi+1
            // Q.push_back(GRBVector{ q_[i-2][0], q_[i - 1][0], q_[i][0], q_[i + 1][0] });  // row0
            // Q.push_back(GRBVector{ q_[i-2][1], q_[i - 1][1], q_[i][1], q_[i + 1][1] });  // row1
            // Q.push_back(GRBVector{ q_[i-2][2], q_[i - 1][2], q_[i][2], q_[i + 1][2] });  // row2

            // GRBMatrix Q_MV=matrixMultiply(Q, eigenMatrix2std(pos_bs2mv_));
            // for (int j = 0; j < 4; j++)
            // {
            //     GRBVector temp_left = matrixMultiply(eigenMatrix2std(A),getColumn(Q_MV, j)); //凸包的第j个点在凸多面体内

            //     for(int k= 0; k< b.rows(); k++) 
            //     {
            //         m_.addConstr(temp_left[k]<=b[k]);
            //     }
            // }      

        }
         
    }



}

void gurobi_optimization::addEndConstraints()
{
    addVectorEqConstraint(m_,q_[N_-1]-q_[N_-2],Eigen::Vector3d::Zero());
    addVectorEqConstraint(m_,q_[N_]-q_[N_-1],Eigen::Vector3d::Zero());
}

void gurobi_optimization::addStartConstraints()
{
    Eigen::Vector3d p0=pos_start_;
    addVectorEqConstraint(m_,0.166666667*q_[0]+0.666666667*q_[1]+0.166666667*q_[2],p0);
    // Eigen::Vector3d v0=vel_start_;
    // addVectorEqConstraint(m_,(1/(2*delta_t_))*(q_[2] -q_[0]),v0);
    // Eigen::Vector3d a0=acc_start_;
    // addVectorEqConstraint(m_,(1/(delta_t_*delta_t_))*(q_[0] -2*q_[1]+q_[2]),a0);
}

void gurobi_optimization::addConstraints()
{
    addVelConstraints();
    addAccConstraints();
    addStepConstraints();

    // addSimpleMICollisionConstraints();
    // addMICollisionConstraints();
    addCollisionConstraints();

    addEndConstraints();
    addStartConstraints();
}

void gurobi_optimization::UpdateStepConstraints(Eigen::MatrixXd ctl_points)
{   
    //更新Step_ 的数据信息
    for (int k = 3; k <=N_-2; ++k)// N_-4
    {
        Eigen::Vector3d q=ctl_points.col(k);
        //此处是否要加判断是否在地图内

        //*更新步长约束
        //1.步长由父节点位置决定
        // Eigen::Vector3d q_father=ctl_points.col(k-1);
        // double dist=map_module_->getDistance(q_father); 
        // Step_[k]=std::max(Step_[k],search_module_->GetStep(dist));

        //2.
        vector<Eigen::Vector3d> LocalCPs;
        LocalCPs.push_back(ctl_points.col(k-3));
        LocalCPs.push_back(ctl_points.col(k-2));
        LocalCPs.push_back(ctl_points.col(k-1));
        Step_[k]=std::max(Step_[k],search_module_->GetStepEASA(LocalCPs));
    }

    //先删除上次的步长约束
    GRBQConstr* cq = 0;
    cq = m_.getQConstrs();
    for (int i = 0; i < m_.get(GRB_IntAttr_NumQConstrs); ++i)
    {
        m_.remove(cq[i]);
    }
    m_.update();
    //添加新的步长约束
    addStepConstraints();
}



