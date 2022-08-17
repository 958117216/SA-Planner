#include "manage_module/sa_planner/sa_manager.h"



void SA_Manager::initManager(ros::NodeHandle& nh,const PlanningVisualization::Ptr& visualization)
{
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);//3
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);//2
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);//4
    nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);//6

    local_data_.traj_id_ = 0;   

    map_module_.reset(new SWINMap);
    map_module_->initMap(nh);

    // search_module_.reset(new UUCtrlAstar_Four);
    // search_module_->initSearch(nh,map_module_,visualization);

    // search_module_.reset(new UUCtrlAstar);
    // search_module_->initSearch(nh,map_module_,visualization);

    search_module_.reset(new UUDircCtrlAstar);
    search_module_->initSearch(nh,map_module_,visualization);

    optimization_module_.reset(new optimization(map_module_));
    // casadi_optimization_module_.reset(new casadi_optimization(map_module_,visualization));
    // casadi_optimization_moduleV2_.reset(new casadi_optimizationV2(nh,map_module_,visualization));
    // casadi_optimization_moduleV3_.reset(new casadi_optimizationV3(nh,map_module_,visualization));
    // casadi_optimization_moduleV4_.reset(new casadi_optimizationV4(nh,map_module_,visualization));
    gurobi_optimization_module_.reset(new gurobi_optimization(nh,map_module_,search_module_,visualization));
}

bool SA_Manager::Replan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,Eigen::Vector3d end_pt)
{
    // std::cout << "\n[重规划]: -----------------------" << std::endl;
    std::cout << "起点位置: [" <<start_pt(0)<<", " <<start_pt(1)<<", " <<start_pt(2)<<"]" ;
    printf("速度:[%.2f,%.2f,%.2f]",start_vel(0),start_vel(1),start_vel(2));
    printf("加速度:[%.2f,%.2f,%.2f]\n",start_acc(0),start_acc(1),start_acc(2));
    std::cout << "终点位置: [" <<end_pt(0)<<", " <<end_pt(1)<<", " <<end_pt(2)<<"]" << std::endl;
  
    if ((start_pt - end_pt).norm() > 7.5)  end_pt = start_pt+7.5*(end_pt-start_pt).normalized();

    if ((start_pt - end_pt).norm() < 0.4) {
    std::cout << "Close goal" << std::endl;
    return false;
    }

    ros::Time t1, t2;

    local_data_.start_time_ = ros::Time::now();
    double t_search = 0.0, t_opt = 0.0;

    Eigen::Vector3d init_pos = start_pt;

    //1.路径搜索
    t1 = ros::Time::now();
    search_module_->reset();
    
    Eigen::MatrixXd ctrl_pts;
    Eigen::RowVectorXd ctrl_steps;
    // ctrl_pts=search_module_->search(start_pt, start_vel, start_acc, end_pt);

    ctrl_pts=search_module_->Overall_Search(start_pt, start_vel, start_acc, end_pt);
    if(ctrl_pts.cols()==0)     
    {
      std::cout << termcolor::red<<"[Manager]: 路径搜索失败!" << termcolor::reset <<std::endl;
      return false;
    } else {
       std::cout <<termcolor::green <<"[Manager]: 路径搜索成功!" <<termcolor::reset <<  std::endl;
       ctrl_steps=search_module_->RetrieveCtrlNodeStep();
       std::cout << "ctrl_steps:"<<ctrl_steps.size()<<std::endl;
       std::cout << "ctrl_steps:"<<ctrl_steps<<std::endl;
    }
    t_search = (ros::Time::now() - t1).toSec();

    //2.轨迹优化
     t2 = ros::Time::now();

     Eigen::MatrixXd afterOpt_ctrl_pts=ctrl_pts;
    //*硬约束
    // Eigen::MatrixXd corridors;
    // corridors = GetCorridors (ctrl_pts);
    // local_data_.corriders_=corridors;
    // // if(ctrl_pts.cols()!=0)     
    // // {
    // //   std::cout << "[Manager]: 保存结果!" <<  std::endl;

    // // }

    std::cout << "delta T is "<<search_module_->GetDelta_t()<<std::endl;
    // afterOpt_ctrl_pts=optimization_module_->traj_optimization(ctrl_pts,ctrl_steps,corridors,search_module_->GetDelta_t(),end_pt);

    // casadi_optimization_module_->traj_optimization_soft(ctrl_pts,afterOpt_ctrl_pts,ctrl_steps,search_module_->GetDelta_t(),end_pt);
    // casadi_optimization_moduleV2_->traj_optimization(ctrl_pts,afterOpt_ctrl_pts,ctrl_steps,search_module_->GetDelta_t(),end_pt);
    // std::cout << "delta T is "<<search_module_->GetDelta_t()<<std::endl;
    // casadi_optimization_moduleV3_->traj_optimization(ctrl_pts,afterOpt_ctrl_pts,ctrl_steps,search_module_->GetDelta_t(),end_pt);
    // t_opt=(ros::Time::now() - t2).toSec();


    // std::cout << "delta T is "<<search_module_->GetDelta_t()<<std::endl;
    // casadi_optimization_moduleV4_->traj_optimization(start_pt, start_vel, start_acc,ctrl_pts,afterOpt_ctrl_pts,ctrl_steps,search_module_->GetDelta_t(),end_pt);
   
   gurobi_optimization_module_->traj_optimization(start_pt, start_vel, start_acc,ctrl_pts,afterOpt_ctrl_pts,ctrl_steps,search_module_->GetDelta_t(),end_pt);
   
   t_opt=(ros::Time::now() - t2).toSec();

    //3. 时间调整

   UnclampedUniformBspline opt_pos(afterOpt_ctrl_pts,search_module_->GetDegree(),search_module_->GetDelta_t());

  double to = opt_pos.getTimeSum();

  opt_pos.setPhysicalLimits(search_module_->max_vel_,search_module_->max_acc_);  //在这里添加运动学约束！！！

  bool feasible = opt_pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {

    feasible = opt_pos.reallocateTime();

    if (++iter_num >= 3) break;
  }


    //4.保存结果
    UnclampedUniformBspline pos(ctrl_pts,search_module_->GetDegree(),search_module_->GetDelta_t());
    local_data_.beforeOpt_position_traj_=pos;
    local_data_.position_traj_ = opt_pos;
    double t_total = t_search + t_opt;
    std::cout << "[Manager]: Replan Time(ms): " << t_total*1000.0 << ", search: " << t_search *1000.0<< ", optimize: " << t_opt*1000.0<<  std::endl;

    pp_.time_search_   = t_search;
    pp_.time_optimize_ = t_opt;

    updateTrajInfo();
    return true;
}

void SA_Manager::planYaw(const Eigen::Vector3d& start_yaw)
{
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

//   double dt_yaw  = 0.4; 
//   int    seg_num = ceil(duration / dt_yaw);
  
//   dt_yaw         = duration / seg_num;
  int    seg_num =10;
  double dt_yaw=duration / seg_num;

  const double            forward_t = 4*dt_yaw;
  double                  last_yaw  = start_yaw(0);
  vector<double> waypts;

  for (int i = 0; i < seg_num; ++i)
   {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    double waypt;
    if (pd.norm() > 1e-6) 
    {
      waypt = atan2(pd(1), pd(0));
      calcNextYaw(last_yaw, waypt);  // round yaw to [-PI, PI]
      last_yaw=waypt;

    } else {
      waypt = waypts.back();
    }
  
    waypts.push_back(waypt);
  }
  // std::cout<<"yaw is"<<std::endl;
  //   for (auto i: waypts)  std::cout << i << ' ';


  // calculate initial control points with boundary state constraints
  Eigen::MatrixXd yaw(1,seg_num + 3);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 
      1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 
      1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 
      1.0,dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
      
  yaw.block(0, 0, 1, 3) = (states2pts * start_yaw).transpose();

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - dt_yaw*5);  //duration - 0.1
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);

  calcNextYaw(last_yaw, end_yaw(0));

  yaw.block(0, seg_num, 1, 3) = (states2pts * end_yaw).transpose();

 // solve
 yaw=optimization_module_->yaw_optimization(yaw, dt_yaw,waypts);

  // update traj info
  local_data_.yaw_traj_=UnclampedUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();


  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}


void SA_Manager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

 while (diff < -M_PI) {
    diff += 2 * M_PI;
  }
  while (diff > M_PI) {
    diff -= 2 * M_PI;
  }
    yaw = last_yaw + diff;

}

void SA_Manager::updateTrajInfo() {
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}


//ctrl_pts -> x;y;z;
//Corridors -> lx;ly;lz;ux;uy;uz;
Eigen::MatrixXd SA_Manager::GetCorridors(Eigen::MatrixXd ctrl_pts)
{
  Eigen::MatrixXd corridors(6,ctrl_pts.cols()-5);
  int grid_lx,grid_ux,grid_ly,grid_uy,grid_lz,grid_uz;

  for(int i=3;i<ctrl_pts.cols()-2;i++)
  {
    Eigen::Vector3d pos_temp=ctrl_pts.col(i);
    Eigen::Vector3i  grid_temp= map_module_->WorldToGrid(pos_temp);

    grid_lx=grid_ux=grid_temp(0);
    grid_ly=grid_uy=grid_temp(1);
    grid_lz=grid_uz=grid_temp(2);

    int iter = 0;
    bool collide;
    int  _step_length=2;

    Eigen::Vector3i GridSize=map_module_->getGridSize();

    int id_x,id_y,id_z;

    //开始迭代查询
    while(iter < 20)
    {   
        // Y轴负方向 
        collide  = false;
        int y_lo =  max(0, grid_ly-_step_length);

        for(id_y =grid_ly; id_y >= y_lo; id_y-- )
        {   
            if( collide == true) break;
            
            for(id_x = grid_ux; id_x >= grid_lx; id_x-- )
            {    
                if( collide == true) break;

                for(id_z = grid_uz; id_z >= grid_lz; id_z-- )
                {
                    double dist= map_module_->getDistance(Eigen::Vector3i(id_x, id_y, id_z));
                    if(dist <  0.0) // the voxel is occupied
                    {   
                        collide = true; break;
                    }
                }
            }
        }
        if(id_y==grid_ly) grid_ly=id_y;
        else grid_ly= id_y + 1;

        // Y轴正方向 
        collide  = false;
        int y_up = min(GridSize(1)-1, grid_uy + _step_length);

        for(id_y =grid_uy; id_y<= y_up; id_y++ )
        {   
            if( collide == true) break;
            
            for(id_x = grid_ux; id_x >= grid_lx; id_x-- )
            {    
                if( collide == true) break;

                for(id_z = grid_uz; id_z >= grid_lz; id_z-- )
                {
                    double dist= map_module_->getDistance(Eigen::Vector3i(id_x, id_y, id_z));
                    if(dist <  0.0) // the voxel is occupied
                    {   
                        collide = true; break;
                    }
                }
            }
        }
        if(id_y==grid_uy) grid_uy=id_y;
       else  grid_uy= id_y - 1;

        // X轴负方向 
        collide  = false;
        int x_lo =  max(0, grid_lx-_step_length);

        for(id_x =grid_lx; id_x >= x_lo; id_x-- )
        {   
            if( collide == true) break;
            
            for(id_y = grid_uy; id_y >= grid_ly; id_y-- )
            {    
                if( collide == true) break;

                for(id_z = grid_uz; id_z >= grid_lz; id_z-- )
                {
                    double dist= map_module_->getDistance(Eigen::Vector3i(id_x, id_y, id_z));
                    if(dist <  0.0) // the voxel is occupied
                    {   
                        collide = true; break;
                    }
                }
            }
        }
        if(id_x==grid_lx) grid_lx=id_x;
        else grid_lx= id_x + 1;

        // X轴正方向 
        collide  = false;
        int x_up =  min(GridSize(0)-1,grid_ux+_step_length);

        for(id_x =grid_ux; id_x <= x_up; id_x++ )
        {   
            if( collide == true) break;
            
            for(id_y = grid_uy; id_y >= grid_ly; id_y-- )
            {    
                if( collide == true) break;

                for(id_z = grid_uz; id_z >= grid_lz; id_z-- )
                {
                    double dist= map_module_->getDistance(Eigen::Vector3i(id_x, id_y, id_z));
                    if(dist < 0.0) // the voxel is occupied
                    {   
                        collide = true; break;
                    }
                }
            }
        }

        if(id_x==grid_ux) grid_ux=id_x;
        else grid_ux= id_x - 1;
        

        // Z轴负方向 
        collide  = false;
        int z_lo =  max(0, grid_lz-_step_length);

        for(id_z =grid_lz; id_z >= z_lo; id_z-- )
        {   
            if( collide == true) break;
            
            for(id_y = grid_uy; id_y >= grid_ly; id_y-- )
            {    
                if( collide == true) break;

                for(id_x = grid_ux; id_x >= grid_lx; id_x-- )
                {
                    double dist= map_module_->getDistance(Eigen::Vector3i(id_x, id_y, id_z));
                    if(dist <  0.0) // the voxel is occupied
                    {   
                        collide = true; break;
                    }
                }
            }
        }
        if(id_z==grid_lz) grid_lz=id_z;
        else grid_lz= id_z + 1;


        // Z轴正方向 
        collide  = false;
        int z_up =  min(GridSize(2)-1,grid_uz+_step_length);

        for(id_z =grid_uz; id_z <= z_up; id_z++ )
        {   
            if( collide == true) break;
            
            for(id_y = grid_uy; id_y >= grid_ly; id_y-- )
            {    
                if( collide == true) break;

                for(id_x = grid_ux; id_x >= grid_lx; id_x-- )
                {
                    double dist= map_module_->getDistance(Eigen::Vector3i(id_x, id_y, id_z));
                    if(dist <  0.0) // the voxel is occupied
                    {   
                        collide = true; break;
                    }
                }
            }
        }
         if(id_z==grid_uz) grid_uz=id_z;
        else grid_uz= id_z - 1;

        iter++;
    }

    map_module_->GridToWorld(Eigen::Vector3i(grid_lx,grid_ly,grid_lz),pos_temp);
    corridors.block<3,1>(0,i-3)=pos_temp;
    map_module_->GridToWorld(Eigen::Vector3i(grid_ux,grid_uy,grid_uz),pos_temp);
    corridors.block<3,1>(3,i-3)=pos_temp;
  }
  
  return corridors;
}

