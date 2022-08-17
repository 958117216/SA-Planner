#include "manage_module/sa_planner/sa_fsm.h"


void SA_FSM::initFSM(ros::NodeHandle& nh)
{
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  trigger_     = false;
  have_target_ = false;
  have_odom_   = false;


  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);//1 手动选择目标 munual target
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);//1.5
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);//2
  nh.param("fsm/waypoint_num", waypoint_num_, -1);  //2
  nh.param("fsm/checkTrajCollision_dist", checkTrajCollision_distance_, 1.0); 
  
  for (int i = 0; i < waypoint_num_; i++)
  {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */ 
  visualization_.reset(new PlanningVisualization(nh));
  sa_manager_.reset(new SA_Manager);//指针重新指向（赋值）
  sa_manager_->initManager(nh,visualization_);


  /* 定时器 */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &SA_FSM::execFSMCallback, this);//定时0.01s
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &SA_FSM::checkCollisionCallback, this);//定时0.05s

// 订阅指定主题，并指定回调函数，1000为队列大小，当我们来不及处理消息时，会存储在该队列中，若队列元素大于1000，则会抛弃老的消息
  waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &SA_FSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &SA_FSM::odometryCallback, this); //话题已重映射

  // 该句告诉master主控节点，我们将在chatter主题中发布std_msgs的String消息，在我们发布消息时，
  // 主控节点将会告知所有订阅该主题的节点，消息队列大小为1000，即在队列里有消息超过1000个之后，才会丢弃以前老的消息
  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<manage_module::Bspline>("/planning/bspline", 10);
}

void SA_FSM::waypointCallback(const nav_msgs::PathConstPtr& msg) 
{
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout <<green <<"Triggered!" <<reset<<endl;
  trigger_ = true;


  if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
  {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
  } 
  
  else if (target_type_ == TARGET_TYPE::PRESET_TARGET) 
  {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }
  else if (target_type_ == TARGET_TYPE::INCREASE_TARGET) 
  {
    end_pt_(0)  =odom_pos_(0)+ waypoints_[current_wp_][0];
    end_pt_(1)  =odom_pos_(1)+  waypoints_[current_wp_][1];
    end_pt_(2)  =odom_pos_(2)+  waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  cout <<green<< "[Trig] 目标点已经获取:"<< end_pt_[0] << " [m] "<<  end_pt_[1]  << " [m] "<<  end_pt_[2]  << " [m] "<<reset<<endl;
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));  
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "触发");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "触发");
}

void SA_FSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) 
{
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void SA_FSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{
    string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
    // string state_str[5] = { "初始化", "等待目标", "规划新轨迹", "重规划轨迹", "执行轨迹" };
    int    pre_s        = int(exec_state_);
    exec_state_         = new_state;
    cout <<cyan<<"[CHGFSM] Called by"<< "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << reset<<endl;
}

void SA_FSM::printFSMExecState() {
    string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
    // string state_str[5] = { "初始化", "等待目标", "规划新轨迹", "重规划轨迹", "执行轨迹" };

    cout << magenta<<"[FSM]: state: " + state_str[int(exec_state_)] <<reset<< endl;
}

bool SA_FSM::callManagerReplan()
{
     bool plan_success = sa_manager_->Replan(start_pt_, start_vel_, start_acc_, end_pt_);

    if (plan_success) 
    {
        //*规划Yaw 
        sa_manager_->planYaw(start_yaw_);
        
        auto info =&sa_manager_->local_data_;
        manage_module::Bspline bspline;
        bspline.degree = 3;
        bspline.traj_id    = info->traj_id_;
        bspline.start_time = info->start_time_;

        //*Yaw 
        Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
        for (int i = 0; i < yaw_pts.cols(); ++i) {
          double yaw = yaw_pts(0,i);
          bspline.yaw_pts.push_back(yaw);
        }
        bspline.yaw_dt = info->yaw_traj_.getInterval();

        //*Pos
        Eigen::MatrixXd ctrl_pts = info->position_traj_.getControlPoint();
        for (int i = 0; i < ctrl_pts.cols(); i++) {
            geometry_msgs::Point pt;
            pt.x = ctrl_pts(0, i);
            pt.y = ctrl_pts(1, i);
            pt.z = ctrl_pts(2, i);
            bspline.ctrl_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        for (int i = 0; i < knots.rows(); ++i) {
            bspline.knots.push_back(knots(i));
        }
        bspline_pub_.publish(bspline);


        /* visulization */
        // visualization_->drawVisitedNode(info->VisitedNodes,0.1,visualization_->Colors_Formula_[visualization_->Purple]);
        // visualization_->drawLayeredVisitedNode(info->VisitedNodes_TimeKind,0.1);
        visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.15,
                                        Eigen::Vector4d(1, 0, 0, 1));
       visualization_->drawBspline(info->beforeOpt_position_traj_, 0.1,  visualization_->Colors_Formula_[visualization_->Yellow], true, 0.15,
                                        visualization_->Colors_Formula_[visualization_->Yellow],10,10);

        return true;
    }else {
    cout<<red << "generate new traj fail." << reset<<endl;
    return false;
    }
}

void SA_FSM::execFSMCallback(const ros::TimerEvent& e)
 {  

  static int fsm_num = 0, fsm_num2=0;
  fsm_num++;  fsm_num2++;
  if (fsm_num == 150)
   {
    printFSMExecState();
    if (!have_odom_) cout << red<<"no odom." << reset<<endl;
    fsm_num = 0;
  }

  switch(exec_state_) 
  {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);//起始于(0，0)，提取块大小为(3，1)	
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callManagerReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        std::cout<<red<< "  规划失败 , 转入等待新的目标点"<<reset<<std::endl;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        // changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData *info = &sa_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      Eigen::Vector3d LocalTraj_goal = info->position_traj_.evaluateDeBoorT(info->duration_-0.001); //这段轨迹的终点

      /* && t_cur > info->duration_ - 1e-2*/
      //转换判断  是否已抵达目标点，抵达则等待下一个目标点
      if ( (end_pt_ - pos).norm() < 0.4 ) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if (( pos- end_pt_).norm() < 2.0 )    // (( LocalTraj_goal- end_pt_).norm() < no_replan_thresh_ ) //当前轨迹终点（算出来） 是否已抵达    规划终点附近end_pt_
      {
      if(fsm_num2 > 50)//50*0.01=0.5s
      {
        fsm_num2 = 0;
        cout <<bold<<white<< "当前轨迹的终点靠近规划终点,不再重规划!" <<reset<< endl;
      }
        return;

      } else if( t_cur <info->duration_*0.2 && (pos - info->start_pos_ ).norm() < 2.0)          //(LocalTraj_goal - pos).norm() >replan_thresh_||      //((pos - info->start_pos_ ).norm() < replan_thresh_ && t_cur <( info->duration_ - info->duration_*0.8) )   //(info->start_pos_ - pos).norm() < replan_thresh_  
       {     
         if(fsm_num2 > 50)
         {
          fsm_num2 = 0;         
          cout <<bold<<white<< " 当前轨迹没有走完,不再重规划" << reset<< endl;
         }
        return;
      
      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &sa_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
     

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      // start_vel_.setZero();
      // start_acc_.setZero();

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];
      

      // start_pt_  = (info->position_traj_.evaluateDeBoorT(t_cur)-odom_pos_)*0.3+odom_pos_;
      //  start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      
      //  start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
      //  start_pt_= odom_pos_;
      //  start_vel_ = odom_vel_;

    //   start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
    //   start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
    //   start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callManagerReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }

}

void SA_FSM::checkCollisionCallback(const ros::TimerEvent &e)
{
  auto info = &sa_manager_->local_data_;
  bool safe=true;
  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ)   //&&sa_manager_->map_module_->need_checkTraj)
  {
    sa_manager_->map_module_->need_checkTraj=false;
    std::vector<Eigen::Vector3d> check_pts;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = min(info->duration_, t_cur);

    double deltaT=sa_manager_->search_module_->GetDelta_t();
    for( t_cur; t_cur< (info->duration_-2*deltaT); t_cur+=0.1)
    {
        Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
        check_pts.push_back(pos);
        
        double dist = sa_manager_->map_module_->getDistance(pos);
        if(dist<=checkTrajCollision_distance_) 
        {
            safe=false;
            cout<<bold<<red << "current traj in DANGER !!!" <<reset<< endl;
            break;
        } 
    }
      
      // Eigen::MatrixXd pos_ctrl = info->position_traj_.getControlPoint();
      // for(int i=3;i<pos_ctrl.cols()-2;i++)
      // {
      //   Eigen::Vector3d qi=pos_ctrl.col(i);
      //   Eigen::Vector3d qI=pos_ctrl.col(i+1);
      //   double step=sa_manager_->search_module_->GetStep(sa_manager_->map_module_->getDistance(qi));
      //   if( (qI-qi).norm()-step >0.3 )//1e-5
      //   {
      //       // safe=false;
      //       // cout<<"第"<<i<<"个控制点"<<std::endl;
      //       // cout<<"(qI-qi).norm() :"<<(qI-qi).norm() << "step is" <<step<< endl;
      //       // cout<<"(qI-qi).norm()-step :"<<(qI-qi).norm()-step<< endl;
      //       // cout<<bold<<red << "current traj in velocity DANGER !!!" <<RESET<< endl;
      //       break;
      //   }
      // }



    visualization_->drawCheckTraj(check_pts,0.15,Eigen::Vector4d(0, 0, 1, 1),0);

    if (safe)
    {
      // cout<<BOLDWHITE << "current traj is SAFE!!!" <<RESET<< endl;
    }
    else
    {
      //visualization_->drawGoal(sa_manager_->collision_point, 0.18, Eigen::Vector4d(0, 0, 1, 1.0));  
     // cout<<BOLDRED << "current traj in DANGER !!!" <<RESET<< endl;
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      ///如果轨迹检测有碰撞，则转入等待状态
      // std_msgs::Empty new_msg;
      // new_pub_.publish(new_msg);
      // have_target_=false;
      // changeFSMExecState(WAIT_TARGET, "SAFETY");
    }
   }
}
