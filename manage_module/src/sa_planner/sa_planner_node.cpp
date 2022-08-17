#include "manage_module/backward.hpp"
#include "manage_module/sa_planner/sa_fsm.h"



/**
 * @brief 段错误查询
 */
namespace backward
{
  backward::SignalHandling sh;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "sa_planner_node");
    ros::NodeHandle nh("~");

    SA_FSM sa_fsm;
    sa_fsm.initFSM(nh);


    ros::Duration(1.0).sleep();
    ros::spin();

  return 0;
}

