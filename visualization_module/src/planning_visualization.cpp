#include "visualization_module/planning_visualization.h"

PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh)
{
  node = nh;

 //颜色初始化
Eigen::Vector4d Red, Orange, Yellow, Green, Blue, Indigo, Purple;
Red<<1,0,0,1;Orange<<1,0.5,0,1;Yellow<<1,1,0,1;Green<<0,1,0,1;Blue<<0,0,1,1;Indigo<<0,1,1,1;Purple<<1,0,1,1;

Colors_Formula_.push_back(Red);
Colors_Formula_.push_back(Orange);
Colors_Formula_.push_back(Yellow);
Colors_Formula_.push_back(Green);
Colors_Formula_.push_back(Blue);
Colors_Formula_.push_back(Indigo);
Colors_Formula_.push_back(Purple);

  //Ros初始化
  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 20);
  pubs_.push_back(traj_pub_);

  checkTraj_pub = node.advertise<visualization_msgs::Marker>("/planning_vis/check_trajectory", 20);
  pubs_.push_back(checkTraj_pub);

  corridor_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/corridor", 20);
  pubs_.push_back(corridor_pub_);

  path_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/path", 20);
  pubs_.push_back(path_pub_);

  expanded_nodes_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/expanded_nodes", 20);
  pubs_.push_back(expanded_nodes_pub_);

  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 20);
  pubs_.push_back(yaw_pub_);

  
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                                              const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::CUBE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1,
                                            const vector<Eigen::Vector3d>& list2, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::LINE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}


 void PlanningVisualization::displaySingleArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end, double resolution,
                      const Eigen::Vector4d& color,int id,int pub_id)
{
  visualization_msgs::Marker mk_state;
  
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = resolution;
  mk_state.scale.y = resolution+0.1;
  mk_state.scale.z = resolution+0.2;

  geometry_msgs::Point pt;
  pt.x = start(0);
  pt.y = start(1);
  pt.z = start(2);
  mk_state.points.push_back(pt);

  pt.x = end(0);
  pt.y = end(1);
  pt.z = end(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  pubs_[pub_id].publish(mk_state);
}


  //*****************************************pub_id 发布者的ID 为 Traj=0*******************************************************
void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  displaySphereList(goal_vec, resolution, color, GOAL + id % 100,Traj);
}

void PlanningVisualization::drawBspline(UnclampedUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts, double size2,
                                        const Eigen::Vector4d& color2, int id1, int id2) {
  if (bspline.getControlPoint().size() == 0) return;

  std::vector<Eigen::Vector3d> traj_pts;
  double                  tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100,Traj);

 vector<Eigen::VectorXd> knotps;
 vector<Eigen::Vector3d> ktp;
 knotps=bspline.calKnotPoints();

  for (int i = 0; i < int(knotps.size()); ++i) {
    Eigen::Vector3d pt = knotps[i];
    ktp.push_back(pt);
  }
displaySphereList(ktp, size2, Colors_Formula_[Purple], PATH + id2 % 100,Traj);

  // draw the control point
  if (!show_ctrl_pts) return;

  Eigen::MatrixXd         ctrl_pts = bspline.getControlPoint();
  vector<Eigen::Vector3d> ctp;

  for (int i = 0; i < int(ctrl_pts.cols()); ++i) {
    Eigen::Vector3d pt = ctrl_pts.col(i);
    ctp.push_back(pt);
  }

  displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100,Traj);
}


  //*****************************************pub_id 发布者的ID 为 CheckTraj=1*******************************************************
void PlanningVisualization::drawCheckTraj(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id ,int pub_id)
{
  displaySphereList(path, resolution, color, PATH + id % 100,pub_id);
}


  //*****************************************pub_id 发布者的ID 为 Corridor=2*******************************************************
void PlanningVisualization::drawCollisionPlane(const vector<Eigen::Vector3d>& path, double resolution,const Eigen::Vector4d& color, int id)
{
  displaySphereList(path, resolution, color, PATH + id % 100,Corridor);
}


void PlanningVisualization::drawCollisionPlaneArrows(const vector<Eigen::Vector3d>& starts, const vector<Eigen::Vector3d>& ends, double resolution,const Eigen::Vector4d& color, int id)
{

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.action          = visualization_msgs::Marker::DELETEALL;
  pubs_[Corridor].publish(mk);

    for (int i = 0; i <starts.size(); i++)
    {
        displaySingleArrow(starts[i], ends[i],resolution,color,BSPLINE+id % 100,Corridor);
        id++;
    }

    // displayLineList(starts,ends,resolution,color,BSPLINE+id % 100, Corridor);
}




  //*****************************************pub_id 发布者的ID 为 Path=3*******************************************************
void PlanningVisualization::drawSearchStart(Eigen::Vector3d start, double resolution, const Eigen::Vector4d& color, int id )
{
  vector<Eigen::Vector3d> start_temp = { start };
  displaySphereList(start_temp, resolution, color, GOAL + id % 100,Path);
}


void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d>& path, double resolution,const Eigen::Vector4d& color, int id) {
  displayCubeList(path, resolution, color, PATH + id % 100,Path);
}


  //*****************************************pub_id 发布者的ID 为  ExpNodes=4*******************************************************
void PlanningVisualization::drawVisitedNode(const vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id ,int pub_id)
{
   displayCubeList(path, resolution, color, BSPLINE_CTRL_PT + id % 100,pub_id);
}

void PlanningVisualization::drawLayeredVisitedNode(const vector<vector<Eigen::Vector3d>>& path, double resolution,
                          Eigen::Vector4d color, int id ,int pub_id)
{
  //  displayCubeList(path[0], resolution, Colors_Formula_[Red],  BSPLINE_CTRL_PT +  id % 100,pub_id);
  //  displayCubeList(path[1], resolution, Colors_Formula_[Orange],  BSPLINE_CTRL_PT + (id+1)% 100,pub_id);
  //  displayCubeList(path[2], resolution, Colors_Formula_[Yellow],  BSPLINE_CTRL_PT + (id+2) % 100,pub_id);
  //  displayCubeList(path[3], resolution, Colors_Formula_[Green],  BSPLINE_CTRL_PT + (id+3) % 100,pub_id);
  //  displayCubeList(path[4], resolution, Colors_Formula_[Blue],  BSPLINE_CTRL_PT + (id+4) % 100,pub_id);

   displayCubeList(path[0], resolution, Colors_Formula_[Orange],  BSPLINE_CTRL_PT + (id+1)% 100,pub_id);
   displayCubeList(path[1], resolution, Colors_Formula_[Yellow],  BSPLINE_CTRL_PT + (id+2) % 100,pub_id);
   displayCubeList(path[2], resolution, Colors_Formula_[Green],  BSPLINE_CTRL_PT + (id+3) % 100,pub_id);
   displayCubeList(path[3], resolution, Colors_Formula_[Blue],  BSPLINE_CTRL_PT + (id+4) % 100,pub_id);
}