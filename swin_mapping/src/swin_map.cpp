#include "swin_mapping/swin_map.h"


void SWINMap::ComputeVoxelOrigin(const Eigen::Vector3d& new_center, Eigen::Vector3i&  new_lower_left_voxel)
{
    Eigen::Vector3i temp_voxel;
    WorldToVoxel(new_center, temp_voxel);
    for (int i = 0; i < 3; i++)
    {
        new_lower_left_voxel[i] = temp_voxel[i] - (mp_.map_grid_num_[i] >> 1);
    }
    new_lower_left_voxel[2] = temp_voxel[2] - (int)floor(0.7 / mp_.resolution_ - 1e-6);
}

void SWINMap::initMap(ros::NodeHandle& nh)
{
    node_=nh;

    node_.param("swin_map/map_center_x", mp_.map_center_(0),0.0);
    node_.param("swin_map/map_center_y", mp_.map_center_(1),0.0);
    node_.param("swin_map/map_center_z", mp_.map_center_(2),0.0);

    node_.param("swin_map/map_size_x", mp_.map_size_(0), -1.0);//10
    node_.param("swin_map/map_size_y", mp_.map_size_(1), -1.0);//10
    node_.param("swin_map/map_size_z", mp_.map_size_(2), -1.0);//4

    node_.param("swin_map/resolution", mp_.resolution_, -1.0);
    mp_.resolution_inv_=1/ mp_.resolution_;

    for(int i=0; i<3;i++)  mp_.map_grid_num_[i]=mp_.map_size_[i]/mp_.resolution_;

    mp_.map_grid_min_<<0,0,0;
    mp_.map_grid_max_= mp_.map_grid_num_-Eigen::Vector3i::Ones();

    ComputeVoxelOrigin(mp_.map_center_,mp_.map_origin_voxel_);
    mp_.num_cells=mp_.map_grid_num_[0]*mp_.map_grid_num_[1]*mp_.map_grid_num_[2];

    node_.param("swin_map/fx", mp_.fx_, -1.0);
    node_.param("swin_map/fy", mp_.fy_, -1.0);
    node_.param("swin_map/cx", mp_.cx_, -1.0);
    node_.param("swin_map/cy", mp_.cy_, -1.0); 

    node_.param("swin_map/use_depth_filter", mp_.use_depth_filter_, true);
    node_.param("swin_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);//公差0.15
    node_.param("swin_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);//5
    node_.param("swin_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);//0.2
    node_.param("swin_map/depth_filter_margin", mp_.depth_filter_margin_, -1);//余量2
    node_.param("swin_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);//1000
    node_.param("swin_map/skip_pixel", mp_.skip_pixel_, -1);//2 

    node_.param("swin_map/min_ray_length", mp_.min_ray_length_, -0.1);//0.5
    node_.param("swin_map/max_ray_length", mp_.max_ray_length_, -0.1);//4.5
    node_.param("swin_map/inflate_size",mp_.inflate_size_,2);

    node_.param("swin_map/init_value", mp_.init_value, -1.0);
    node_.param("swin_map/hit_inc", mp_.hit_inc, -1.0);
    node_.param("swin_map/miss_inc", mp_.miss_inc, -1.0);
    node_.param("swin_map/occupancy_threshold", mp_.occupancy_threshold, -1.0); 

    node_.param("swin_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);//0.3  esdf切片高度
    node_.param("swin_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);//2.49
    // node_.param("swin_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);//2.5
    // node_.param("swin_map/ground_height", mp_.ground_height_, 1.0);//-1

    node_.param("swin_map/show_occ_time", mp_.show_occ_time_, false);
    node_.param("swin_map/show_esdf_time", mp_.show_esdf_time_, false);

    node_.param("swin_map/frame_id", mp_.frame_id_, string("world"));
    node_.param("swin_map/pose_type", mp_.pose_type_, 1);  //1

   /*MappingData 初始化*/
   md_.occupancy_buffer_=vector<double>(mp_.num_cells, -1);
   md_.occupancy_buffer_neg = vector<char>(mp_.num_cells, 0);
   md_.occupancy_buffer_inflate_ = vector<double>(mp_.num_cells, -1);
  
  vector<Eigen::Vector3d> temp;
  for (int i = 0; i < mp_.num_cells; ++i)  md_.occupancy_inflate_adjoint_.push_back(temp);

   md_.distance_buffer_=vector<double>(mp_.num_cells, 0);
   md_.distance_buffer_neg_=vector<double>(mp_.num_cells, 0);
   md_.distance_buffer_all_=vector<double>(mp_.num_cells, 0);
    md_.tmp_buffer1_ = vector<double>(mp_.num_cells, 0);
    md_.tmp_buffer2_ = vector<double>(mp_.num_cells, 0);

  md_.count_hit_and_miss_ = vector<short>(mp_.num_cells, 0);
  md_.count_hit_ = vector<short>(mp_.num_cells, 0);
  md_.flag_rayend_ = vector<char>(mp_.num_cells, -1);  
  md_.flag_traverse_ = vector<char>(mp_.num_cells, -1);
  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);//640*480/2/2  进行深度滤波
  // md_.proj_points_.resize(640 * 480); //不进行深度滤波
  md_.proj_points_cnt = 0;

  /*ROS*/
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/swin_map/depth", 50));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/swin_map/odom", 100));

    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(&SWINMap::depthOdomCallback, this, _1, _2));

    occ_timer_ = node_.createTimer(ros::Duration(0.01), &SWINMap::updateOccupancyCallback, this);
    esdf_timer_ = node_.createTimer(ros::Duration(0.05), &SWINMap::updateESDFCallback, this);
    vis_timer_ = node_.createTimer(ros::Duration(0.05), &SWINMap::visCallback, this);

    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/swin_map/occupancy", 10);
    map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/swin_map/occupancy_inflate", 10);
    map_range_pub_ = node_.advertise<visualization_msgs::Marker>("/swin_map/map_range", 10);
    esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/swin_map/esdf", 10);
    unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/swin_map/unknown", 10);
    depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/swin_map/depth_cloud", 10);

    md_.occ_need_update_ = false;
    md_.esdf_need_update_ = false;
    // md_.has_first_depth_ = false;
    md_.has_odom_ = false;
    md_.has_cloud_ = false;
    md_.image_cnt_ = 0;

    md_.esdf_time_ = 0.0;
    md_.fuse_time_ = 0.0;
    md_.update_num_ = 0;
    md_.max_esdf_time_ = 0.0;
    md_.max_fuse_time_ = 0.0;
}

void SWINMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom) {
                                 
  /* get pose */
  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;


  md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);


  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
}

void SWINMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.occ_need_update_) return;

  version_++;
  /* update occupancy */
  ros::Time t1, t2;
  t1 = ros::Time::now();
  //更新地图起点，迭代占据栅格地图数据
  UpdateOrigin(md_.camera_pos_);
  // t2 = ros::Time::now();
  // std::cout << "UpdateOrigin Time "<< (t2 - t1).toSec()<<std::endl;
 //深度图投影
  projectDepthImage();

  //光线追踪
  // t1 = ros::Time::now();
  raycastProcess();
  // t2 = ros::Time::now();
  // std::cout << "raycastProcess Time "<< (t2 - t1).toSec()<<std::endl;
   //膨胀障碍物
  // if(1)  InflateObstacles();
  // else     md_.occupancy_buffer_inflate_=md_.occupancy_buffer_;


  t2 = ros::Time::now();

  md_.fuse_time_ += (t2 - t1).toSec();
  md_.update_num_++;
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  if (mp_.show_occ_time_)
    ROS_WARN("Occupancy Time : cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);


  md_.occ_need_update_ = false;
  md_.esdf_need_update_ = true;
}

std::vector<int> SWINMap::UpdateOrigin(const  Eigen::Vector3d& center)
{

  Eigen::Vector3i new_lower_left_voxels;
  Eigen::Vector3i voxel_shift;
  std::vector<int> slice_indexs;
  
  ComputeVoxelOrigin(center, new_lower_left_voxels);
  voxel_shift=new_lower_left_voxels-mp_.map_origin_voxel_;

  if (voxel_shift[0] == 0 && voxel_shift[1] == 0 && voxel_shift[2] == 0)
  {
    return slice_indexs;
  }

  slice_indexs = GetSliceIndexes(new_lower_left_voxels);


 
  mp_.map_center_=center;
  mp_.map_origin_voxel_=new_lower_left_voxels;

 PostShiftOrigin(slice_indexs);
  
  return slice_indexs;
}

std::vector<int> SWINMap::GetSliceIndexes(const Eigen::Vector3i& new_lower_left_voxels)
{
    Eigen::Vector3i clear_width;
    Eigen::Vector3i voxel_shift;
    voxel_shift=new_lower_left_voxels-mp_.map_origin_voxel_;

    for (int i = 0; i < 3; i++)
    {
      clear_width[i] = std::min(abs(voxel_shift[i]), mp_.map_grid_num_[i]);
    }

    std::vector<int> slice_inds;
    for (int i = 0; i < 3; i++)
    {
      if (voxel_shift[i] > 0)
      {
        GetSlice(0, clear_width[i], i, &slice_inds);
      }
      else if (voxel_shift[i] < 0)
      {
        GetSlice(mp_.map_grid_num_[i]- clear_width[i], clear_width[i], i, &slice_inds);
      }
    }
    return slice_inds;
}

void SWINMap::GetSlice(const int i, const int width, const int dimension, std::vector<int>* slice_indexes)
{
  // set minimum dimensions
  int ixyz_min[3] = { 0, 0, 0 };
  ixyz_min[dimension] = i;

  // set max dimensions
  int ixyz_max[3] = { mp_.map_grid_num_[0], mp_.map_grid_num_[1], mp_.map_grid_num_[2] };
  ixyz_max[dimension] = i + width;

  Eigen::Vector3i ixyz;
  for (int ix = ixyz_min[0]; ix < ixyz_max[0]; ix++)
  {
    for (int iy = ixyz_min[1]; iy < ixyz_max[1]; iy++)
    {
      for (int iz = ixyz_min[2]; iz < ixyz_max[2]; iz++)
      {
        ixyz[0] = ix;
        ixyz[1] = iy;
        ixyz[2] = iz;
        int ind;
        GridToIndex(ixyz,ind);
        slice_indexes->push_back(ind);
      }
    }
  }
}

void SWINMap::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
    for (int index : slice_indexes)
    {
        md_.occupancy_buffer_[index] = -1;
        md_.occupancy_buffer_inflate_[index] = -1;
        md_.occupancy_inflate_adjoint_[index].clear();
    }
}

void SWINMap::projectDepthImage()
{
    md_.proj_points_cnt = 0;

    uint16_t* row_ptr;

    int cols = md_.depth_image_.cols;
    int rows = md_.depth_image_.rows;

    double depth;

    Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix(); //四元数转换为旋转矩阵

    Eigen::Vector3d euler_angle(0,0,0);
    Eigen::Matrix3d  Change_Matrix;
    Change_Matrix<<0,0,1,-1,0,0,0,-1,0;

    Change_Matrix = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) * 
                Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitX()) *Change_Matrix;



  //  std::cout << "Change_Matrix: " << Change_Matrix << std::endl;
 
    camera_r=camera_r*Change_Matrix;

   // std::cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

  //不深度滤波
  if (!mp_.use_depth_filter_) 
  {
    for (int v = 0; v < rows; v++) {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u++) {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        if (*row_ptr == 0)   depth = mp_.max_ray_length_ + 0.1;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240) std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_q_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) ; 

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_) {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          if (*row_ptr == 0) {
            depth = mp_.max_ray_length_ + 0.1;
          } else if (depth < mp_.depth_filter_mindist_) {
            continue;
          } else if (depth > mp_.depth_filter_maxdist_) {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame  再次修改
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_; //-0.035;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;
        }
    }
  }

  /* maintain camera pose for consistency check */
  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_q_ = md_.camera_q_;
  md_.last_depth_image_ = md_.depth_image_;
}

void SWINMap::raycastProcess() {

  if (md_.proj_points_cnt == 0) return;

  ros::Time t1, t2;
   

  // std::cout<<"md_.proj_points_cnt: "<<md_.proj_points_cnt<<std::endl;
  
  md_.raycast_num_ += 1;

  int vox_idx;
  double length;


  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);

    } else {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }


    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }


  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_grid_.empty())
   {
      Eigen::Vector3i idx = md_.cache_grid_.front(); //取第一个元素
      
      int idx_ctns;
      GridToIndex(idx,idx_ctns);

      md_.cache_grid_.pop();   //从队列里弹出第一个元素

      //占据信息更新
      double log_odds_update =
          md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
          mp_.hit_inc ://0.4
          mp_.miss_inc;//-0.01

      md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

      //消除栅格的unknown标记 
      if( md_.occupancy_buffer_[idx_ctns]== -1) 
      {
         md_.occupancy_buffer_[idx_ctns]=0;
         md_.occupancy_buffer_inflate_[idx_ctns] = 0;
      } 

      if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] > mp_.occupancy_threshold) 
      {
        md_.occupancy_buffer_[idx_ctns] = std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, 0.0),1.0);
        continue;
      } 
      else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.occupancy_threshold) 
      {
        md_.occupancy_buffer_[idx_ctns] = std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, 0.0),1.0);
        continue;
      }

      
      // double temp_last= md_.occupancy_buffer_[idx_ctns]; //记录上次占据数据

      //限定occupancy_buffer_取值范围于0-1
      md_.occupancy_buffer_[idx_ctns] = std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, 0.0),1.0);
      

      // double temp_now=md_.occupancy_buffer_[idx_ctns];
      //划分点的类别
      // if(temp_point_type=NewAppear)
      // temp_point_type=NewAppear;
      // else if (temp_last<= mp_.occupancy_threshold&&temp_now > mp_.occupancy_threshold)
      // temp_point_type=FisDisappearSecExist; 
      // else if (temp_last> mp_.occupancy_threshold&&temp_now <=mp_.occupancy_threshold)
      // temp_point_type=FisExistSecDisappear; 



    //更新occupancy_buffer_inflate_
      // if (temp_point_type==NewAppear||temp_point_type==FisDisappearSecExist||temp_point_type==FisExistSecDisappear) 
        {

          int inf_step = mp_.inflate_size_;//膨胀大小针对栅格而言

          vector<Eigen::Vector3i> inf_pts;
          Eigen::Vector3i inf_pt;

          inflatePoint(idx, inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf;
            GridToIndex(inf_pt,idx_inf);

            if(md_.occupancy_buffer_[idx_ctns] > mp_.occupancy_threshold) 
            {
              md_.occupancy_buffer_inflate_[idx_inf] = 1;
              Eigen::Vector3d temp_pos;
              GridToWorld(idx,temp_pos);
              md_.occupancy_inflate_adjoint_[idx_inf].push_back(temp_pos);
            } 
            else if(md_.occupancy_buffer_inflate_[idx_inf]==1)
            {
              if(md_.occupancy_inflate_adjoint_[idx_inf].size()!=0)
                {
                  int temp_index;
                  for(int i=0;i<md_.occupancy_inflate_adjoint_[idx_inf].size();i++)
                  {
                     if(i==md_.occupancy_inflate_adjoint_[idx_inf].size()-1) md_.occupancy_buffer_inflate_[idx_inf] = 0;
                      if(isInMap(md_.occupancy_inflate_adjoint_[idx_inf][i])==false)  continue;
                      WorldToIndex(md_.occupancy_inflate_adjoint_[idx_inf][i],temp_index);
                      if(md_.occupancy_buffer_[temp_index]>mp_.occupancy_threshold) break;
                  }
                }   
                else    md_.occupancy_buffer_inflate_[idx_inf] = 0;
            }
            else md_.occupancy_buffer_inflate_[idx_inf] = 0;
            
          }
        }


    }
}
Eigen::Vector3d SWINMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  
  Eigen::Vector3d diff = pt - camera_pt;

  Eigen::Vector3d map_max_boundary ;
  Eigen::Vector3d map_min_boundary ;
  VoxelToWorld(mp_.map_origin_voxel_,map_min_boundary );
  map_min_boundary-=0.5*mp_.resolution_*Eigen::Vector3d::Ones();
  
  map_max_boundary=map_min_boundary+mp_.map_size_;

  Eigen::Vector3d max_tc = map_max_boundary- camera_pt;
  Eigen::Vector3d min_tc = map_min_boundary- camera_pt;
  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

int SWINMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
 {
    if (occ != 1 && occ != 0) return INVALID_IDX;

    Eigen::Vector3i id;
    int idx_ctns;

    WorldToGrid(pos,id);
    GridToIndex(id,idx_ctns);


    md_.count_hit_and_miss_[idx_ctns] += 1;

    if (md_.count_hit_and_miss_[idx_ctns] == 1) 
    {
        md_.cache_grid_.push(id);
    }

    if (occ == 1) md_.count_hit_[idx_ctns] += 1;

    return idx_ctns;
}

void SWINMap::InflateObstacles()
{

    md_.occupancy_buffer_inflate_=md_.occupancy_buffer_;

    int inf_step = 4;//膨胀大小针对栅格而言

    vector<Eigen::Vector3i> inf_pts;
    Eigen::Vector3i inf_pt;

  for (int x = mp_.map_grid_min_(0); x <= mp_.map_grid_max_(0); ++x)
    for (int y = mp_.map_grid_min_(1); y <= mp_.map_grid_max_(1); ++y)
      for (int z = mp_.map_grid_min_(2); z <= mp_.map_grid_max_(2); ++z)
       {
            int temp_index;
            Eigen::Vector3i temp_gird;
            temp_gird<<x,y,z;
            GridToIndex(temp_gird,temp_index);
            if (md_.occupancy_buffer_[temp_index] > mp_.occupancy_threshold) 
            {
                inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

                for (int k = 0; k < (int)inf_pts.size(); ++k) {
                    inf_pt = inf_pts[k];
                    int idx_inf;
                    GridToIndex(inf_pt,idx_inf);

                    if (idx_inf < 0 ||idx_inf >= mp_.num_cells) 
                    {
                    continue;
                    }
                    md_.occupancy_buffer_inflate_[idx_inf] = 1;
                }
            }
        }
}
        
void SWINMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!md_.esdf_need_update_) return;

  /* esdf */
  ros::Time t1, t2;
  t1 = ros::Time::now();

  updateESDF3d();

  t2 = ros::Time::now();

  md_.esdf_time_ += (t2 - t1).toSec();
  md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

  md_.esdf_need_update_ = false;
  need_checkTraj=true;
}

template <typename F_get_val, typename F_set_val>
void SWINMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_grid_num_(dim)];
  double z[mp_.map_grid_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

          
  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }
  
  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SWINMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = mp_.map_grid_min_;
  Eigen::Vector3i max_esdf = mp_.map_grid_max_;

  /* ========== compute positive DT ========== */

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_inflate_[GridToIndex(x, y, z)]>mp_.occupancy_threshold ?
                0 :
                std::numeric_limits<double>::max();//返回 编译器允许的 double 型数 最大值。
          },
          [&](int z, double val) { md_.tmp_buffer1_[GridToIndex(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[GridToIndex(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[GridToIndex(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[GridToIndex(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_[GridToIndex(x, y, z)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val),
                 //      md_.distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = GridToIndex(x, y, z);
        if (md_.occupancy_buffer_inflate_[idx] <= mp_.occupancy_threshold) {
          md_.occupancy_buffer_neg[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] >mp_.occupancy_threshold) {
          md_.occupancy_buffer_neg[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
      }

  ros::Time t1, t2;
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF(
          [&](int z) {
            return md_.occupancy_buffer_neg[GridToIndex(x, y, z)]>mp_.occupancy_threshold ?
                0 :
                std::numeric_limits<double>::max();
          },
          [&](int z, double val) { md_.tmp_buffer1_[GridToIndex(x, y, z)] = val; }, min_esdf[2],
          max_esdf[2], 2);
    }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return md_.tmp_buffer1_[GridToIndex(x, y, z)]; },
               [&](int y, double val) { md_.tmp_buffer2_[GridToIndex(x, y, z)] = val; }, min_esdf[1],
               max_esdf[1], 1);
    }
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return md_.tmp_buffer2_[GridToIndex(x, y, z)]; },
               [&](int x, double val) {
                 md_.distance_buffer_neg_[GridToIndex(x, y, z)] = mp_.resolution_ * std::sqrt(val);
               },
               min_esdf[0], max_esdf[0], 0);
    }
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y)
      for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

        int idx = GridToIndex(x, y, z);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_); 
      }

      // std::cout << "ESDF的最小值为:"<<*std::min_element(md_.distance_buffer_all_.begin(), md_.distance_buffer_all_.end())<< std::endl;
      // std::cout << "ESDF的最大值为:"<<*std::max_element(md_.distance_buffer_all_.begin(), md_.distance_buffer_all_.end())<< std::endl;
      //到此为止，esdf被计算，第一层障碍物计算为0，障碍物内为负，障碍物外为正  论文: Distance Transforms Of Sampled Functions
}

void SWINMap::visCallback(const ros::TimerEvent& /*event*/)
 {
  publishOccMap();
  publishOccMapInflate();
  publishMapRange();
  publishESDF();
  publishUnknown();
  publishDepth();
}

void SWINMap::publishOccMap() 
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_grid_min_;
  Eigen::Vector3i max_cut = mp_.map_grid_max_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (md_.occupancy_buffer_[GridToIndex(x, y, z)] <= mp_.occupancy_threshold) continue;

        Eigen::Vector3d pos;
        GridToWorld(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.points.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);

  //ROS_INFO("pub occupancy map");
}

void SWINMap::publishOccMapInflate() {

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_grid_min_;
  Eigen::Vector3i max_cut = mp_.map_grid_max_;


  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_inflate_[GridToIndex(x, y, z)]  <=mp_.occupancy_threshold) continue;

        Eigen::Vector3d pos;
        GridToWorld(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_) continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub occupancy inflate  map");
}

void SWINMap::publishESDF()
{
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist =  0.0;
  const double max_dist = 0.2;

  //  const double min_dist = *std::min_element(md_.distance_buffer_all_.begin(), md_.distance_buffer_all_.end());
  //  const double max_dist= *std::max_element(md_.distance_buffer_all_.begin(), md_.distance_buffer_all_.end());

  Eigen::Vector3i min_cut = mp_.map_grid_min_ ;

  Eigen::Vector3i max_cut = mp_.map_grid_max_  ;

  std::vector<double> store_esdf;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector3d pos;
      
      GridToWorld(Eigen::Vector3i(x, y, (min_cut(2)+max_cut(2))/2 ), pos);
      pos(2)=mp_.map_center_(2);

      if(mp_.esdf_slice_height_!=999.0)
      {
        pos(2) = mp_.esdf_slice_height_;
      }
      

      dist = getDistance(pos);
      store_esdf.push_back(dist);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

    

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);   //-0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }
      
    Eigen::MatrixXd    store_esdf_temp;
    store_esdf_temp =Eigen::Map<Eigen::Matrix<double,Eigen:: Dynamic,Eigen:: Dynamic, Eigen::RowMajor>>
    (store_esdf.data(), sqrt(store_esdf.size()), sqrt(store_esdf.size()));


  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);
  // ROS_INFO("pub esdf");
}

void SWINMap::publishMapRange()
{

  Eigen::Vector3d map_min_pos, map_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;

  GridToWorld(mp_.map_grid_min_,map_min_pos);
  GridToWorld(mp_.map_grid_max_,map_max_pos);

  cube_pos = 0.5 * (map_min_pos + map_max_pos);
  cube_scale = map_max_pos - map_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  map_range_pub_.publish(mk);
}

void SWINMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut =  mp_.map_grid_min_;
  Eigen::Vector3i max_cut = mp_.map_grid_max_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (md_.occupancy_buffer_[GridToIndex(x, y, z)] == -1) {
          Eigen::Vector3d pos;
          GridToWorld(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > mp_.visualization_truncate_height_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void SWINMap:: publishDepth()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = md_.proj_points_[i][2];
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}


//插值计算pos的ESDF
void SWINMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],Eigen::Vector3d& diff)
{
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  WorldToGrid(pos_m, idx);
  GridToWorld(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        GridToWorld(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }


}

void SWINMap::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2])
{
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        dists[x][y][z] = getDistance(pts[x][y][z]);
      }
    }
  }

}

pair<double, Eigen::Vector3d>  SWINMap::interpolateTrilinear(double values[2][2][2],const Eigen::Vector3d& diff,double& value,Eigen::Vector3d& grad)
{
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_.resolution_inv_;

}

pair<double, Eigen::Vector3d> SWINMap::evaluateEDTWithGrad(const Eigen::Vector3d& pos, double& dist,Eigen::Vector3d& grad)
{
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinear(dists, diff, dist, grad);
}


vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> SWINMap::getObsFromESDF(double threshold)
{

  vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> obs;

  Eigen::Vector3i min_cut = mp_.map_grid_min_;
  Eigen::Vector3i max_cut = mp_.map_grid_max_;


  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        
        if (getDistance(Eigen::Vector3i(x,y,z))>threshold) continue;
        
        Eigen::Vector3d pos;
        GridToWorld(Eigen::Vector3i(x, y, z), pos);
        obs.push_back(pos);
      }


      return obs;


}

