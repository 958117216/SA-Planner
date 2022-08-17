/**
 * @file swin_map.h
 * @author Guo Yilin (958117216@qq.com)
 * @brief   构图模块——构建滚动式占据栅格地图和ESDF地图
 * @version 0.1
 * @date 2022-02-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include "swin_mapping/raycast.h"


using namespace std;


enum  UpdatePointType{NewAppear,FisExistSecDisappear, FisDisappearSecExist };


//地图参数 包括四个层面
//1) 世界 world——double3
//2) 体素 voxel——int3 =世界/分辨率的结果向下取整
//3) 栅格 grid——int3 0~grid_dimensions_[3]   =体素-下界int lower_left_voxels_[3] 
//4) 索引 index——int1 相对于下界的偏移量
struct MappingParameters 
{
  Eigen::Vector3d map_center_; //世界层面下的地图中心 应为无人机位置
  Eigen::Vector3d map_size_;      //世界层面下的地图大小

 double resolution_, resolution_inv_; //世界->体素

  Eigen::Vector3i map_grid_num_;  //栅格层面下的地图大小
  Eigen::Vector3i map_grid_min_; //栅格层面下的地图左下角 ->0,0,0
  Eigen::Vector3i map_grid_max_;//栅格层面下的地图右上角

  Eigen::Vector3i map_origin_voxel_; //体素层面下的地图左下角

    int num_cells;//总的栅格数量

    /* camera parameters */
    double cx_, cy_, fx_, fy_;//相机内参

    /* depth image projection filtering 深度图投影滤波*/ 
    bool use_depth_filter_;//是否使用深度图像滤波
    double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;//深度图像滤波参数
    int depth_filter_margin_;//深度图像滤去的图像边界
    double k_depth_scaling_factor_;  //深度图像像素系数 mm->m
    int skip_pixel_; //深度图像滤波
   
   /*raycasting param光线追踪的参数*/
    double min_ray_length_, max_ray_length_;  // range of doing raycasting
    int inflate_size_; //
   /*概率栅格*/
    double init_value=-1.0;
    double hit_inc=0.4;
    double miss_inc=-0.01;
    double occupancy_threshold=0.6; //[0.0, 1.0]

    /* visualization and computation time display */
    double esdf_slice_height_, virtual_ceil_height_, ground_height_;
    double visualization_truncate_height_; //可视化 <该高度的占据栅格
    bool show_esdf_time_, show_occ_time_; //显示运行时间标志
    string frame_id_; //地图坐标系
    int pose_type_;     //里程计的类型
    string map_input_;  // 1: pose+depth; 2: odom + cloud

};

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance
  //每个体素的占有率和欧氏距离
   
  std::vector<double> occupancy_buffer_;  //占据栅格
  std::vector<char> occupancy_buffer_neg; 
  std::vector<double> occupancy_buffer_inflate_;  //occupancy_buffer_的障碍物膨胀
  std::vector<std::vector<Eigen::Vector3d>> occupancy_inflate_adjoint_;  //occupancy_buffer_的障碍物膨胀的来源节点集


  std::vector<double> distance_buffer_;   //+SDF
  std::vector<double> distance_buffer_neg_; //-SDF
  std::vector<double> distance_buffer_all_;  //最终ESDF地图
  std::vector<double> tmp_buffer1_;  //临时
  std::vector<double> tmp_buffer2_;  //临时

  // 相机位置和姿态
  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Quaterniond camera_q_, last_camera_q_;

  // 深度图
  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  // 构图模块的状态标志位
  bool occ_need_update_, esdf_need_update_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  // depth image projected point cloud 深度图像投影点云
  vector<Eigen::Vector3d> proj_points_; //投影点云
  int proj_points_cnt; //投影点云计数 每次开始前清零

  // flag buffers for speeding up raycasting
  vector<short> count_hit_, count_hit_and_miss_; //统计hit/miss的次数
  vector<char> flag_traverse_;  //过程点是否进行过光追的标志
  vector<char> flag_rayend_;   //在目标点处是否进行过光追的标志
  char raycast_num_; //光追的次数
  queue<Eigen::Vector3i> cache_grid_; //储存需要更新的占据信息的栅格

  // 统计运行时间computation time
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SWINMap
{
public:
        enum { POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000 };
        bool need_checkTraj;
        double version_=0;
public:
         SWINMap(){}
         ~SWINMap(){}

        //*四个层面的转换关系
        inline void WorldToVoxel(const Eigen::Vector3d& pos, Eigen::Vector3i& voxel);
        inline void VoxelToGrid(const Eigen::Vector3i& voxel, Eigen::Vector3i& grid);
        inline void GridToIndex(const Eigen::Vector3i& grid, int& index);

        inline void IndexToGrid(const int& index, Eigen::Vector3i& grid);
        inline void GridToVoxel(const Eigen::Vector3i& grid, Eigen::Vector3i& voxel);
        inline void VoxelToWorld(const Eigen::Vector3i& voxel, Eigen::Vector3d& pos);

        inline void WorldToGrid(const Eigen::Vector3d& pos, Eigen::Vector3i& grid);
        inline Eigen::Vector3i WorldToGrid(const Eigen::Vector3d& pos);
        inline void WorldToIndex(const Eigen::Vector3d& pos, int& index);
       
        inline void GridToWorld(const Eigen::Vector3i& grid,Eigen::Vector3d& pos);
        inline Eigen::Vector3d GridToWorld(const Eigen::Vector3i& grid);
        inline int GridToIndex(const int& x, const int& y,const  int& z);

        //*是否在地图中
        inline bool isInMap(const Eigen::Vector3d& pos);
        inline bool isInMap(const Eigen::Vector3i& grid);
        inline bool isInMap(const int& index);


        //*初始化
        void initMap(ros::NodeHandle& nh);

       //*对外接口
        inline double getDistance(const Eigen::Vector3d& pos);
        inline double getDistance(const Eigen::Vector3i& grid);

        inline Eigen::Vector3d getGradient(const Eigen::Vector3d& pos);
        inline Eigen::Vector3i GridBound(const Eigen::Vector3i& grid);
        
        inline Eigen::Vector3i getGridSize();
        inline int  getGridNum();

        inline void GetMapRange(Eigen::Vector3d& map_min_pos,Eigen::Vector3d& map_max_pos);

        //插值计算pos的ESDF
        void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],Eigen::Vector3d& diff);
        void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
        pair<double, Eigen::Vector3d>  interpolateTrilinear(double values[2][2][2],const Eigen::Vector3d& diff,double& value,Eigen::Vector3d& grad);
        pair<double, Eigen::Vector3d> evaluateEDTWithGrad(const Eigen::Vector3d& pos, double& dist,Eigen::Vector3d& grad);

        //获得障碍点 (ESDF<=threshold)
        vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> getObsFromESDF(double threshold);

        typedef std::shared_ptr<SWINMap> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:  
        void ComputeVoxelOrigin(const Eigen::Vector3d& new_center, Eigen::Vector3i&  new_lower_left_voxel);

        //深度图和里程计同步回调函数
        void depthOdomCallback(const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);

        //更新占据栅格地图
        void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
        //更新ESDF地图
        void updateESDFCallback(const ros::TimerEvent& /*event*/);
        //发布地图
        void visCallback(const ros::TimerEvent& /*event*/);


        //*更新地图起点，迭代占据栅格地图数据
        std::vector<int> UpdateOrigin(const  Eigen::Vector3d& center);
        //得到需要清除的索引
        std::vector<int> GetSliceIndexes(const Eigen::Vector3i& new_lower_left_voxels);
        void GetSlice(const int i, const int width, const int dimension, std::vector<int>* slice_indexes);
        void PostShiftOrigin(const std::vector<int>& slice_indexes);


        //*深度图投影，得到点云
        void projectDepthImage();


        //* 光线追踪
        void raycastProcess();
        Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
        int setCacheOccupancy(Eigen::Vector3d pos, int occ);


        //*膨胀障碍物
        void InflateObstacles();
        inline void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);


        //*计算ESDF地图 
        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
        void updateESDF3d();

        
        //*地图发布模块
        void publishOccMap();
        void publishOccMapInflate();
        void publishESDF();
        void publishMapRange();
        void publishUnknown();
        void publishDepth();


private:
        MappingParameters mp_;
        MappingData md_;

        //ROS参数
        ros::NodeHandle node_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> SyncPolicyImageOdom;
        typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
        SynchronizerImageOdom sync_image_odom_;

        shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
        shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;

        ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, map_range_pub_;
        ros::Publisher unknown_pub_, depth_pub_;
        ros::Timer occ_timer_, esdf_timer_, vis_timer_;
};


inline void SWINMap::WorldToVoxel(const Eigen::Vector3d& pos, Eigen::Vector3i& voxel)
{
    for (int i = 0; i < 3; i++)
    {
        double float_voxels = floor(pos[i] / mp_.resolution_ + 1e-6);
        voxel[i] = static_cast<int>(float_voxels);
    }
}

inline void SWINMap::VoxelToGrid(const Eigen::Vector3i& voxel, Eigen::Vector3i& grid)
{
  grid=voxel-mp_.map_origin_voxel_;
}

inline void SWINMap::GridToIndex(const Eigen::Vector3i& grid, int& index)
{
   Eigen::Vector3i ixyz_offset;
   for(int i=0;i<3;i++)
   {
    ixyz_offset[i]=(grid[i]+mp_.map_origin_voxel_[i])%mp_.map_grid_num_[i]; //叹
    if(ixyz_offset[i]<0)  ixyz_offset[i]+=mp_.map_grid_num_[i];
   }

   //vector 存储顺序 x递增-yz不变
   index=ixyz_offset[0]+ixyz_offset[1]*mp_.map_grid_num_[0]+ixyz_offset[2]*(mp_.map_grid_num_[0]*mp_.map_grid_num_[1]);
    //vector 存储顺序 z递增-yx不变
    //    index=ixyz_offset[2]+ixyz_offset[1]*mp_.map_grid_num_[2]+ixyz_offset[0]*(mp_.map_grid_num_[2]*mp_.map_grid_num_[1]);
}

//为计算ESDF方便
inline int SWINMap::GridToIndex(const int& x, const int& y,const  int& z)
{
   Eigen::Vector3i ixyz_offset;
   Eigen::Vector3i grid; 
   grid<<x,y,z;

   for(int i=0;i<3;i++)
   {
   ixyz_offset[i]=(grid[i]+mp_.map_origin_voxel_[i])%mp_.map_grid_num_[i]; //叹
   if(ixyz_offset[i]<0)  ixyz_offset[i]+=mp_.map_grid_num_[i];
   }

   //vector 存储顺序 x递增-yz不变
   return ixyz_offset[0]+ixyz_offset[1]*mp_.map_grid_num_[0]+ixyz_offset[2]*(mp_.map_grid_num_[0]*mp_.map_grid_num_[1]);
    //vector 存储顺序 z递增-yx不变
    //    return ixyz_offset[2]+ixyz_offset[1]*mp_.map_grid_num_[2]+ixyz_offset[0]*(mp_.map_grid_num_[2]*mp_.map_grid_num_[1]);
}

inline void SWINMap::IndexToGrid(const int& index, Eigen::Vector3i& grid)
{
    int index_=index;

    const int cells_per_floor = mp_.map_grid_num_[0] * mp_.map_grid_num_[1];
    const int cells_per_row = mp_.map_grid_num_[0];
    grid[2] = index / cells_per_floor;
    index_ -= grid[2] * cells_per_floor;
    grid[1] = index_ / cells_per_row;
    index_ -= grid[1] * cells_per_row;
    grid[0] = index_;

    for (int i = 0; i < 3; i++)
    {
    grid[i] = (grid[i] - mp_.map_origin_voxel_[i]) % mp_.map_grid_num_[i];
    if (grid[i] < 0)  grid[i] += mp_.map_grid_num_[i];
    }
   
}

inline void SWINMap::GridToVoxel(const Eigen::Vector3i& grid, Eigen::Vector3i& voxel)
{
        voxel=grid+mp_.map_origin_voxel_;
}

inline void SWINMap::VoxelToWorld(const Eigen::Vector3i& voxel, Eigen::Vector3d& pos)
{
    for(int i = 0; i < 3; i++)
    {
        pos(i)=(voxel(i)+0.5)* mp_.resolution_; 
    }

}

inline void SWINMap::WorldToGrid(const Eigen::Vector3d& pos, Eigen::Vector3i& grid)
{
    Eigen::Vector3i temp_voxel;
    WorldToVoxel(pos, temp_voxel);
    VoxelToGrid(temp_voxel,grid);
}

inline Eigen::Vector3i SWINMap::WorldToGrid(const Eigen::Vector3d& pos)
{
    Eigen::Vector3i temp_grid;
    WorldToGrid(pos,temp_grid);
    return temp_grid;
}

inline void SWINMap::WorldToIndex(const Eigen::Vector3d& pos, int& index)
{
    Eigen::Vector3i temp_voxel,temp_grid;
    WorldToVoxel(pos,temp_voxel);
    VoxelToGrid(temp_voxel,temp_grid);
    GridToIndex(temp_grid,index);
}

inline void SWINMap::GridToWorld(const Eigen::Vector3i& grid,Eigen::Vector3d& pos)
{
    Eigen::Vector3i temp_voxel;
    GridToVoxel(grid,temp_voxel);
    VoxelToWorld(temp_voxel,pos);
}
inline Eigen::Vector3d SWINMap::GridToWorld(const Eigen::Vector3i& grid)
{
    Eigen::Vector3d pos;
    Eigen::Vector3i temp_voxel;
    GridToVoxel(grid,temp_voxel);
    VoxelToWorld(temp_voxel,pos);
    return pos;
}


inline bool SWINMap::isInMap(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i temp_grid;
  WorldToGrid(pos,temp_grid);
  return isInMap(temp_grid);
}

inline bool SWINMap::isInMap(const Eigen::Vector3i& grid)
{
    for (int i = 0; i < 3; i++)
    {
        if (grid[i] < 0 || grid[i] >= mp_.map_grid_num_[i])
        {
            return false;
        }
    }
    return true;
}

inline bool SWINMap::isInMap(const int& index)
{
    return index >= 0 && index <mp_.num_cells;
}

inline void SWINMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   // if (x == 0)
  //   //   continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   // if (y == 0)
  //   //   continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -1; z <= 1; ++z) {
          Eigen::Vector3i temp_point;
          temp_point<<pt(0) + x, pt(1) + y, pt(2) + z;
          if(isInMap(temp_point))  pts.push_back(temp_point);
      }

  /* ---------- inflate  x，y---------- */
  //  for (int x = -step; x <= step; ++x)
  //   for (int y = -step; y <= step; ++y)
  //       pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2));
      
}

inline double SWINMap::getDistance(const Eigen::Vector3d& pos)
{
    int index_temp;
    WorldToIndex(pos,index_temp);
    return  md_.distance_buffer_all_[index_temp];
}
inline double SWINMap::getDistance(const Eigen::Vector3i& grid)
{
    int index_temp;
    GridToIndex(grid,index_temp);
    return  md_.distance_buffer_all_[index_temp];
}

inline Eigen::Vector3d SWINMap::getGradient(const Eigen::Vector3d& pos)
{
    Eigen::Vector3d gradient_temp;
    Eigen::Vector3i  grid_temp;
    WorldToGrid(pos,grid_temp);

    for(int i = 0; i < 3; i++)
    {
        Eigen::Vector3i  positive_grid_temp=grid_temp;
        positive_grid_temp[i]+=1;
        positive_grid_temp = GridBound(positive_grid_temp);

        Eigen::Vector3i negative_grid_temp=grid_temp;
        negative_grid_temp[i]-=1;
        negative_grid_temp = GridBound(negative_grid_temp);
        
        gradient_temp[i]=(getDistance(positive_grid_temp)-getDistance(negative_grid_temp))/(2*mp_.resolution_);
    }
   
   return gradient_temp;

}

inline Eigen::Vector3i SWINMap::GridBound(const Eigen::Vector3i& grid)
{
    Eigen::Vector3i grid_temp=grid;
    for (int i = 0; i < 3; i++)
    {
        if (grid_temp[i] < 0)
        {
            grid_temp[i] =0;
        }else if(grid_temp[i] >= mp_.map_grid_num_[i])
        {
            grid_temp[i]=mp_.map_grid_num_[i]-1;
        }
    }
    return grid_temp;
}

inline Eigen::Vector3i SWINMap::getGridSize()
{
    Eigen::Vector3i grid;

    for(int i = 0; i < 3; i++)
    {
        grid(i) = mp_.map_grid_num_[i];
    }
}

inline int  SWINMap::getGridNum()
{
    return   mp_.map_grid_num_[0]* mp_.map_grid_num_[1]* mp_.map_grid_num_[2];
}


inline void SWINMap::GetMapRange(Eigen::Vector3d& map_min_pos,Eigen::Vector3d& map_max_pos)
{
    GridToWorld(mp_.map_grid_min_,map_min_pos);
    GridToWorld(mp_.map_grid_max_,map_max_pos);
}