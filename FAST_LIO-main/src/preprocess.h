#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <fast_lio/CustomMsg.h>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64}; //{1, 2, 3}
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint}; //点云特征类别
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
  double range; //深度
  double dista; //和同一线内下一个相邻点之间的距离的平方
  double angle[2]; //向量(o->i)和相邻点向量(i->i-1)/(i->i+1)夹角的余弦值
  double intersect; //向量(i->i-1)和向量(i->i+1)夹角的余弦值
  E_jump edj[2]; //i-1和i+1的标记
  Feature ftype; //点云特征
  orgtype() //初始化
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_ros {                           
  struct EIGEN_ALIGN16 Point {  //定义点类型结构, 强制SSE 对齐
      PCL_ADD_POINT4D; // 该点类型有4个元素
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符对齐操作
  };
}  // namespace velodyne_ros

// 注册点类型宏
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
   // (uint16_t, ring, ring)
)


namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)



class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  //void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE; //雷达类型，降采样(采样间隔)，线数，频率
  double blind; //激光雷达近点过滤范围
  bool feature_enabled, given_offset_time; //是否特征选取，是否自带时间戳
  ros::Publisher pub_full, pub_surf, pub_corn;
    

  private:
  //void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  // bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
