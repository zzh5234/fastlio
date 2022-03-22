#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  //disA= 0.1; // B?
  disB = 0.1;
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

//设置参数
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

/*
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);
  *pcl_out = pl_surf;
}
*/

//描述:从原始点云中提取特征点
//输入:1.原始点云消息msg;
//输出:1.提取的特征点云pcl_out;
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case OUST64:
    // oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;  //只有平面点？
}

/*
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;
  
  if (feature_enabled)
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

        bool is_new = false;
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    for(int j=0; j<N_SCANS; j++)
    {
      if(pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types);
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}
*/

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t / 1e6; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}


void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size(); //一帧点云中点的数量
    if (plsize == 0) return;
    //reserve是容器预留空间，但在空间内不真正创建元素对象，所以在没有添加新的对象之前，不能引用容器内的元素。
    //加入新的元素时，要调用push_back()/insert()函数。
    //resize是改变容器的大小，且创建对象，因此，调用这个函数之后，就可以引用容器内的对象了，
    //加入新的元素时，用operator[]操作符，或者用迭代器来引用元素对象。此时再调用push_back()函数，是加在这个新的空间后面的。
    pl_surf.reserve(plsize);

    //没有时间戳
    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE; // scan angular velocity，每ms转的角度
    std::vector<bool> is_first(N_SCANS,true); //判断是否为每线第一个点
    std::vector<double> yaw_fp(N_SCANS, 0.0); // yaw of first scan point，每线第一个扫描点的角度
    std::vector<float> yaw_last(N_SCANS, 0.0); // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0); // last offset time
    /*****************************************************************/

    //如果每个点自带时间戳
    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    //如果每个点不自带时间戳
    //寻找第0个点所在的那一线的开始和结束扫描角度,作为一帧点云的开始和结束扫描角度
    else
    {
      given_offset_time = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578; //180/PI = 57.29578
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      //寻找和第0个点同一线的最后一个点
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    //如果提取特征点
    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        //每一线都预留一帧点云的空间大小
        pl_buff[i].reserve(plsize);
      }
      
      //计算每个点的相对(每帧第一个点的)扫描时间,并按线数存储点云
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer  = pl_orig.points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms

        //如果每个点不自带时间戳,需要根据水平旋转角度进行计算
        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          //每一线第一个点
          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle; //存储每一线第一个点的水平旋转角度
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle; 
              time_last[layer]=added_pt.curvature; //上一个点的相对(每一线第一个点的)扫描时间
              continue;
          }
          //lidar原始点云的坐标系是前左上系,即角度增长方向为逆时针,
          //但lidar的扫描方向(从+z向-z看去)是顺时针,因此时间戳越往后的点角度应该越小
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;//omega_l表示每ms转的角度
          }
          //一线中的每个点水平角度都应该小于等于第一个点的水平角度,如果不符,就要补偿周期性
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          //如果这一点比上一点的时间快，说明超过了起始点，需要补偿周期
          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;//一线中最后一点的角度
          time_last[layer]=added_pt.curvature;//上一个点的相对(每一线第一个点的)扫描时间
        }

        //把带有相对扫描时间的点按线数存储
        //这里上面的计算用相对每一线第一个点的扫描时间来近似相对每帧点云第一个点的扫描时间
        pl_buff[layer].points.push_back(added_pt);
      }

      //遍历每一线,分别提取特征点
      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j]; //一线点云
        int linesize = pl.size(); //每一线点数
        if (linesize < 2) continue; //跳过点数过少的扫描线
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y); //每个点的水平深度
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz; //和下一个点之间的距离的平方
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);//加上最后一个点

        //从一线中提取特征点,平面特征点存入pl_surf,角特征点存入pl_corn
        give_feature(pl, types);
      }
    }

    //如果不提取特征点
    //没有时间戳的情况计算同上
    else
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = pl_orig.points[i].time / 1000.0;  // curvature unit: ms

        if (!given_offset_time)
        {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;
              is_first[layer]=false;
              added_pt.curvature = 0.0;
              yaw_last[layer]=yaw_angle;
              time_last[layer]=added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer]=added_pt.curvature;
        }

        //没有特征提取直接进行降采样，最后放入面点中
        if (i % point_filter_num == 0)
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }
}


//描述:从一线点云中提取特征点
//输入:1.pl:一线点云;
//          2.types:对应于pl存储每个点的深度和每个点距离下一个点的距离的平方;
//输出:1.pl_surf:平面特征点云;
//          2.pl_corn:角特征点云;
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size(); //一线点云数量
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
 //跳过深度在盲区阈值内的点
  while(types[head].range < blind)
  {
    head++;
  }

  //--------------------遍历每一个点,尝试提取平面特征--------------------//
  // Surf
  //提取的平面特征线段最少为group_size个点,因此要判断一线点云个数是否大于group_size
  //plsize2为选取特征线段的起点，所以最后留有group_size个点
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  //遍历每一个点,尝试提取平面特征,结果存入types中
  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    //从一线点云中提取起点为i的平面特征线段;
    //(i_nex-1)为提取的平面特征线段的终点在一线中的索引,curr_direct为所提取的平面特征线段的单位方向向量
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    //1代表成功提取了平面特征线段
    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          //平面特征线段的中间点标记为Real_Plane
          types[j].ftype = Real_Plane;
        }
        else
        {
          //平面特征线段的两个端点标记为Poss_Plane,
          //起点的标记在本次会被修改为Edge_Plane或Real_Plane,
          //终点的标记在本次不会被修改,在继续往后提取的过程中,
          //如果终点往后还是平面特征,那么终点的标记会被修改为Real_Plane或者Edge_Plane
          //如果终点往后不是平面特征,那么终点的标记不会被修改,仍旧是Poss_Plane
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        //本次和上一次提取的平面特征线段的单位法向量点乘
        double mod = last_direct.transpose() * curr_direct;
        //如果和前一个方向向量夹角大于45°,说明是一个新的小平面,因此第一个点就是平面的边缘,即Edge_Plane
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        //如果法向量一致,说明是前一个平面的延伸,因此是真平面点Real_Plane
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      //如果成功提取了平面特征线段,就从本次平面特征线段的最后一个点开始继续提取,
      //本次平面特征线段的最后一个点处在平面上,因此从最后一个点开始继续往后提取以保持连续性,
      //这样本次线段的最后一个点变为下次线段的第一个点,从而可以根据上面的逻辑判断进一步修改该点的标记
      i = i_nex - 1;
      last_state = 1;
    }
    //如果未成功提取平面特征线段,从失败的那个点的下一个点开始继续往后提取(因为失败的那个点不在平面上,没有必要再次尝试提取)
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    /*
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }
    */

    //last_i = i2;//没有用
    //last_i_nex = i_nex;//没有用
    last_direct = curr_direct;
  }
  //--------------------遍历每一个点,尝试提取平面特征--------------------//

  //--------------------遍历每一个点,尝试提取角点特征--------------------//
  //Question:这里i为什么要从head+3开始?下面的代码中,往前索引最多至i-2(edge_jump_judge()函数中),从head+2开始不就可以了吗?
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }
    //和后面一个点重影，舍弃
    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);//vec_a=(o->pl[i])
    Eigen::Vector3d vecs[2];

    //对i的相邻两个点i-1和i+1进行判断和标记
    for(int j=0; j<2; j++)
    {
      int m = -1;
      //m只能取±1,即i的相邻两个点
      //j=0,m=-1;j=1,m=1
      if(j == 1)
      {
        m = 1;
      }

      //types[i].edj[0]代表点i-1的标记,types[i].edj[1]代表点i+1的标记
      //如果i-1在盲区内,i在视野外,标记i-1为Nr_inf;
      //如果i-1在盲区内,i在视野内,标记i-1为Nr_blind;
      //如果i+1在盲区内,i在视野外,标记i+1为Nr_inf;
      //如果i+1在盲区内,i在视野内,标记i+1为Nr_blind;
      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z); //vecs[j]=(o->pl[i+m])
      vecs[j] = vecs[j] - vec_a; //vecs[j]=(pl[i]->pl[i+m])
      //求扫描线(o->i)和相邻两扫描点(i->i-1)/(i->i+1)连线的夹角的余弦值
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();//dot点乘

      //(o->i)和(i->i-1)/(i->i+1)之间的夹角接近180°,说明i-1/i+1距离扫描中心(o)更近,
      //这种情况说明i不可靠,很可能是存在被遮挡的情况产生的,需要被特殊标记,后续不能被提取为角点特征
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      //(o->i)和(i->i-1)/(i->i+1)之间的夹角接近0°,说明i距离扫描中心(o)更近
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    //vecs[Prev]即vecs[0],为向量(i->i-1);vecs[Next]即vecs[1],为向量(i->i+1)
    //即求当前点i分别和i-1,i+1连线的夹角
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    //所有types[i]的edj[0,1]都被初始化为Nr_nor,因此经过上面的步骤如果还是Nr_nor那就代表i和i-1/i+1之间距离较为接近

    //如果i-1和i之间接近,i和i+1相距较远且i距离扫描中心(o)更近
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      //如果(i->i-1)和(i->i+1)之间的夹角小于160°(越趋近90°越弯曲,越趋近180°越平坦)
      if(types[i].intersect > cos160)
      {
        //确保角点另一侧的点处在盲区外且较为平坦
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    //如果i-1和i相距较远且i距离扫描中心(o)更近,i和i+1相距较近
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }

    //如果i-1和i之间接近,i和i+1相距较远且i在视野外,i+1在盲区内
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }

    //如果i-1和i相距较远且i-1在盲区内i在视野外,i和i+1相距较近
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }

    //如果i-1和i+1都和i相距较远
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      //且如果i还未被标记
      if(types[i].ftype == Nor)
      {
        //就标记i为Wire,单独的角点特征
        types[i].ftype = Wire;
      }
    }
  }
  //--------------------遍历每一个点,尝试提取角点特征--------------------//

  //--------------------对于未标记的点,再次尝试提取平面特征--------------------//
  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    //如果i还未被标记
    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        //ratio为较大距离与较小距离比值的平方
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      //如果i-1,i,i+1之间较为平坦
      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        //如果i-1/i/i+1还未被标记,就将其标记为Real_Plane
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }
  //--------------------对于未标记的点,再次尝试提取平面特征--------------------//
  
  //--------------------降采样选取特征点--------------------//
  //平面特征点存入pl_surf中,角特征点存入pl_corn中
  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    //降采样提取平面点
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }

      //降采样
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;//强度值
        ap.curvature = pl[j].curvature;//相对(每线第一个点的)扫描时间
        pl_surf.push_back(ap);//存入pl_surf中

        last_surface = -1;
      }
    }
    else
    {
      //角点不做滤波
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }

      //如果last_surface是平面点,但在往后数到last_surface+point_filter_num-1的过程中,出现了非平面点,
      //那么程序就会运行至此,此时把从last_surface开始到目前为止这些连续的点取平均作为平面特征点存入pl_surf;
      //Question:这样不会降低精度吗?
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
  //--------------------降采样选取特征点--------------------//
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

//描述:从一线点云中提取起点为i_cur的平面特征线段;
//输入:1.pl:一线点云;
//          2.types:对应于pl存储每个点的深度和每个点距离下一个点的距离的平方;
//          3.i_cur:所提取的平面特征线段的起点;
//输出:1.i_nex:所提取的平面特征线段的终点的下一个点的索引;
//          2.curr_direct:所提取的平面特征线段的单位方向向量;
//          3.int型返回值:0:所提取的特征线段太过弯曲,不足以成为平面特征;
//                                      1:成功提取起点为i_cur的平面特征线段;
//                                      2:i_cur起的group_size个点中有处于lidar盲区阈值内的点,提取失败;                 
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  //group_dis的含义是一个设定的距离阈值(的平方),该阈值和距离成正相关,应该是为了补偿分辨率的下降
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  //--------------------扩展队列--------------------//
  //按如下规则沿着一线点云顺序从i_cur扩展一个队列,扩展至i_nex:
  //前group_size个点只要满足不在盲区范围内,就直接加入队列,
  //内在含义是所提取的点云队列最小长度为group_size
  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista); //dista中存储和下一个点之间的平方距离
  }
  
  //group_size个之后的点除了要满足不在盲区范围内,还要满足该点和队列第一个点i_cur之间的距离小于设定阈值
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;//应该是break？
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    //v=[vx; vy; vz]
    two_dis = vx*vx + vy*vy + vz*vz;
    //如果尾部的点和头部的点之间的距离大于设定的阈值,就不再扩展队列,即限定了队列的最大长度
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }
  //i_cur为队列起点,i_nex为队列终点的下一个点,disarr存储队列中每个点和下一个点之间的平方距离
  //--------------------扩展队列--------------------//

  //在队列中寻找一点j,满足队列头i_cur,队列尾i_nex-1和j构成的三角形面积最大,
  //即到向量(i_cur->i_nex-1)距离最远的点,即为点j,因为底一定,高越大,面积越大
  double leng_wid = 0;
  double v1[3], v2[3]; //v1=(i_cur->pl[j]),v2=v1xv
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    //v1为队列头i_cur指向pl[j]的向量
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    //[vx; vy; vz]为队列头i_cur指向队列尾i_nex-1的向量
    //v2为v1叉乘[vx; vy; vz]的结果
    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    //lw为v2的模的平方,||v2||=v1*v*sin(theta)
    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }

  //记v = [vx; vy; vz];为队列头i_cur指向队列尾i_nex-1的向量
  //v1为队列头i_cur指向pl[j]的向量
  //记v1和v之间的夹角为theta(\in{(0, M_PI/2)});
  //v2 = v1 x v;
  //leng_wid = ||v2||^2 = ||v1 x v||^2 = ||v1||^2 * ||v||^2 * sin(theta)^2;
  //two_dis是队列头和尾距离的平方, two_dis = ||v||^2;
  //从而:
  //          leng_wid * p2l_ratio(225) > two_dis * two_dis
  // <=> ||v1||^2 * ||v||^2 * sin(theta)^2 * p2l_ratio > ||v||^4
  // <=> ||v1|| * sin(theta) * sqrt(p2l_ratio) > ||v||
  // <=> ||v1|| * sin(theta) / ||v|| > 1 / sqrt(p2l_ratio)
  //||v1|| * sin(theta)代表队列中到v最远的距离,和v的模的比值表示了该队列线段的弯曲程度,用于近似曲率
  //曲率大于设定阈值,表示不是特别平,返回0
  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  //disarr存储队列中每个点和下一个点之间的平方距离
  //将disarr中的值从大到小降序排列
  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  //下面要做除数,需要确保不为0
  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    //最大间距的平方与最小间距的平方之间的比值,即限制间距,保证间隔是均匀变化的
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  //满足以上一切条件,所提取的线段队列就被选择为平面特征,
  //同时计算该特征所处线段(因为是在一线中提取的)的单位方向向量,存入curr_direct
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}


//确保角点另一侧的点处在盲区外且较为平坦
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  //确保相邻的点不在盲区内
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }

  //nor_dir = 0, d1 = ||pl[i-1] - pl[i]||^2, d2 = ||pl[i-2] - pl[i-1]||^2
  //nor_dir = 1, d1 = ||pl[i] - pl[i+1]||^2, d2 = ||pl[i+1] - pl[i+2]||^2
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);
  //经过上面的排序和开根号,d1为较大的那个距离,d2为较小的那个距离

  //如果d1和d2相距过大,代表i-2,i-1和i之间(或是i,i+1和i+2之间)的距离不均匀,说明不够平坦,因此i不能作为很好的角点特征
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}
