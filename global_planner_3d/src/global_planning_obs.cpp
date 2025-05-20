#include <tf/transform_listener.h>
#include "backward.hpp"
#include "execution_classes.h"
#include <pcl/filters/passthrough.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/filters/voxel_grid.h> // 包含VoxelGrid滤波器的头文件

using namespace std;
using namespace Eigen;
using namespace EXECUTION;

namespace backward
{
  backward::SignalHandling sh;
}
World *world = NULL;
// ros related
ros::Subscriber pt_sub;
ros::Subscriber world_sub;
ros::Publisher obs_pub;
ros::Publisher obs_cost_pub;
ros::Publisher obs_array_pub;

// 参数
double resolution, leaf_size, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;
string map_frame_id;
string lidar_frame_id;
string base_frame_id;

double expansionCoefficient = 1;
double expansion = 1;

tf::TransformListener *listener_ptr;
sensor_msgs::PointCloud2 worldPoints;
pcl::PointCloud<pcl::PointXYZ> worldCloud;

void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points);

void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(lidar_points, *cloud);

  // 设置 VoxelGrid 滤波器
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_VoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*cloud_after_VoxelGrid);

  // 过滤到指定范围内的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(cloud_after_VoxelGrid);
  passthrough.setFilterFieldName("x");
  passthrough.setFilterLimits(local_x_l, local_x_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("y");
  passthrough.setFilterLimits(local_y_l, local_y_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(local_z_l, local_z_u);
  passthrough.filter(*cloud_after_PassThrough);

  // 新建一个新的局部World 即:3D Map
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);

  Vector3d lowerbound(local_x_l, local_y_l, local_z_l);
  Vector3d upperbound(local_x_u, local_y_u, local_z_u);

  World local_world = World(resolution);
  local_world.initGridMap(lowerbound, upperbound);

  // 填充局部World 即:3D Map
  for (const auto &pt : (*cloud_after_PassThrough).points)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    if (local_world.isFree(obstacle))
    {
      local_world.setObs(obstacle);
      Vector3d obstacle_round = local_world.coordRounding(obstacle);
      pcl::PointXYZ pt_add;
      pt_add.x = obstacle_round(0);
      pt_add.y = obstacle_round(1);
      pt_add.z = obstacle_round(2);
      cloud_filt->points.push_back(pt_add);
    }
  }

  listener_ptr->waitForTransform(map_frame_id, base_frame_id, ros::Time(0), ros::Duration(2.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran_cost(new pcl::PointCloud<pcl::PointXYZ>);

  std_msgs::Float32MultiArray obs_array;
  for (const auto &pt : cloud_filt->points)
  {
    geometry_msgs::PointStamped origin_point;
    origin_point.header.frame_id = lidar_frame_id;
    origin_point.header.stamp = ros::Time();
    origin_point.point.x = pt.x;
    origin_point.point.y = pt.y;
    origin_point.point.z = pt.z;

    geometry_msgs::PointStamped trans_point;

    listener_ptr->transformPoint(map_frame_id, origin_point, trans_point);

    pcl::PointXYZ _pt;

    _pt.x = trans_point.point.x;
    _pt.y = trans_point.point.y;
    _pt.z = trans_point.point.z;

    //********************************************************
    for (const auto &pt : worldCloud)
    {
      // 判断是否包含在一个体素内
      if (((_pt.x > pt.x - resolution / 2 && _pt.x < pt.x + resolution / 2) || _pt.x == pt.x - resolution / 2 || _pt.x == pt.x + resolution / 2) && ((_pt.y > pt.y - resolution / 2 && _pt.y < pt.y + resolution / 2) || _pt.y == pt.y - resolution / 2 || _pt.y == pt.y + resolution / 2))
      {
        if ((_pt.z - pt.z) == resolution * 3 || (_pt.z - pt.z) > resolution * 3)
        {
          cloud_tran_cost->points.push_back(_pt);

          cloud_tran->points.push_back(_pt);

          pcl::PointXYZ _pt_1;
          _pt_1.x = _pt.x + expansion;
          _pt_1.y = _pt.y;
          _pt_1.z = _pt.z;
          cloud_tran->points.push_back(_pt_1);

          pcl::PointXYZ _pt_2;
          _pt_2.x = _pt.x - expansion;
          _pt_2.y = _pt.y;
          _pt_2.z = _pt.z;
          cloud_tran->points.push_back(_pt_2);

          pcl::PointXYZ _pt_3;
          _pt_3.x = _pt.x;
          _pt_3.y = _pt.y + expansion;
          _pt_3.z = _pt.z;
          cloud_tran->points.push_back(_pt_3);

          pcl::PointXYZ _pt_4;
          _pt_4.x = _pt.x;
          _pt_4.y = _pt.y - expansion;
          _pt_4.z = _pt.z;
          cloud_tran->points.push_back(_pt_4);

          pcl::PointXYZ _pt_5;
          _pt_5.x = _pt.x + expansion;
          _pt_5.y = _pt.y + expansion;
          _pt_5.z = _pt.z;
          cloud_tran->points.push_back(_pt_5);

          pcl::PointXYZ _pt_6;
          _pt_6.x = _pt.x - expansion;
          _pt_6.y = _pt.y - expansion;
          _pt_6.z = _pt.z;
          cloud_tran->points.push_back(_pt_6);

          pcl::PointXYZ _pt_7;
          _pt_7.x = _pt.x + expansion;
          _pt_7.y = _pt.y - expansion;
          _pt_7.z = _pt.z;
          cloud_tran->points.push_back(_pt_7);

          pcl::PointXYZ _pt_8;
          _pt_8.x = _pt.x - expansion;
          _pt_8.y = _pt.y + expansion;
          _pt_8.z = _pt.z;
          cloud_tran->points.push_back(_pt_8);

          // 传递数据
          obs_array.data.push_back(_pt.x);
          obs_array.data.push_back(_pt.y);
          obs_array.data.push_back(_pt.z);

          obs_array.data.push_back(_pt_1.x);
          obs_array.data.push_back(_pt_1.y);
          obs_array.data.push_back(_pt_1.z);

          obs_array.data.push_back(_pt_2.x);
          obs_array.data.push_back(_pt_2.y);
          obs_array.data.push_back(_pt_2.z);

          obs_array.data.push_back(_pt_3.x);
          obs_array.data.push_back(_pt_3.y);
          obs_array.data.push_back(_pt_3.z);

          obs_array.data.push_back(_pt_4.x);
          obs_array.data.push_back(_pt_4.y);
          obs_array.data.push_back(_pt_4.z);

          obs_array.data.push_back(_pt_5.x);
          obs_array.data.push_back(_pt_5.y);
          obs_array.data.push_back(_pt_5.z);

          obs_array.data.push_back(_pt_6.x);
          obs_array.data.push_back(_pt_6.y);
          obs_array.data.push_back(_pt_6.z);

          obs_array.data.push_back(_pt_7.x);
          obs_array.data.push_back(_pt_7.y);
          obs_array.data.push_back(_pt_7.z);

          obs_array.data.push_back(_pt_8.x);
          obs_array.data.push_back(_pt_8.y);
          obs_array.data.push_back(_pt_8.z);
        }
      }
    }

  }

  sensor_msgs::PointCloud2 obs_vis;
  pcl::toROSMsg(*cloud_tran, obs_vis);
  obs_vis.header.frame_id = map_frame_id;
  obs_pub.publish(obs_vis);

  obs_array_pub.publish(obs_array);

  sensor_msgs::PointCloud2 obs_cost;
  pcl::toROSMsg(*cloud_tran_cost, obs_cost);
  obs_cost.header.frame_id = map_frame_id;
  obs_cost_pub.publish(obs_cost);
}

void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

/**
 *@brief receive point cloud to build the grid map
 */
void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  pcl::fromROSMsg(pointcloud_map, worldCloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planning_obs");
  ros::NodeHandle nh("~");

  pt_sub = nh.subscribe("pointCloud", 1, rcvLidarCallBack);
  world_sub = nh.subscribe("map", 1, rcvWorldCallBack);

  obs_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_vis", 1);
  obs_cost_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_cost", 1);
  obs_array_pub = nh.advertise<std_msgs::Float32MultiArray>("/obs_raw", 1);

  nh.getParam("map/map_frame_id", map_frame_id);
  nh.getParam("map/lidar_frame_id", lidar_frame_id);
  nh.getParam("map/base_frame_id", base_frame_id);
  nh.param("map/resolution", resolution, 0.1);

  nh.param("map/expansionCoefficient", expansionCoefficient, 1.0);
  // 降采样过滤
  nh.param("map/leaf_size", leaf_size, 0.2);
  // 边界过滤
  nh.param("map/local_x_l", local_x_l, -2.0);
  nh.param("map/local_x_u", local_x_u, 2.0);
  nh.param("map/local_y_l", local_y_l, -2.0);
  nh.param("map/local_y_u", local_y_u, 2.0);
  nh.param("map/local_z_l", local_z_l, -0.3);
  nh.param("map/local_z_u", local_z_u, 0.5);

  expansion = resolution * expansionCoefficient;

  tf::TransformListener listener;
  listener_ptr = &listener;
  world = new World(resolution);

  while (ros::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);
    ros::spinOnce();
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 50);
  }
  return 0;
}
