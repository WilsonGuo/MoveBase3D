#include "backward.hpp"
#include "execution_planner.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <rover_msgs/roverGoalStatus.h>

using namespace std;
using namespace std_msgs;
using namespace Eigen;
using namespace EXECUTION;
using namespace EXECUTION::visualization;
using namespace EXECUTION::planner;

namespace backward
{
  backward::SignalHandling sh;
}

uint PENDING = 0;   // 任务处于等待状态，尚未开始
uint ACTIVE = 1;    // 任务正在进行中
uint PREEMPTED = 2; // 任务被抢占，执行中止
uint SUCCEEDED = 3; // 任务成功完成
uint ABORTED = 4;   // 任务由于某种原因被中止
uint REJECTED = 5;  // 任务被拒绝，未开始执行
uint PREEMPTING = 6;
uint RECALLING = 7;
uint RECALLED = 8;
uint LOST = 9;

// ros related
ros::Subscriber map_sub, goal_sub;
ros::Subscriber obs_cost_sub;

ros::Publisher grid_map_vis_pub;
ros::Publisher path_vis_pub;
ros::Publisher goal_vis_pub;
ros::Publisher surf_vis_pub;
ros::Publisher tree_vis_pub;
ros::Publisher path_interpolation_pub;
ros::Publisher tree_tra_pub;
ros::Publisher goal_status_pub;

ros::Publisher obs_pub;

// indicate whether the robot has a moving goal
bool has_goal = false;
// simulation param from launch file
double resolution;
double expansion;
double expansionCoefficient;
double goal_thre;
double step_size;
double h_surf_car;
double max_initial_time;
double radius_fit_plane;
FitPlaneArg fit_plane_arg;
double neighbor_radius;

// useful global variables
Vector3d start_pt;
Vector3d target_pt;
World *world = NULL;
PFRRTStar *pf_rrt_star = NULL;

rover_msgs::roverGoalStatus curGoalStatus;

void rcvGoalCallBack(const geometry_msgs::PoseStamped &goalPose);
// function declaration
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);
void pubInterpolatedPath(const vector<Node *> &solution, ros::Publisher *_path_interpolation_pub);
void findSolution();
void callPlanner();

// 函数：计算二维平面中两点之间的距离
double calculate2DDistance(Vector3d start_pt, Vector3d target_pt)
{
  return std::sqrt((target_pt.x() - start_pt.x()) * (target_pt.x() - start_pt.x()) + (target_pt.y() - start_pt.y()) * (target_pt.y() - start_pt.y()));
}

void rcvGoalCallBack(const geometry_msgs::PoseStamped &goalPose)
{
  ROS_ERROR("****************");
  ROS_ERROR("x=%f", goalPose.pose.position.x);
  ROS_ERROR("y=%f", goalPose.pose.position.y);
  ROS_ERROR("z=%f", goalPose.pose.position.z);
  ROS_ERROR("O_x=%f", goalPose.pose.orientation.x);
  ROS_ERROR("O_y=%f", goalPose.pose.orientation.y);
  ROS_ERROR("O_z=%f", goalPose.pose.orientation.z);
  ROS_ERROR("O_w=%f", goalPose.pose.orientation.w);
  ROS_ERROR("*****************");

  if (!world->has_map_)
    return;
  has_goal = true;
  target_pt = Vector3d(goalPose.pose.position.x, goalPose.pose.position.y, goalPose.pose.position.z);

  curGoalStatus.x = goalPose.pose.position.x;
  curGoalStatus.y = goalPose.pose.position.y;
  curGoalStatus.z = goalPose.pose.position.z;

  curGoalStatus.orientation_x = goalPose.pose.orientation.x;
  curGoalStatus.orientation_y = goalPose.pose.orientation.y;
  curGoalStatus.orientation_z = goalPose.pose.orientation.z;
  curGoalStatus.orientation_w = goalPose.pose.orientation.w;

  ROS_INFO("Receive the planning target");
}

pcl::PointCloud<pcl::PointXYZ> worldCloud;
/**
 *@brief receive point cloud to build the grid map
 */
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(pointcloud_map, cloud);
  worldCloud = cloud;
  world->initGridMap(cloud);

  for (const auto &pt : cloud)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    world->setObs(obstacle);

  }
  visWorld(world, &grid_map_vis_pub);
}

pcl::PointCloud<pcl::PointXYZ> projected_cloud;

void projectToWorldCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                         pcl::PointCloud<pcl::PointXYZ> &projected_cloud,
                         const pcl::PointCloud<pcl::PointXYZ> &worldCloud,
                         double resolution)
{
  // 清空投影点云
  projected_cloud.clear();

  // 遍历 cloud 中的每个点
  for (const auto &_pt : cloud.points)
  {
    // 将点投影到网格中心（世界地图坐标）
    double x_proj = std::round(_pt.x / resolution) * resolution;
    double y_proj = std::round(_pt.y / resolution) * resolution;
    double z_proj = std::round(_pt.z / resolution) * resolution;

    // 检查投影点是否存在于 worldCloud
    for (const auto &world_pt : worldCloud.points)
    {
      if ((std::fabs(x_proj - world_pt.x) < resolution / 2 || std::fabs(x_proj - world_pt.x) == resolution / 2) &&
          (std::fabs(y_proj - world_pt.y) < resolution / 2 || std::fabs(y_proj - world_pt.y) == resolution / 2) &&
          (std::fabs((z_proj - resolution * 3) - world_pt.z) < resolution / 2 || std::fabs((z_proj - resolution * 3) - world_pt.z) == resolution / 2)) // resolution * 3  解释：由于在局部地图发布障碍物的时候，做了高度为（resolution * 3）的过滤，这里需要恢复，否则无法与全局Gridmap相匹配
      {
        pcl::PointXYZ projected_pt(x_proj, y_proj, world_pt.z);
        projected_cloud.push_back(projected_pt);
        // ROS_INFO("( %f, %f, %f )", projected_pt.x, projected_pt.y, projected_pt.z);
        break; // 一个点只投影到最近的体素一次
      }
    }
  }
}

/**
 * 处理障碍物的回调
 */
void rcvObsCostCallBack(const sensor_msgs::PointCloud2 &obsCost)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(obsCost, cloud);
  if (!world->has_map_)
    return;

  // 将 cloud 投影到 worldCloud 的最近网格中心
  projectToWorldCloud(cloud, projected_cloud, worldCloud, resolution);

  sensor_msgs::PointCloud2 obs_project;
  pcl::toROSMsg(projected_cloud, obs_project);
  obs_project.header.frame_id = "map";
  obs_pub.publish(obs_project);

  if (projected_cloud.points.size() > 0)
  {
    for (const auto &pt : worldCloud)
    {
      Vector3d obstacle(pt.x, pt.y, pt.z);
      world->setObs(obstacle); // 默认都为Grid
      for (const auto &_pt : projected_cloud.points)
      {
        // 判断该点是否包含在膨胀后所在的体素集合内
        if (((_pt.x > pt.x - expansion && _pt.x < pt.x + expansion) || (_pt.x == pt.x - expansion) || (_pt.x == pt.x + expansion)) && ((_pt.y > pt.y - expansion && _pt.y < pt.y + expansion) || (_pt.y == pt.y - expansion) || (_pt.y == pt.y + expansion)))
        {
          world->setObsReverse(obstacle); // 将膨胀后的体素集合全部反转，激发规划器避障
        }
      }
    }
  }
  else
  {
    for (const auto &pt : worldCloud)
    {
      Vector3d obstacle(pt.x, pt.y, pt.z);
      world->setObs(obstacle);
    }
  }

  visWorld(world, &grid_map_vis_pub);
}

/**
 *@brief Linearly interpolate the generated path to meet the needs of local planning
 */
void pubInterpolatedPath(const vector<Node *> &solution, ros::Publisher *path_interpolation_pub)
{
  if (path_interpolation_pub == NULL)
    return;
  Float32MultiArray msg;
  for (size_t i = 0; i < solution.size(); i++)
  {
    if (i == solution.size() - 1)
    {
      msg.data.push_back(solution[i]->position_(0));
      msg.data.push_back(solution[i]->position_(1));
      msg.data.push_back(solution[i]->position_(2));
    }
    else
    {
      size_t interpolation_num = (size_t)(EuclideanDistance(solution[i + 1], solution[i]) / 0.1);
      Vector3d diff_pt = solution[i + 1]->position_ - solution[i]->position_;
      for (size_t j = 0; j < interpolation_num; j++)
      {
        Vector3d interpt = solution[i]->position_ + diff_pt * (float)j / interpolation_num;
        msg.data.push_back(interpt(0));
        msg.data.push_back(interpt(1));
        msg.data.push_back(interpt(2));
      }
    }
  }
  path_interpolation_pub->publish(msg);
}

/**
 *@brief 在指定起点和终点的前提下，调用PF-RRT*算法进行规划。
 * 根据原点和目标点的投影结果，可分为三种情况。
 */
void findSolution()
{
  Path solution = Path();
  pf_rrt_star->initWithGoal(start_pt, target_pt);

  // 案例1：当原点无法投影到地面时，PF-RRT*无法工作
  if (pf_rrt_star->state() == Invalid)
  {
    ROS_WARN("The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
  }

  // 案例2: 如果原点和目标都可以投影，则 PF-RRT* 将执行
  // 全局规划并尝试生成路径
  else if (pf_rrt_star->state() == Global)
  {
    ROS_INFO("Starting PF-RRT* algorithm at the state of global planning");
    int max_iter = 5000;
    double max_time = 100.0;

    while (solution.type_ == Path::Empty && max_time < max_initial_time)
    {
      solution = pf_rrt_star->planner(max_iter, max_time);
      max_time += 100.0;
    }

    if (solution.nodes_.empty())
    {
      ROS_WARN("No solution found!");

      curGoalStatus.status = REJECTED;
    }
  }
  // 案例3: 如果原点可以投影而目标不能投影，则PF-RRT*
  // 将尝试找到一个临时的转换目标。
  else
  {
    // ROS_INFO("Starting PF-RRT* algorithm at the state of rolling planning");
    int max_iter = 1500;
    double max_time = 100.0;

    solution = pf_rrt_star->planner(max_iter, max_time);

    if (solution.nodes_.empty())
    {
      ROS_WARN("No solution found!");

      curGoalStatus.status = REJECTED;
    }
  }

  pubInterpolatedPath(solution.nodes_, &path_interpolation_pub);
  visPath(solution.nodes_, &path_vis_pub);
  visSurf(solution.nodes_, &surf_vis_pub);

  // 当生成的路径足够短时，就说明已经到达目的地了
  if (solution.type_ == Path::Global && EuclideanDistance(pf_rrt_star->origin(), pf_rrt_star->target()) < goal_thre)
  {
    has_goal = false;
    visOriginAndGoal({}, &goal_vis_pub); // 重新初始化
    visPath({}, &path_vis_pub);
    ROS_INFO("The Robot has achieved the goal!!!");
  }

  if (solution.type_ == Path::Empty)
    visPath({}, &path_vis_pub);
}

/**
 *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
 *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
 */
void callPlanner()
{
  static double init_time_cost = 0.0;
  if (!world->has_map_)
    return;

  // The tree will expand at a certain frequency to explore the space more fully
  if (!has_goal && init_time_cost < 1000)
  {
    timeval start;
    gettimeofday(&start, NULL);
    pf_rrt_star->initWithoutGoal(start_pt);
    timeval end;
    gettimeofday(&end, NULL);
    init_time_cost = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    if (pf_rrt_star->state() == WithoutGoal)
    {
      int max_iter = 550;
      double max_time = 100.0;
      pf_rrt_star->planner(max_iter, max_time);
      // ROS_INFO("Current size of tree: %d", (int)(pf_rrt_star->tree().size()));
    }
    else
      ROS_WARN("Attention: the start point can't be projected");
  }
  // If there is a specified moving target,call PF-RRT* to find a solution
  else if (has_goal)
  {
    findSolution();
    init_time_cost = 0.0;
  }
  // The expansion of tree will stop after the process of initialization takes more than 1s
  else
    ROS_INFO("The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planning_node");
  ros::NodeHandle nh("~");

  map_sub = nh.subscribe("map", 1, rcvPointCloudCallBack);
  goal_sub = nh.subscribe("/cur_goal", 1, rcvGoalCallBack);
  obs_cost_sub = nh.subscribe("/global_planning_obs_node/obs_cost", 100, rcvObsCostCallBack);

  grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
  path_vis_pub = nh.advertise<visualization_msgs::Marker>("path_vis", 20);
  goal_vis_pub = nh.advertise<visualization_msgs::Marker>("goal_vis", 1);
  surf_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("surf_vis", 100);
  tree_vis_pub = nh.advertise<visualization_msgs::Marker>("tree_vis", 1);
  tree_tra_pub = nh.advertise<std_msgs::Float32MultiArray>("tree_tra", 1);
  goal_status_pub = nh.advertise<rover_msgs::roverGoalStatus>("/cur_global_goal_status", 1);
  path_interpolation_pub = nh.advertise<std_msgs::Float32MultiArray>("global_path", 1000);

  obs_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_vis", 1);

  nh.param("map/resolution", resolution, 0.1);
  nh.param("map/expansionCoefficient", expansionCoefficient, 1.0);
  expansion = resolution * expansionCoefficient;

  nh.param("planning/goal_thre", goal_thre, 1.0);
  nh.param("planning/step_size", step_size, 0.2);
  nh.param("planning/h_surf_car", h_surf_car, 0.4);
  nh.param("planning/neighbor_radius", neighbor_radius, 1.0);

  nh.param("planning/w_fit_plane", fit_plane_arg.w_total_, 0.4);
  nh.param("planning/w_flatness", fit_plane_arg.w_flatness_, 4000.0);
  nh.param("planning/w_slope", fit_plane_arg.w_slope_, 0.4);
  nh.param("planning/w_sparsity", fit_plane_arg.w_sparsity_, 0.4);
  nh.param("planning/ratio_min", fit_plane_arg.ratio_min_, 0.25);
  nh.param("planning/ratio_max", fit_plane_arg.ratio_max_, 0.4);
  nh.param("planning/conv_thre", fit_plane_arg.conv_thre_, 0.1152);
  nh.param("planning/radius_fit_plane", radius_fit_plane, 1.0);
  nh.param("planning/max_initial_time", max_initial_time, 1000.0);

  // Initialization
  world = new World(resolution);
  pf_rrt_star = new PFRRTStar(h_surf_car, world);

  // Set argument of PF-RRT*
  pf_rrt_star->setGoalThre(goal_thre);
  pf_rrt_star->setStepSize(step_size);
  pf_rrt_star->setFitPlaneArg(fit_plane_arg);
  pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
  pf_rrt_star->setNeighborRadius(neighbor_radius);

  pf_rrt_star->goal_vis_pub_ = &goal_vis_pub;
  pf_rrt_star->tree_vis_pub_ = &tree_vis_pub;
  pf_rrt_star->tree_tra_pub_ = &tree_tra_pub;

  tf::TransformListener listener;

  while (ros::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);

    // Update the position of the origin
    tf::StampedTransform transform;
    while (true && ros::ok())
    {
      if (listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0)))
      {
        try
        {
          listener.lookupTransform("/map", "/base_link", ros::Time(0), transform); // 查询变换
          break;
        }
        catch (tf::TransformException &ex)
        {
          continue;
        }
      }
      else
      {
        ROS_ERROR("Transform not available within timeout.");
      }
    }
    start_pt << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();

    if (has_goal)
    {
      double remain_distance = calculate2DDistance(start_pt, target_pt);
      ROS_INFO("remain_distance=%lf", remain_distance);
      if (remain_distance < goal_thre || remain_distance == goal_thre)
      {
        curGoalStatus.status = SUCCEEDED;
      }
      else
      {
        curGoalStatus.status = ACTIVE;
      }
      goal_status_pub.publish(curGoalStatus);
    }

    // Execute the callback functions to update the grid map and check if there's a new goal
    ros::spinOnce();
    // Call the PF-RRT* to work
    callPlanner();
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 100); // Cycle in 100ms
  }
  return 0;
}
