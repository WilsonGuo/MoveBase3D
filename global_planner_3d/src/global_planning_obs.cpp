#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "backward.hpp"
#include "execution_classes.hpp"
#include "pcl/filters/passthrough.h"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "pcl/filters/voxel_grid.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std;
using namespace Eigen;
using namespace EXECUTION;

namespace backward
{
  backward::SignalHandling sh;
}

class GlobalPlanningObs : public rclcpp::Node
{
public:
    GlobalPlanningObs():Node("global_planning_obs")
    {
        // 参数声明
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("lidar_frame_id", "front_mid360");
        this->declare_parameter("base_frame_id", "base_footprint");
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("leaf_size", 0.2);
        this->declare_parameter("local_x_l", -2.0);
        this->declare_parameter("local_x_u", 2.0);
        this->declare_parameter("local_y_l", -2.0);
        this->declare_parameter("local_y_u", 2.0);
        this->declare_parameter("local_z_l", -0.3);
        this->declare_parameter("local_z_u", 0.5);
        this->declare_parameter("expansionCoefficient", 1.0);

        // 初始化TF
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // 初始化发布订阅
        pt_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointCloud", 10, std::bind(&GlobalPlanningObs::rcvLidarCallBack, this, std::placeholders::_1));
        world_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "global_map", 10, std::bind(&GlobalPlanningObs::rcvWorldCallBack, this, std::placeholders::_1));

        obs_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obs_vis", 10);
        obs_cost_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("obs_cost", 10);
        obs_array_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/obs_raw", 10);

        // 获取参数
        map_frame_id = this->get_parameter("map_frame_id").as_string();
        lidar_frame_id = this->get_parameter("lidar_frame_id").as_string();
        base_frame_id = this->get_parameter("base_frame_id").as_string();
        resolution = this->get_parameter("resolution").as_double();
        leaf_size = this->get_parameter("leaf_size").as_double();
        local_x_l = this->get_parameter("local_x_l").as_double();
        local_x_u = this->get_parameter("local_x_u").as_double();
        local_y_l = this->get_parameter("local_y_l").as_double();
        local_y_u = this->get_parameter("local_y_u").as_double();
        local_z_l = this->get_parameter("local_z_l").as_double();
        local_z_u = this->get_parameter("local_z_u").as_double();
        expansionCoefficient = this->get_parameter("expansionCoefficient").as_double();
        
        expansion = resolution * expansionCoefficient;
        world = new World(resolution);
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr world_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_cost_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_array_pub_;

    // 参数
    World *world = nullptr;
    double resolution, leaf_size, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;
    string map_frame_id;
    string lidar_frame_id;
    string base_frame_id;

    double expansionCoefficient = 1;
    double expansion = 1;

    // tf变换
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // 点云
    sensor_msgs::msg::PointCloud2 worldPoints;
    pcl::PointCloud<pcl::PointXYZ> worldCloud;

    void rcvLidarCallBack(const sensor_msgs::msg::PointCloud2 &lidar_points)
    {
        RCLCPP_INFO(this->get_logger(),"rcvLidarCallBack");
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

        // listener_ptr->waitForTransform(map_frame_id, base_frame_id, ros::Time(0), ros::Duration(2.0));
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer->lookupTransform(
            map_frame_id, lidar_frame_id, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran_cost(new pcl::PointCloud<pcl::PointXYZ>);

        std_msgs::msg::Float32MultiArray obs_array;
        for (const auto &pt : cloud_filt->points)
        {
            geometry_msgs::msg::PointStamped origin_point;
            origin_point.header.frame_id = lidar_frame_id;
            origin_point.header.stamp = this->now();
            origin_point.point.x = pt.x;
            origin_point.point.y = pt.y;
            origin_point.point.z = pt.z;

            geometry_msgs::msg::PointStamped trans_point;
            tf2::doTransform(origin_point, trans_point, transform);

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

        // 发布点云数据
        sensor_msgs::msg::PointCloud2 obs_vis;
        pcl::toROSMsg(*cloud_tran, obs_vis);
        obs_vis.header.frame_id = map_frame_id;
        obs_pub_->publish(obs_vis);

        // 发布障碍物数组
        obs_array_pub_->publish(obs_array);

        // 发布代价点云
        sensor_msgs::msg::PointCloud2 obs_cost;
        pcl::toROSMsg(*cloud_tran_cost, obs_cost);
        obs_cost.header.frame_id = map_frame_id;
        obs_cost_pub_->publish(obs_cost);
    }

    void rcvWorldCallBack(const sensor_msgs::msg::PointCloud2 &pointcloud_map)
    {
        RCLCPP_INFO(this->get_logger(),"rcvWorldCallBack");
        pcl::fromROSMsg(pointcloud_map, worldCloud);
    }
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<GlobalPlanningObs>());
    rclcpp::shutdown();
    return 0;
}