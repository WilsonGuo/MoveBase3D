#include <ros/ros.h>
// PCL specific includes 用PCL的时候需要用到的库
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common.h>
#include <cmath>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h> //半径滤波器头文件
// 先声明发布器，因为回调函数里面会用到
ros::Publisher pub;
float x_min, x_max, y_min, y_max, z_min, z_max;
float filter_z_min, filter_z_max;
// 计算点与参考方向的夹角（弧度）
float calculateAngle(const pcl::PointXYZ &point, const Eigen::Vector3f &reference_direction)
{
    float norm = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    Eigen::Vector3f point_vector(point.x, point.y, point.z);
    return std::acos(point_vector.dot(reference_direction) / (norm * reference_direction.norm()));
}

// 点云角度过滤
void filterByAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                   float min_angle, float max_angle)
{
    Eigen::Vector3f reference_direction(1, 0, 0); // Z轴方向

    // 过滤点云
    pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());
    for (size_t i = 0; i < cloud_in->size(); ++i)
    {
        float angle = calculateAngle(cloud_in->points[i], reference_direction);
        if (angle < min_angle || angle > max_angle)
        {
            indices_to_remove->indices.push_back(i);
        }
    }

    // 提取保留的点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices_to_remove);
    extract.setNegative(true); // 反选，保留未移除的点
    extract.filter(*cloud_out);
}

// 点云区域过滤函数
void filterByRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                    float x_min, float x_max,
                    float y_min, float y_max,
                    float z_min, float z_max)
{
    pcl::PointIndices::Ptr indices_to_keep(new pcl::PointIndices());

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        const pcl::PointXYZ &point = cloud_in->points[i];
        if (point.x >= x_min && point.x <= x_max &&
            point.y >= y_min && point.y <= y_max &&
            point.z >= z_min && point.z <= z_max)
        {
            indices_to_keep->indices.push_back(i); // 保留符合范围的点
        }
    }

    // 提取保留点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(indices_to_keep);
    extract.setNegative(true); // 保留匹配点
    extract.filter(*cloud_out);
}
// const 可以保护input的内容
// sensor_msgs::PointCloud2ConstPtr用来创建常量点云的指针
// sensor_msgs::PointCloud2 用来创建普通的点云容器
// 点云的数据类型还有很多，但是常用这个，参考第4步
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{

    // 转换 ROS 点云消息为 PCL 格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud_in);

    // 输出点云容器
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());

    // 调用区域过滤函数
    filterByRegion(cloud_in, cloud_out, x_min, x_max, y_min, y_max, z_min, z_max);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_z(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_volex(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_blob_final(new pcl::PointCloud<pcl::PointXYZ>());

    // 定义 PassThrough 滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_out);
    pass.setFilterFieldName("z");                     // 过滤 Z 轴
    pass.setFilterLimits(filter_z_min, filter_z_max); // 高度范围：0.5 到 1.5 米
    pass.filter(*cloud_out_z);

    //   // 创建体素栅格下采样: 下采样的大小为1cm
    pcl::VoxelGrid<pcl::PointXYZ> sor_voxel; // 体素栅格下采样对象
    sor_voxel.setInputCloud(cloud_out_z);    // 原始点云
    sor_voxel.setLeafSize(0.1f, 0.1f, 0.1f); // 设置采样体素大小
    sor_voxel.filter(*cloud_out_volex);      // 保存

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_out_volex);
    outrem.setRadiusSearch(1.0);
    outrem.setMinNeighborsInRadius(20); // apply
    outrem.filter(*cloud_filtered_blob_final);

    // 转换过滤后的点云为 ROS 格式
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered_blob_final, output);
    output.header = cloud_msg->header; // 保留原始头信息

    // 发布过滤后的点云
    pub.publish(output);

}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_filter_manager");
    ros::NodeHandle nh;
    nh.param("x_min", x_min, -0.3f);
    nh.param("x_max", x_max, 0.3f);
    nh.param("y_min", y_min, -0.3f);
    nh.param("y_max", y_max, 0.3f);
    nh.param("z_min", z_min, 0.0f);
    nh.param("z_max", z_max, 0.4f);
    nh.param("filter_z_min", filter_z_min, -0.05f);
    nh.param("filter_z_max", filter_z_max, 1.6f);
    // Create a ROS subscriber for the input point cloud
    // 创建订阅器的时候是和PCL没有关系的，只和ROS有关系，回调函数才需要涉及PCL
    // 这里的Input可以改成自己想订阅的topic,比如"/rslidar_points"
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    // 创建发布器的时候和PCL有关系，因为要指定发布出去的信息格式
    pub = nh.advertise<sensor_msgs::PointCloud2>("output_filter", 1);

    // Spin
    ros::spin(); // ros::spin()用于调用所有可触发的回调函数,将进入循环,不会返回,类似于在循环里反复调用spinOnce()
}
