#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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

class pclFilterManager :public rclcpp::Node
{
public:
    pclFilterManager():Node("pcl_filter_manager")
    {
        // 声明参数
        declare_parameter("x_min",-0.3f);
        declare_parameter("x_max", 0.3f);
        declare_parameter("y_min",-0.3f);
        declare_parameter("y_max", 0.3f);
        declare_parameter("z_min", 0.0f);
        declare_parameter("z_max", 0.4f);
        declare_parameter("filter_z_min",-0.05f);
        declare_parameter("filter_z_max",  1.6f);

        // 承接参数
        x_min=get_parameter("x_min").get_value<float>();
        x_max=get_parameter("x_max").get_value<float>();
        y_min=get_parameter("y_min").get_value<float>();
        y_max=get_parameter("y_max").get_value<float>();
        z_min=get_parameter("z_min").get_value<float>();
        z_max=get_parameter("z_max").get_value<float>();
        filter_z_min=get_parameter("filter_z_min").get_value<float>();
        filter_z_max=get_parameter("filter_z_max").get_value<float>();

        // 初始化消息订阅/接收
        sub_=this->create_subscription<sensor_msgs::msg::PointCloud2>("pointCloud",10,
                    std::bind(&pclFilterManager::cloudCallback,this,std::placeholders::_1)); // 话题名称可调
        pub_=this->create_publisher<sensor_msgs::msg::PointCloud2>("output_filter",10);
    }
private:

    // 订阅回调函数
    void cloudCallback(const sensor_msgs::msg::PointCloud2 &cloud_msg)
    {
        // 转换 ROS 点云消息为 PCL 格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *cloud_in);

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
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered_blob_final, output);
        output.header = cloud_msg.header; // 保留原始头信息

        // 发布过滤后的点云
        pub_->publish(output);
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

    // 点云角度过滤（未应用）
    void filterByAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,float min_angle, float max_angle)
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

    // 计算点与参考方向的夹角（弧度）
    float calculateAngle(const pcl::PointXYZ &point, const Eigen::Vector3f &reference_direction)
    {
        float norm = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        Eigen::Vector3f point_vector(point.x, point.y, point.z);
        return std::acos(point_vector.dot(reference_direction) / (norm * reference_direction.norm()));
    }

    // 参数
    float x_min, x_max, y_min, y_max, z_min, z_max;
    float filter_z_min, filter_z_max;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<pclFilterManager>());
    rclcpp::shutdown();
    return 0;
}