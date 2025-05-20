#include "backward.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rover_msgs/msg/rover_goal_status.hpp>
#include <execution_planner.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>

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

class GlobalPlanningNode : public rclcpp::Node
{
public:
    GlobalPlanningNode() : rclcpp::Node("global_planning_node")
    {
        // 创建回调组
        timer_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        subscription_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        planning_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        declare_parameter("map_frame_id", "map");
        declare_parameter("base_frame_id", "base_link");
        declare_parameter("map/resolution", 0.1);
        declare_parameter("map/expansionCoefficient", 1.0);
        declare_parameter("planning/goal_thre",1.0);
        declare_parameter("planning/step_size",0.2);
        declare_parameter("planning/h_surf_car",0.4);
        declare_parameter("planning/neighbor_radius",1.0);
        declare_parameter("planning/w_fit_plane",0.4);
        declare_parameter("planning/w_flatness",4000.0);
        declare_parameter("planning/w_slope",0.4);
        declare_parameter("planning/w_sparsity",0.4);
        declare_parameter("planning/ratio_min",0.25);
        declare_parameter("planning/ratio_max",0.4);
        declare_parameter("planning/conv_thre",0.1152);
        declare_parameter("planning/radius_fit_plane",1.0);
        declare_parameter("planning/max_initial_time",1000.0);

        map_frame_id = this->get_parameter("map_frame_id").as_string();
        base_frame_id = this->get_parameter("base_frame_id").as_string();
        get_parameter("map/resolution", resolution);
        get_parameter("map/expansionCoefficient", expansionCoefficient);
        get_parameter("planning/goal_thre", goal_thre);
        get_parameter("planning/step_size", step_size);
        get_parameter("planning/h_surf_car", h_surf_car);
        get_parameter("planning/neighbor_radius", neighbor_radius);

        get_parameter("planning/w_fit_plane", fit_plane_arg.w_total_);  
        get_parameter("planning/w_flatness", fit_plane_arg.w_flatness_); 
        get_parameter("planning/w_slope", fit_plane_arg.w_slope_);   
        get_parameter("planning/w_sparsity", fit_plane_arg.w_sparsity_);
        get_parameter("planning/ratio_min", fit_plane_arg.ratio_min_);
        get_parameter("planning/ratio_max", fit_plane_arg.ratio_max_);
        get_parameter("planning/conv_thre", fit_plane_arg.conv_thre_);

        get_parameter("planning/radius_fit_plane", radius_fit_plane);
        get_parameter("planning/max_initial_time", max_initial_time);

        expansion = resolution * expansionCoefficient;

        // 初始化互斥锁
        mutex_ = std::make_shared<std::mutex>();
        is_planning_ = false;

        // 初始化TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

        grid_map_vis_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis", 1);
        path_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("path_vis", 20);
        goal_vis_pub = create_publisher<visualization_msgs::msg::Marker>("goal_vis", 1);
        surf_vis_pub = create_publisher<sensor_msgs::msg::PointCloud2>("surf_vis", 100);
        tree_vis_pub = create_publisher<visualization_msgs::msg::Marker>("tree_vis", 1);
        tree_tra_pub = create_publisher<std_msgs::msg::Float32MultiArray>("tree_tra", 1);
        goal_status_pub = create_publisher<rover_msgs::msg::RoverGoalStatus>("/cur_global_goal_status", 1);
        path_interpolation_pub = create_publisher<std_msgs::msg::Float32MultiArray>("global_path", 1000);
        obs_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("obs_vis", 10);
    
        // 配置订阅选项
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = subscription_callback_group_;
        
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "global_map", 1, 
            [this](const sensor_msgs::msg::PointCloud2 msg) { rcvPointCloudCallBack(msg); },
            sub_options);
            
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cur_goal", 1,
            [this](const geometry_msgs::msg::PoseStamped msg) { rcvGoalCallBack(msg); },
            sub_options);

        obs_cost_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "obs_cost", 100,
            std::bind(&GlobalPlanningNode::rcvObsCostCallBack, this, std::placeholders::_1),
            sub_options);

        // 初始化规划组件
        world = new World(resolution);
        pf_rrt_star = new PFRRTStar(h_surf_car, world);
        configurePlanner();

        // 创建高精度的100ms定时器（带回调组设置）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&GlobalPlanningNode::timerCallback, this),
            timer_callback_group_);
    }
private:
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obs_cost_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_vis_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surf_vis_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_vis_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr tree_tra_pub;
    rclcpp::Publisher<rover_msgs::msg::RoverGoalStatus>::SharedPtr goal_status_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_interpolation_pub;
    rclcpp::TimerBase::SharedPtr timer_;  // 主程序定时器

    // TF2相关
    shared_ptr<tf2_ros::Buffer> tf_buffer_;
    shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 规划组件
    World* world = NULL;
    PFRRTStar* pf_rrt_star = NULL;

    // 状态变量
    bool has_goal = false;
    Vector3d start_pt;
    Vector3d target_pt;
    rover_msgs::msg::RoverGoalStatus curGoalStatus;
    pcl::PointCloud<pcl::PointXYZ> worldCloud;
    pcl::PointCloud<pcl::PointXYZ> projected_cloud;

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
    string map_frame_id;
    string base_frame_id;

    // 回调组
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
    rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
    std::shared_ptr<std::mutex> mutex_;
    std::atomic<bool> is_planning_{false};
    
    // 添加指针有效性检查标记
    std::atomic<bool> world_initialized_{false};

    void configurePlanner() 
    {
        pf_rrt_star->setGoalThre(goal_thre);
        pf_rrt_star->setStepSize(step_size);
        pf_rrt_star->setFitPlaneArg(fit_plane_arg);
        pf_rrt_star->setFitPlaneRadius(radius_fit_plane);
        pf_rrt_star->setNeighborRadius(neighbor_radius);
        pf_rrt_star->goal_vis_pub_ = goal_vis_pub;
        pf_rrt_star->tree_vis_pub_ = tree_vis_pub;
        pf_rrt_star->tree_tra_pub_ = tree_tra_pub;
    }

    void timerCallback() {
        // 获取机器人当前位置
        try {
            auto transform = tf_buffer_->lookupTransform(
                map_frame_id, base_frame_id, tf2::TimePointZero);
            
            // 保护共享数据
            {
                std::lock_guard<std::mutex> lock(*mutex_);
                start_pt.x() = transform.transform.translation.x;
                start_pt.y() = transform.transform.translation.y;
                start_pt.z() = transform.transform.translation.z;
            }

        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "TF异常: %s", ex.what());
            return;
        }

        // 处理目标状态
        {
            std::lock_guard<std::mutex> lock(*mutex_);
            if (has_goal) {
                double remain_distance = (target_pt - start_pt).head<2>().norm();
                RCLCPP_INFO(get_logger(), "剩余距离: %.2f", remain_distance);

                // 更新目标状态
                curGoalStatus.status = (remain_distance <= goal_thre) ? 
                    rover_msgs::msg::RoverGoalStatus::SUCCEEDED : 
                    rover_msgs::msg::RoverGoalStatus::ACTIVE;
                
                goal_status_pub->publish(curGoalStatus);
            }
        }

        // 检查是否有规划正在进行，使用原子变量确保线程安全
        bool expected = false;
        if (!is_planning_.compare_exchange_strong(expected, true)) {
            return;
        }

        // 直接在当前线程执行规划，避免创建新线程带来的开销
        try {
            auto start_time = this->now();
            this->callPlanner();
            auto end_time = this->now();
            
            // 重置规划标志
            is_planning_ = false;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "规划异常: %s", e.what());
            // 确保重置标志
            is_planning_ = false;
        }
        catch (...) {
            RCLCPP_ERROR(get_logger(), "规划未知异常");
            // 确保重置标志
            is_planning_ = false;
        }
    }

    void rcvGoalCallBack(const geometry_msgs::msg::PoseStamped &goalPose)
    {
        RCLCPP_ERROR(this->get_logger(),"****************");
        RCLCPP_ERROR(this->get_logger(),"x=%f", goalPose.pose.position.x);
        RCLCPP_ERROR(this->get_logger(),"y=%f", goalPose.pose.position.y);
        RCLCPP_ERROR(this->get_logger(),"z=%f", goalPose.pose.position.z);
        RCLCPP_ERROR(this->get_logger(),"O_x=%f", goalPose.pose.orientation.x);
        RCLCPP_ERROR(this->get_logger(),"O_y=%f", goalPose.pose.orientation.y);
        RCLCPP_ERROR(this->get_logger(),"O_z=%f", goalPose.pose.orientation.z);
        RCLCPP_ERROR(this->get_logger(),"O_w=%f", goalPose.pose.orientation.w);
        RCLCPP_ERROR(this->get_logger(),"*****************");

        std::lock_guard<std::mutex> lock(*mutex_);
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

        RCLCPP_INFO(this->get_logger(),"Receive the planning target");
    }

    /**
     *@brief receive point cloud to build the grid map
    */
    void rcvPointCloudCallBack(const sensor_msgs::msg::PointCloud2 &pointcloud_map)
    {
        std::lock_guard<std::mutex> lock(*mutex_);
        try {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(pointcloud_map, cloud);
            worldCloud = cloud;
            
            if (world) {
                world->initGridMap(cloud);
                
                for (const auto &pt : cloud)
                {
                    Vector3d obstacle(pt.x, pt.y, pt.z);
                    world->setObs(obstacle);
                }
                visWorld(world, grid_map_vis_pub_);
                world_initialized_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "World指针未初始化");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理点云异常: %s", e.what());
        }
    }

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
    void rcvObsCostCallBack(const sensor_msgs::msg::PointCloud2 &obsCost)
    {
        std::lock_guard<std::mutex> lock(*mutex_);
        RCLCPP_INFO(this->get_logger(),"rcvObsCostCallBack");
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(obsCost, cloud);
        if (!world->has_map_)
            return;

        // 将 cloud 投影到 worldCloud 的最近网格中心
        projectToWorldCloud(cloud, projected_cloud, worldCloud, resolution);

        sensor_msgs::msg::PointCloud2 obs_project;
        pcl::toROSMsg(projected_cloud, obs_project);
        obs_project.header.frame_id = map_frame_id;
        obs_pub_->publish(obs_project);

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

    visWorld(world, grid_map_vis_pub_);
    }

    /**
     *@brief Linearly interpolate the generated path to meet the needs of local planning
    */
    void pubInterpolatedPath(vector<EXECUTION::Node *> &solution, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_interpolation_pub)
    {
        if (path_interpolation_pub == NULL)
        {
            RCLCPP_ERROR(this->get_logger(),"path_interpolation_pub is NULL");
            return;
        }

        RCLCPP_ERROR(this->get_logger(),"path_interpolation_pub is working");
        std_msgs::msg::Float32MultiArray msg;
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
            RCLCPP_WARN(this->get_logger(),"The start point can't be projected.Unable to start PF-RRT* algorithm!!!");
        }

        // 案例2: 如果原点和目标都可以投影，则 PF-RRT* 将执行
        // 全局规划并尝试生成路径
        else if (pf_rrt_star->state() == Global)
        {
            RCLCPP_INFO(this->get_logger(),"Starting PF-RRT* algorithm at the state of global planning");
            int max_iter = 8000;  // 增大最大迭代次数
            double max_time = 50.0;  // 降低初始规划时间限制，更快进行迭代

            // 连续规划，直到找到解或达到最大时间限制
            while (solution.type_ == Path::Empty && max_time < max_initial_time)
            {
                solution = pf_rrt_star->planner(max_iter, max_time);
                max_time += 50.0;  // 更小的时间增量，更频繁地检查结果
            }

            if (solution.nodes_.empty())
            {
                RCLCPP_WARN(this->get_logger(),"No solution found!");
                curGoalStatus.status = REJECTED;
            }
        }
        // 案例3: 如果原点可以投影而目标不能投影，则PF-RRT*
        // 将尝试找到一个临时的转换目标。
        else
        {
            RCLCPP_INFO(this->get_logger(),"Starting PF-RRT* algorithm at the state of rolling planning");
            int max_iter = 2500;  // 增大滚动规划的迭代次数
            double max_time = 80.0;  // 增加滚动规划的时间限制

            solution = pf_rrt_star->planner(max_iter, max_time);

            if (solution.nodes_.empty())
            {
                RCLCPP_WARN(this->get_logger(),"No solution found!");
                curGoalStatus.status = REJECTED;
            }
        }
        
        pubInterpolatedPath(solution.nodes_, path_interpolation_pub);
        visPath(solution.nodes_, path_vis_pub_);
        visSurf(solution.nodes_, surf_vis_pub);

        // 当生成的路径足够短时，就说明已经到达目的地了
        if (solution.type_ == Path::Global && EuclideanDistance(pf_rrt_star->origin(), pf_rrt_star->target()) < goal_thre)
        {
            has_goal = false;
            visOriginAndGoal({}, goal_vis_pub); // 重新初始化
            visPath({}, path_vis_pub_);
            RCLCPP_INFO(this->get_logger(),"The Robot has achieved the goal!!!");
        }

        if (solution.type_ == Path::Empty)
            visPath({}, path_vis_pub_);
    }

    /**
     *@brief On the premise that the origin and target have been specified,call PF-RRT* algorithm for planning.
    *       Accroding to the projecting results of the origin and the target,it can be divided into three cases.
    */
    void callPlanner()
    {
        static double init_time_cost = 0.0;
        
        // 添加指针检查
        if (!world || !world->has_map_) {
            RCLCPP_WARN(this->get_logger(), "World未初始化或地图不可用");
            return;
        }
        
        if (!pf_rrt_star) {
            RCLCPP_ERROR(this->get_logger(), "规划器未初始化");
            return;
        }

        // 使用共享锁保护数据访问
        {
            std::lock_guard<std::mutex> lock(*mutex_);
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
                    int max_iter = 800;  // 增大无目标状态下的迭代次数
                    double max_time = 80.0;  // 增加无目标状态的时间限制
                    pf_rrt_star->planner(max_iter, max_time);
                }
                else
                    RCLCPP_WARN(this->get_logger(),"Attention: the start point can't be projected");
            }
            // If there is a specified moving target,call PF-RRT* to find a solution
            else if (has_goal)
            {
                findSolution();
                init_time_cost = 0.0;
            }
            // The expansion of tree will stop after the process of initialization takes more than 1s
            else
                RCLCPP_INFO(this->get_logger(),"The tree is large enough.Stop expansion!Current size: %d", (int)(pf_rrt_star->tree().size()));
        }
    }

};

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    
    // 创建多线程执行器，增加线程数以提高性能
    rclcpp::ExecutorOptions options;
    options.context = std::make_shared<rclcpp::Context>();
    options.context->init(argc, argv);
    
    // 配置实时性能参数
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        options, 6);  // 使用6个线程提高并行处理能力
    
    auto node = std::make_shared<GlobalPlanningNode>();
    executor->add_node(node);
    
    // 设置执行器的CPU亲和性以提高性能
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    for (int i = 0; i < 6; i++) {
        CPU_SET(i, &cpu_set);
    }
    
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}