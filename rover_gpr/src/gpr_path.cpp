#include "rover_gpr/gaussian_process_regression.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <fstream>
#include <cstring> // 添加cstring头文件，解决strlen未定义问题
#include <string>  // 添加string头文件
#include <iostream> // 添加iostream头文件
#include <std_msgs/msg/float32_multi_array.hpp>
#include <eigen3/Eigen/Dense>
#include <cstddef>  // 添加对std::size_t的支持
#include <thread>
#include <atomic>   // 添加atomic头文件

using namespace std;
using namespace Eigen;
using std::placeholders::_1;

/**
 * @class GPRNode
 * @brief 高斯过程回归节点，负责处理路径规划数据并进行曲面预测
 * 
 * 订阅全局规划节点发布的路径和树数据，使用高斯过程回归进行地形预测，
 * 并发布预测结果用于后续路径优化
 */
class GPRNode : public rclcpp::Node
{
public:
  GPRNode():Node("gpr_node")
  {
    // 创建回调组
    timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscription_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    regression_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 参数声明和初始化，获取配置文件参数
    this->declare_parameter<string>("file.cfg_path", "");
    filepath = this->get_parameter("file.cfg_path").as_string();

    // 初始化GPR参数
    input_dim = 3;  // 输入维度(x,y,z)
    output_dim = 1;  // 输出维度（可通行性）

    // 初始化锁对象
    mutex_ = std::make_shared<std::mutex>();
    is_processing_ = false;

    // 初始化发布订阅
    init_publishers(); // 预测结果发布器
    init_subscribers();
  }

private:
  // 回调组
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr subscription_callback_group_;
  rclcpp::CallbackGroup::SharedPtr regression_callback_group_;
  std::shared_ptr<std::mutex> mutex_;
  std::atomic<bool> is_processing_{false};

  // 类型定义 
  typedef Eigen::Matrix<float,3,1> input_type;  // 三维输入(x,y,z)
  typedef Eigen::Matrix<float,1,1> output_type;  // 一维输出(可通行性)

  // ROS2接口
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr tree_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr path_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr surf_predict_pub_;

  // 数据存储
  vector<input_type> train_inputs, test_inputs;
  vector<output_type> train_outputs, test_outputs;

  // 参数
  string filepath;
  std::size_t input_dim;  // 固定为3，与input_type保持一致
  std::size_t output_dim;

  // 初始化消息发布方
  void init_publishers()
  {
    // 使用rclcpp::Node的接口创建发布者
    surf_predict_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/surf_predict_pub", 10);
  }

  // 初始化消息接收方
  void init_subscribers()
  {
    // 配置订阅选项
    rclcpp::SubscriptionOptions tree_options;
    tree_options.callback_group = subscription_callback_group_;
    
    rclcpp::SubscriptionOptions path_options;
    path_options.callback_group = subscription_callback_group_;

    // 订阅全局规划树数据（格式：[x, y, z, 可通行性]）
    tree_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "tree_tra", 10, 
            std::bind(&GPRNode::tree_callback, this, std::placeholders::_1),
            tree_options);
    
    // 订阅全局路径数据（格式：[x, y, z]）
    path_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "global_path", 10, 
            std::bind(&GPRNode::path_callback, this, std::placeholders::_1),
            path_options);
  }

  /**
   * @brief 处理全局规划树数据回调
   * @param msg 包含地形点数据的消息（格式：x,y,z,可通行性）
   * 
   * 功能流程：
   * 1. 解析消息数据到训练集
   * 2. 配置GPR超参数
   * 3. 训练GPR模型
   * 4. 对规划路径进行预测
   * 5. 发布预测结果
   */
  void tree_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // 检查是否有其他处理正在进行
    bool expected = false;
    if (!is_processing_.compare_exchange_strong(expected, true)) {
      RCLCPP_INFO(get_logger(), "上一次处理尚未完成，跳过本次处理");
      return;
    }

    // 创建强引用指针
    auto node_ptr = shared_from_this();
    
    // 启动新线程进行密集型计算
    std::thread process_thread([this, msg, node_ptr]() {
      try {
        this->process_tree_data(msg);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "处理树数据异常: %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "处理树数据时发生未知异常");
      }
      
      // 处理完成，重置标志
      is_processing_ = false;
    });
    process_thread.detach();
  }

  void process_tree_data(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if(msg->data.empty()) return;
    auto start = this->now();

    // 数据解析
    {
      std::lock_guard<std::mutex> lock(*mutex_);
      train_inputs.clear();
      train_outputs.clear();
      
      int num = (int)(msg->data.size()/4);
      for (int i=0; i<num; i++) 
      {
        input_type tmp_in;
        tmp_in << msg->data[4*i], msg->data[4*i+1], msg->data[4*i+2];
        train_inputs.push_back(tmp_in);
        output_type tmp_out;
        tmp_out << msg->data[4*i+3];
        train_outputs.push_back(tmp_out);
      }
    }

    // 检查数据状态
    std::vector<input_type> current_train_inputs;
    std::vector<output_type> current_train_outputs;
    std::vector<input_type> current_test_inputs;
    
    {
      std::lock_guard<std::mutex> lock(*mutex_);
      if(train_inputs.empty() || test_inputs.empty()) {
        RCLCPP_WARN(get_logger(), "训练数据或测试数据为空，跳过回归");
        return;
      }
      
      current_train_inputs = train_inputs;
      current_train_outputs = train_outputs;
      current_test_inputs = test_inputs;
    }

    // GPR高斯过程回归
    GaussianProcessRegression<float> myGPR(input_dim, output_dim);
    set_hyperparameters_from_file(filepath, myGPR);

    for(std::size_t k=0; k<current_train_inputs.size(); k++)
    {
      myGPR.AddTrainingData(current_train_inputs[k], current_train_outputs[k]);
    }

    // 预测结果
    std_msgs::msg::Float32MultiArray out_ym;
    
    for(std::size_t k=0; k<current_test_inputs.size(); k++)
    {
      auto outp = myGPR.DoRegression(current_test_inputs[k]);
      output_type tmp_out;
      tmp_out << outp;
      
      auto outp_cov = myGPR.DoRegressioncov(current_test_inputs[k]);
      out_ym.data.push_back(current_test_inputs[k](0,0));
      out_ym.data.push_back(current_test_inputs[k](1,0));
      out_ym.data.push_back(current_test_inputs[k](2,0));
      out_ym.data.push_back(tmp_out(0,0));
      out_ym.data.push_back(outp_cov(0,0));
    }

    // 发布结果
    surf_predict_pub_->publish(out_ym);
    
    auto dur = this->now() - start;
    RCLCPP_INFO(get_logger(), "GPR处理完成，耗时: %.2f秒", dur.seconds());
  }

  template<typename R>
  void set_hyperparameters_from_file(const string &fname, GaussianProcessRegression<R> & gpr) 
  {
    ifstream myfile;
    myfile.open(fname);
    if (!myfile.is_open())
    {
        cout << "Fail to open the file" << endl;
        return;
    }
    R l, f, n;
    myfile>>l>>f>>n;
    myfile.close();
    gpr.SetHyperParams(l,f,n);
  }

  // path回调函数
  void path_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "收到路径数据");
    if(msg->data.empty()) return;

    // 使用锁保护共享数据
    std::lock_guard<std::mutex> lock(*mutex_);
    
    // 清空之前的测试数据
    test_inputs.clear();
    
    int num = (int)(msg->data.size()/3);
    for (int i=0; i<num; i++) 
    {
      input_type tmp_in;
      tmp_in <<msg->data[3*i],msg->data[3*i+1],msg->data[3*i+2];
      test_inputs.push_back(tmp_in);
    }
    
    RCLCPP_INFO(get_logger(), "收到 %d 个路径点", num);
  }

  // 清理容器
  void clean_vectors()
  {
    vector<input_type>().swap(train_inputs);
    vector<input_type>().swap(test_inputs);
    vector<output_type>().swap(train_outputs);
    vector<output_type>().swap(test_outputs);
  }
};


// 从未用到的参数，予以保留
double length_scale ;
double sigma_f ;
double sigma_n ;
// 调用文件训练模型，暂时用不到，予以保留 
template<typename input_type, typename output_type>
void load_data(const char *fname, std::vector<input_type> &inputs, std::vector<output_type> &outputs, int input_dim, int output_dim) {
  std::cout<<"entry this branch........"<<std::endl;
  input_type inp,tinp;
  output_type outp,toutp;
  std::ifstream myfile(fname);
  if (!myfile.is_open())
  {
      std::cout << "Fail to open the file" << std::endl;
      return;
  }
  std::string line;
  while(getline(myfile,line)){
      std::cout<<line<<" ";
    std::istringstream line_stream(line);
    for(size_t k = 0; k < input_dim; k++)
      line_stream>>inp(k);
    for(size_t k = 0; k < output_dim; k++)
      line_stream>>outp(k);
    inputs.push_back(inp);
    outputs.push_back(outp);
  }
  std::cout<<"finish loading..."<<std::endl;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);
  
  // 创建多线程执行器
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 3);  // 使用3个线程
    
  auto node = std::make_shared<GPRNode>();
  executor.add_node(node);
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
};
