#include <cstdio>

#include <ros/console.h>

#include <fstream>
#include <sstream>
#include <QDebug>
#include <QDir>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/qheaderview.h>

#include "multi_navi_goal_panel.h"
// #include <rover_msgs/roverGoalStatus.h>

namespace rviz_multi_goals_manager_plugins
{

    MultiNaviGoalsPanel::MultiNaviGoalsPanel(QWidget *parent)
        : rviz::Panel(parent), nh_(), maxNumGoal_(1)
    { // 设置自定义窗口标题
        if (this->parentWidget())
        {
            this->parentWidget()->setWindowTitle("你好");
        }

        this->setName("你好");
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("goal", 1,
                                                              boost::bind(&MultiNaviGoalsPanel::goalCntCB, this, _1));

        global_goal_status_sub_ = nh_.subscribe<rover_msgs::roverGoalStatus>("/cur_global_goal_status", 1,
                                                                             boost::bind(&MultiNaviGoalsPanel::statusGlobalPoseCB, this, _1));
        local_goal_status_sub_ = nh_.subscribe<rover_msgs::roverGoalStatus>("/cur_local_goal_status", 1,
                                                                            boost::bind(&MultiNaviGoalsPanel::statusLocalPoseCB, this, _1));
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cur_goal", 1);

        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);

        QVBoxLayout *root_layout = new QVBoxLayout;
        // create a panel about "maxNumGoal"
        QHBoxLayout *maxNumGoal_layout = new QHBoxLayout;
        // maxNumGoal_layout->addWidget(new QLabel("模式切换"));

        output_maxNumGoal_button_ = new QPushButton("多点模式");
        output_maxNumGoal_button_->setToolTip("切换到多点导航模式");
        output_maxNumGoal_button_->setContentsMargins(5, 5, 55, 5);

        maxNumGoal_layout->addWidget(output_maxNumGoal_button_);
        // 创建 QLabel 显示图片
        QLabel *image_label = new QLabel();
        image_label->setAlignment(Qt::AlignCenter);

        QPixmap pixmap("/home/wilson/catkin_ws_movebase_3d/src/rover_slam/rviz_plugins/rviz_multi_goals_manager_plugins/image/logo3.png");
        if (!pixmap.isNull())
        {
            image_label->setPixmap(pixmap.scaled(100, 100, Qt::KeepAspectRatio));
        }
        else
        {
            image_label->setText("无法加载图片");
        }
        maxNumGoal_layout->addWidget(image_label);

        root_layout->addLayout(maxNumGoal_layout);

        cycle_checkbox_ = new QCheckBox("开启循环");
        root_layout->addWidget(cycle_checkbox_);
        // creat a QTable to contain the poseArray
        poseArray_table_ = new QTableWidget;
        initPoseTable();
        poseArray_table_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        root_layout->addWidget(poseArray_table_);
        // creat a manipulate layout
        QHBoxLayout *manipulate_layout = new QHBoxLayout;
        output_reset_button_ = new QPushButton("重置列表");
        output_reset_button_->setToolTip("清空目标点列表");
        manipulate_layout->addWidget(output_reset_button_);

        output_startNavi_button_ = new QPushButton("开始导航");
        output_startNavi_button_->setToolTip("启动导航任务");

        manipulate_layout->addWidget(output_startNavi_button_);
        manipulate_layout->setSpacing(10);
        manipulate_layout->setContentsMargins(5, 5, 5, 5);
        root_layout->addLayout(manipulate_layout);

        setLayout(root_layout);
        // set a Qtimer to start a spin for subscriptions
        QTimer *output_timer = new QTimer(this);
        output_timer->start(200);

        setMaxNumGoal("200");

        // 设置信号与槽的连接
        connect(output_maxNumGoal_button_, SIGNAL(clicked()), this,
                SLOT(changeEvent()));
        connect(output_reset_button_, SIGNAL(clicked()), this, SLOT(initPoseTable()));
        connect(output_startNavi_button_, SIGNAL(clicked()), this, SLOT(startNavi()));
        connect(cycle_checkbox_, SIGNAL(clicked(bool)), this, SLOT(checkCycle()));
        connect(output_timer, SIGNAL(timeout()), this, SLOT(startSpin()));
    }

    // 更新maxNumGoal命名
    void MultiNaviGoalsPanel::changeEvent()
    {
        ROS_ERROR("*********************************************updateMaxNumGoal");

        Q_EMIT configChanged();
    }

    // set up the maximum number of goals
    void MultiNaviGoalsPanel::setMaxNumGoal(const QString &new_maxNumGoal)
    {
        // 检查maxNumGoal是否发生改变.
        if (new_maxNumGoal != output_maxNumGoal_)
        {
            output_maxNumGoal_ = new_maxNumGoal;

            // 如果命名为空，不发布任何信息
            if (output_maxNumGoal_ == "")
            {
                nh_.setParam("maxNumGoal_", 1);
                maxNumGoal_ = 1;
            }
            else
            {
                //                velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_maxNumGoal_.toStdString(), 1);
                nh_.setParam("maxNumGoal_", output_maxNumGoal_.toInt());
                maxNumGoal_ = output_maxNumGoal_.toInt();
            }
            Q_EMIT configChanged();
        }
    }

    // initialize the table of pose
    void MultiNaviGoalsPanel::initPoseTable()
    {
        ROS_INFO("Initialize");
        curGoalIdx_ = 0, cycleCnt_ = 0;
        permit_ = false, cycle_ = false;
        poseArray_table_->clear();
        pose_array_.poses.clear();
        deleteMark();
        poseArray_table_->setRowCount(200);
        poseArray_table_->setColumnCount(3);
        poseArray_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        poseArray_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        cycle_checkbox_->setCheckState(Qt::Unchecked);
    }

    // delete marks in the map
    void MultiNaviGoalsPanel::deleteMark()
    {
        visualization_msgs::Marker marker_delete;
        marker_delete.action = visualization_msgs::Marker::DELETEALL;
        marker_pub_.publish(marker_delete);
    }

    // update the table of pose
    void MultiNaviGoalsPanel::updatePoseTable()
    {
        poseArray_table_->setRowCount(maxNumGoal_);
        //        pose_array_.poses.resize(maxNumGoal_);
        QStringList pose_header;
        pose_header << "x" << "y" << "yaw";
        poseArray_table_->setHorizontalHeaderLabels(pose_header);
        poseArray_table_->show();
    }

    // call back function for counting goals
    void MultiNaviGoalsPanel::goalCntCB(const geometry_msgs::PoseStamped::ConstPtr &pose)
    {
        if (pose_array_.poses.size() < maxNumGoal_)
        {
            pose_array_.poses.push_back(pose->pose);
            pose_array_.header.frame_id = pose->header.frame_id;
            writePose(pose->pose);
            markPose(pose);

            ROS_ERROR("pose_array_.poses.size(): %ld", pose_array_.poses.size());
        }
        else
        {
            ROS_ERROR("Beyond the maximum number of goals: %d", maxNumGoal_);
        }
    }

    // write the poses into the table
    void MultiNaviGoalsPanel::writePose(geometry_msgs::Pose pose)
    {

        poseArray_table_->setItem(pose_array_.poses.size() - 1, 0,
                                  new QTableWidgetItem(QString::number(pose.position.x, 'f', 2)));
        poseArray_table_->setItem(pose_array_.poses.size() - 1, 1,
                                  new QTableWidgetItem(QString::number(pose.position.y, 'f', 2)));
        poseArray_table_->setItem(pose_array_.poses.size() - 1, 2,
                                  new QTableWidgetItem(
                                      QString::number(tf::getYaw(pose.orientation) * 180.0 / 3.14, 'f', 2)));
    }

    // when setting a Navi Goal, it will set a mark on the map
    void MultiNaviGoalsPanel::markPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
    {
        if (ros::ok())
        {
            visualization_msgs::Marker arrow;
            visualization_msgs::Marker number;
            arrow.header.frame_id = number.header.frame_id = pose->header.frame_id;
            arrow.ns = "navi_point_arrow";
            number.ns = "navi_point_number";
            arrow.action = number.action = visualization_msgs::Marker::ADD;
            arrow.type = visualization_msgs::Marker::ARROW;
            number.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            arrow.pose = number.pose = pose->pose;
            arrow.scale.x = 1.0;
            arrow.scale.y = 0.2;
            arrow.scale.z = 0.2;

            arrow.pose.position.z += 0.5;
            number.pose.position.z += 1.4;

            number.scale.z = 1.0;
            arrow.color.r = number.color.r = 0.08f;
            arrow.color.g = number.color.g = 0.08f;
            arrow.color.b = number.color.b = 0.08f;
            arrow.color.a = number.color.a = 1.0;

            number.color.r = number.color.r = 1.0f;
            number.color.g = number.color.g = 0.08f;
            number.color.b = number.color.b = 0.08f;
            number.color.a = number.color.a = 1.0;

            arrow.id = number.id = pose_array_.poses.size();
            number.text = std::to_string(pose_array_.poses.size());
            marker_pub_.publish(arrow);
            marker_pub_.publish(number);
        }
    }

    // check whether it is in the cycling situation
    void MultiNaviGoalsPanel::checkCycle()
    {
        cycle_ = cycle_checkbox_->isChecked();
    }

    // start to navigate, and only command the first goal
    void MultiNaviGoalsPanel::startNavi()
    {

        // 获取当前按钮文本
        QString currentText = output_startNavi_button_->text();
        // 根据文本内容切换显示
        if (currentText == tr("开始导航"))
        {
            output_startNavi_button_->setText(tr("停止导航"));
            output_reset_button_->setEnabled(false);
            // 添加导航启动逻辑
            curGoalIdx_ = curGoalIdx_ % pose_array_.poses.size();
            if (!pose_array_.poses.empty() && curGoalIdx_ < maxNumGoal_)
            {
                geometry_msgs::PoseStamped goal;
                goal.header = pose_array_.header;
                goal.pose = pose_array_.poses.at(curGoalIdx_);
                goal_pub_.publish(goal);

                ROS_INFO("Navi to the Goal%d", curGoalIdx_ + 1);
                poseArray_table_->item(curGoalIdx_, 0)->setBackgroundColor(QColor(255, 69, 0));
                poseArray_table_->item(curGoalIdx_, 1)->setBackgroundColor(QColor(255, 69, 0));
                poseArray_table_->item(curGoalIdx_, 2)->setBackgroundColor(QColor(255, 69, 0));
                curGoalIdx_ += 1;
                permit_ = true;
            }
            else
            {
                ROS_ERROR("Something Wrong");
            }
        }
        else
        {
            output_startNavi_button_->setText(tr("开始导航"));
            output_reset_button_->setEnabled(true);
            // 添加导航停止逻辑
        }
    }

    // complete the remaining goals
    void MultiNaviGoalsPanel::completeNavi()
    {
        if (curGoalIdx_ < pose_array_.poses.size())
        {
            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_);
            goal_pub_.publish(goal);
            ROS_ERROR(">>>>>>>>>>>>>>>>>>>>>>Navi to the Goal%d", curGoalIdx_ + 1);
            poseArray_table_->item(curGoalIdx_, 0)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 1)->setBackgroundColor(QColor(255, 69, 0));
            poseArray_table_->item(curGoalIdx_, 2)->setBackgroundColor(QColor(255, 69, 0));
            curGoalIdx_ += 1;
            permit_ = true;
        }
        else
        {
            ROS_ERROR(">>> All goals are completed <<<");
            permit_ = false;
            curGoalIdx_ = 0;
        }
    }

    // command the goals cyclically
    void MultiNaviGoalsPanel::cycleNavi()
    {
        ROS_ERROR(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>cycleNavi start");

        if (permit_)
        {
            ROS_ERROR(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>cycleNavi");

            geometry_msgs::PoseStamped goal;
            goal.header = pose_array_.header;
            goal.pose = pose_array_.poses.at(curGoalIdx_ % pose_array_.poses.size());
            goal_pub_.publish(goal);
            ROS_INFO("Navi to the Goal%lu, in the %dth cycle", curGoalIdx_ % pose_array_.poses.size() + 1,
                     cycleCnt_ + 1);
            bool even = ((cycleCnt_ + 1) % 2 != 0);
            QColor color_table;
            if (even)
                color_table = QColor(255, 69, 0);
            else
                color_table = QColor(100, 149, 237);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 0)->setBackgroundColor(color_table);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 1)->setBackgroundColor(color_table);
            poseArray_table_->item(curGoalIdx_ % pose_array_.poses.size(), 2)->setBackgroundColor(color_table);
            curGoalIdx_ += 1;
            cycleCnt_ = curGoalIdx_ / pose_array_.poses.size();
        }
    }

    // call back for listening current state
    void MultiNaviGoalsPanel::statusGlobalPoseCB(const rover_msgs::roverGoalStatus::ConstPtr &poseStatus)
    {
        globalGoalStatus = *poseStatus;
    }
    // call back for listening current state
    void MultiNaviGoalsPanel::statusLocalPoseCB(const rover_msgs::roverGoalStatus::ConstPtr &poseStatus)
    {
        localGoalStatus = *poseStatus;

        arrived_ = checkGoalStatus(globalGoalStatus, localGoalStatus);
        if (arrived_ == true && ros::ok() && permit_)
        {
            if (cycle_)
                cycleNavi();
            else
                completeNavi();
        }
    }
    // check the current state of goal
    /**
     * uint8 PENDING=0
     * uint8 ACTIVE=1
     * uint8 PREEMPTED=2
     * uint8 SUCCEEDED=3
     * uint8 ABORTED=4
     * uint8 REJECTED=5
     * uint8 PREEMPTING=6
     * uint8 RECALLING=7
     * uint8 RECALLED=8
     * uint8 LOST=9
     */
    bool MultiNaviGoalsPanel::checkGoalStatus(rover_msgs::roverGoalStatus globalStatus, rover_msgs::roverGoalStatus localStatus)
    {
        bool done;
        if (globalStatus.x == localStatus.x && globalStatus.y == localStatus.y)
        {

            if (globalStatus.status == 3 && localStatus.status == 3)
            {
                done = true;
                // ROS_ERROR("has reached the Goal( %f, %f )", globalStatus.x, globalStatus.y);
            }
            else
            {
                done = false;
            }
        }

        return done;
    }

    // check the current state of goal
    /**
     * uint8 PENDING=0
     * uint8 ACTIVE=1
     * uint8 PREEMPTED=2
     * uint8 SUCCEEDED=3
     * uint8 ABORTED=4
     * uint8 REJECTED=5
     * uint8 PREEMPTING=6
     * uint8 RECALLING=7
     * uint8 RECALLED=8
     * uint8 LOST=9
     */
    bool MultiNaviGoalsPanel::checkGoal(std::vector<actionlib_msgs::GoalStatus> status_list)
    {
        bool done;
        if (!status_list.empty())
        {
            for (auto &i : status_list)
            {
                if (i.status == 3)
                {
                    done = true;
                    ROS_INFO("completed Goal%d", curGoalIdx_);
                }
                else if (i.status == 4)
                {
                    ROS_ERROR("Goal%d is Invalid, Navi to Next Goal%d", curGoalIdx_, curGoalIdx_ + 1);
                    return true;
                }
                else if (i.status == 0)
                {
                    done = true;
                }
                else if (i.status == 1)
                {
                    cur_goalid_ = i.goal_id;
                    done = false;
                }
                else
                    done = false;
            }
        }
        else
        {
            ROS_INFO("Please input the Navi Goal");
            done = false;
        }
        return done;
    }

    // spin for subscribing
    void MultiNaviGoalsPanel::startSpin()
    {
        if (ros::ok())
        {
            ros::spinOnce();
        }
    }

} // end namespace navi-multi-goals-pub-rviz-plugin

// 声明此类是一个rviz的插件

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_multi_goals_manager_plugins::MultiNaviGoalsPanel, rviz::Panel)
