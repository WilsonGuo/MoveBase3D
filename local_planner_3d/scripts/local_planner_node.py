#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float64, Float32MultiArray, Int16
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Twist
from nav_msgs.msg import Path
from rover_msgs.msg import roverGoalStatus
import numpy as np
import tf
from visualization_msgs.msg import Marker, MarkerArray
import math
from scipy.spatial.transform import Rotation
import time
import casadi as ca


# PENDING = 0       # 任务处于等待状态，尚未开始
# ACTIVE = 1        # 任务正在进行中
# PREEMPTED = 2     # 任务被抢占，执行中止
# SUCCEEDED = 3     # 任务成功完成
# ABORTED = 4       # 任务由于某种原因被中止
# REJECTED = 5      # 任务被拒绝，未开始执行
# PREEMPTING = 6    # 任务正在被抢占的过程中
# RECALLING = 7     # 任务正在被召回的过程中
# RECALLED = 8      # 任务已被成功召回
# LOST = 9          # 任务状态丢失或未知


class Local_Planner():
    def __init__(self):
        self.replan_period = rospy.get_param(
            '~replan_period', 0.01)
        self.cmd_period = rospy.get_param(
            '~cmd_period', 0.01)
        self.state_period = rospy.get_param(
            '~state_period', 0.1)

        self.reached_position_tolerance = rospy.get_param(
            '~reached_position_tolerance', 0.2)
        self.reached_yaw_tolerance = rospy.get_param(
            '~reached_yaw_tolerance', 0.2)
        self.goal_status_period = rospy.get_param(
            '~goal_status_period', 0.1)
        self.goal_min_angular_speed = rospy.get_param(
            '~goal_min_angular_speed', -0.5)

        self.goal_max_angular_speed = rospy.get_param(
            '~goal_max_angular_speed', 0.5)
        self.goal_angular_gain = rospy.get_param(
            '~goal_angular_gain', 2.0)
        
        self.map_frame_id = rospy.get_param(
            '~map_frame_id', '/map')
        self.base_frame_id = rospy.get_param(
            '~base_frame_id', '/base_footprint')
        
        self.global_path_step = rospy.get_param(
            '~global_path_step', 10)
        self.dwa_lookhead_step = rospy.get_param(
            '~dwa_lookhead_step', 0.1)
        self.predict_time = rospy.get_param(
            '~predict_time', 2.0)
        self.goal_weight = rospy.get_param(
            '~goal_weight', 1.0)   
        self.speed_weight = rospy.get_param(
            '~speed_weight', 0.5)        
        self.obstacle_weight = rospy.get_param(
            '~obstacle_weight', 1.9) 
        self.safe_distance = rospy.get_param(
            '~safe_distance', 0.3)      
        self.omega_min = rospy.get_param(
            '~omega_min', -1.0)    
        self.omega_max = rospy.get_param(
            '~omega_max', 1.0)   
        
        self.v_min = rospy.get_param(
            '~v_min', -0.2)    
        self.v_max = rospy.get_param(
            '~v_max', 0.6)  
        
        
        self.curr_state = np.zeros(5)
        self.z = 0
        self.local_plan = np.zeros([self.global_path_step, 2])
        self.best_control = [0.0, 0.0]  # 最优控制输入 [v, ω]

        self.goal_state = np.zeros([self.global_path_step, 4])
        self.ref_path_close_set = False
        self.goal_position_reached = False  # 标记是否到达目标点(x,y)
        self.goal_yaw_reached = False  # 标记是否到达目标点(yaw)

        self.reached_position_tolerance = self.reached_position_tolerance  # 到达目标点的速度容差
        self.goal_position = None
        self.cur_position = [0.0, 0.0, 0.0]
        self.goal_yaw = 0
        self.cur_yaw = 0

        self.target_state = np.array([-1, 4, np.pi/2])
        self.target_state_close = np.zeros(3)
        self.desired_global_path = [np.zeros([300, 4]), 0]
        self.is_close = False
        self.is_get = False
        self.is_grasp = False
        self.is_all_task_down = False
        self.robot_state_set = False
        self.ref_path_set = False
        self.ob = []
        self.is_end = 0
        self.ob_total = []
        self.last_states_sol = np.zeros(self.global_path_step+1)
        self.control_cmd = Twist()

        self.last_cmd_sol = np.zeros([self.global_path_step, 2])

        rospy.Subscriber('/obs_raw', Float32MultiArray,
                         self.obs_cb, queue_size=100)

        rospy.Subscriber('/surf_predict_pub', Float32MultiArray,
                         self.global_path_callback, queue_size=10)

        rospy.Subscriber('/cur_goal', PoseStamped,
                         self.rcvGoalCallBack, queue_size=10)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_status = rospy.Publisher(
            '/cur_local_goal_status', roverGoalStatus, queue_size=10)

        self.pub_local_path = rospy.Publisher(
            '/local_path', Path, queue_size=10)

        self.rover_goal_status = roverGoalStatus()

        self.control_cmd = Twist()
        self.listener = tf.TransformListener()
        self.times = 0
        self.obstacle_markerarray = MarkerArray()
        self.ob_pub = rospy.Publisher('/ob_draw', MarkerArray, queue_size=10)

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.replan_cb()
            self.pub_cmd()
            self.pub_goal_status()
            rate.sleep()

    def distance_sqaure(self, c1, c2):
        distance = (c1[0]-c2[0])*(c1[0]-c2[0])+(c1[1]-c2[1])*(c1[1]-c2[1])
        return distance

    def has_reached_goal(self):
        """
        判断是否已经到达目标点。
        :return: True 如果到达目标点，否则 False。
        """
        current_position = self.curr_state[:2]
        distance = np.sqrt((current_position[0] - self.goal_position.x) ** 2 +
                           (current_position[1] - self.goal_position.y) ** 2)
        return distance <= self.reached_position_tolerance

    def obs_cb(self, data):
        self.ob = []
        start_time = time.time()  # 记录起始时间
        if (len(data.data) != 0):

            size = len(data.data)/3
            for i in range(int(size)):
                self.ob.append(
                    ((data.data[3*i]//0.3)*0.3, (data.data[3*i+1]//0.3)*0.3))
            dic = list(set([tuple(t) for t in self.ob]))
            self.ob = [list(v) for v in dic]
            end_time = time.time()  # 记录结束时间
            elapsed_time = end_time - start_time
            # print(f"obs_cb time: {elapsed_time:.6f} seconds")
    def simulate_trajectory(self,x, y, theta, v, omega, predict_time, dt):
        """
        模拟机器人在一段时间内的轨迹。
        """
        trajectory = []
        for _ in np.arange(0, predict_time, dt):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += omega * dt
            trajectory.append([x, y, theta])
        return np.array(trajectory)

    def evaluate_goal_cost(self,trajectory, goal_x, goal_y):
        """
        计算轨迹到目标的得分（目标距离）。
        """
        last_state = trajectory[-1]
        dist_to_goal = math.sqrt(
            (last_state[0] - goal_x)**2 + (last_state[1] - goal_y)**2)
        return -dist_to_goal  # 距离越近，得分越高

    def evaluate_obstacle_cost(self,trajectory, obstacles, safe_distance):
    
        trajectory = np.array(trajectory)[:, :2]
        obstacles = np.array(obstacles)
        dists = np.linalg.norm(trajectory[:, None, :] - obstacles[None, :, :], axis=2)
        close_dists = dists[dists < safe_distance]
        epsilon = 1e-6
        weights = 1 / (close_dists ** 2 + epsilon)
        cost = np.sum(weights * (safe_distance - close_dists) ** 2)
        return cost

    def localPlan(self,cur_position, goal_position, obstacles, cur_v=0.0, cur_omega=0.0):

        # 参数设置
        v_min, v_max = self.v_min, self.v_max   # 线速度范围 (m/s)
        omega_min, omega_max = self.omega_min, self.omega_max  # 角速度范围 (rad/s)
        predict_time = self.predict_time          # 轨迹预测时间 (s)
        dt = self.dwa_lookhead_step               # 采样时间步长 (s)
        safe_distance = self.safe_distance        # 障碍物安全距离 (m)
        goal_weight = self.goal_weight          # 朝向目标的权重
        speed_weight = self.speed_weight         # 保持速度的权重
        obstacle_weight = self.obstacle_weight      # 避障的权重

        # 当前状态
        x = cur_position[0]
        y = cur_position[1]
        theta = cur_position[2]

        
        goal = self.goal_state[:,:3]
        
        cur_x = goal[-1, 0]  
        cur_y = goal[-1, 1]  
        cur_z = goal[-1, 2]  

        # 动态窗口
        dynamic_window = {
            "v_min": max(v_min, 0),  
            "v_max": min(v_max, 0.6),
            "omega_min": max(omega_min, -1.0),  
            "omega_max": min(omega_max, 1.0)
        }

        # 轨迹采样
        best_trajectory = None
        best_score = float('-inf')
        best_control = [0.0, 0.0]  # 最优控制输入 [v, ω]

        for v in np.arange(dynamic_window["v_min"], dynamic_window["v_max"], 0.1):
            for omega in np.arange(dynamic_window["omega_min"], dynamic_window["omega_max"], 0.1):

                trajectory = self.simulate_trajectory(
                    x, y, theta, v, omega, predict_time, dt)

                goal_score = goal_weight * \
                    self.evaluate_goal_cost(trajectory, cur_x, cur_y)
                speed_score = speed_weight * v
                obstacle_score = -obstacle_weight * \
                    self.evaluate_obstacle_cost(
                        trajectory, obstacles, safe_distance)

                total_score = goal_score + speed_score + obstacle_score

                if total_score > best_score:
                    best_score = total_score
                    best_trajectory = trajectory
                    best_control = [v, omega]

        isOK = best_trajectory is not None
        return best_trajectory, best_control, isOK


    def replan_cb(self):

        if self.goal_position_reached and self.goal_yaw_reached:
            self.rover_goal_status.status = 3

        if self.goal_position_reached:  # 机器人已经到达目标点，停止路径规划
            return

        if self.robot_state_set and self.ref_path_set:

            self.choose_goal_state()   # 更新目标状态

            # 测量 localPlan 的执行时间
            start_time = time.time()  # 记录起始时间

            trajectory, self.best_control, isOK = self.localPlan(
                self.cur_position, self.goal_position, self.ob, self.control_cmd.linear.x, self.control_cmd.angular.z)  # local planning
            end_time = time.time()  # 记录结束时间

            # 计算并打印耗时
            elapsed_time = end_time - start_time
            print(
                f"localPlan execution time: {elapsed_time:.6f} seconds")

            # 如果规划成功，发布路径和指令
            if isOK and not self.goal_position_reached:
                self.publish_local_plan(trajectory)
                self.rover_goal_status.status = 1
            # 检查是否到达目标点
            if self.has_reached_goal():
                self.goal_position_reached = True  # 设置标志

            # 如果规划失败
            if not isOK:
                self.rover_goal_status.status = 5

        elif self.robot_state_set == False and self.ref_path_set == True:
            print("no pose")
            self.best_control = [0.0, 0.0]  # 如果规划失败，设置默认值
        elif self.robot_state_set == True and self.ref_path_set == False:
            self.best_control = [0.0, 0.0]  # 如果规划失败，设置默认值
        else:
            print("********please set your init pose !*********")

            self.best_control = [0.0, 0.0]  # 如果规划失败，设置默认值

    def publish_local_plan(self, trajectory):
        local_path = Path()
        sequ = 0
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "map"

        length = trajectory.shape[0]
        for i in range(length):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = trajectory[i, 0]
            this_pose_stamped.pose.position.y = trajectory[i, 1]
            this_pose_stamped.pose.position.z = self.z + \
                0.5  
            this_pose_stamped.header.seq = sequ
            sequ += 1
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "map"
            local_path.poses.append(this_pose_stamped)

        self.pub_local_path.publish(local_path)

    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y

    def pub_goal_status(self):
        self.pub_status.publish(self.rover_goal_status)

    def pub_cmd(self):
        self.cmd(self.best_control)

        try:
            self.listener.waitForTransform(
                self.map_frame_id, self.base_frame_id, rospy.Time(0), rospy.Duration(1.0))

            (trans, rot) = self.listener.lookupTransform(
                self.map_frame_id, self.base_frame_id, rospy.Time(0))

            roll, pitch, yaw = self.quart_to_rpy(
                rot[0], rot[1], rot[2], rot[3])
            self.curr_state[0] = trans[0]
            self.curr_state[1] = trans[1]
            self.curr_state[2] = (yaw+np.pi) % (2*np.pi)-np.pi
            self.curr_state[3] = roll
            self.curr_state[4] = pitch

            self.z = trans[2]
            self.robot_state_set = True
            self.cur_yaw = yaw

            self.cur_position[0] = trans[0]
            self.cur_position[1] = trans[1]
            self.cur_position[2] = (yaw+np.pi) % (2*np.pi)-np.pi

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("TF transform failed, skipping this cycle.")

            return

    def cmd(self, data):
        # 如果目标未到达，正常行驶
        if not self.goal_position_reached:
            self.control_cmd.linear.x = data[0]
            self.control_cmd.angular.z = data[1]
        else:
            yaw_difference = (self.goal_yaw - self.cur_yaw +
                              np.pi) % (2 * np.pi) - np.pi  # 差值归一化到 [-π, π]

            if abs(yaw_difference) > self.reached_yaw_tolerance:
                # 动态调整角速度
                self.control_cmd.angular.z = max(self.goal_min_angular_speed, min(
                    self.goal_max_angular_speed, self.goal_angular_gain * yaw_difference))
            elif abs(yaw_difference) > 0.05:  # 设置更小的误差范围平滑停止
                self.control_cmd.angular.z = 0.1 * \
                    np.sign(yaw_difference)  # 低速微调
            else:
                self.goal_yaw_reached = True
                # 误差小于阈值范围，停止旋转
                self.control_cmd.angular.z = 0

            self.control_cmd.linear.x = 0  # 停止前进

        self.pub.publish(self.control_cmd)

    def distance_global(self, c1, c2):
        distance = np.sqrt((c1[0]-c2[0])*(c1[0]-c2[0]) +
                           (c1[1]-c2[1])*(c1[1]-c2[1]))
        return distance

    def find_min_distance(self, c1):
        number = np.argmin(np.array([self.distance_global(
            c1, self.desired_global_path[0][i]) for i in range(int(self.desired_global_path[1]))]))
        return number

    def choose_goal_state(self):
        num = self.find_min_distance(self.curr_state)
        scale = 1
        num_list = []
        for i in range(self.global_path_step):
            num_path = min(self.desired_global_path[1]-1, int(num+i*scale))
            num_list.append(num_path)
        if (num >= self.desired_global_path[1]):
            self.is_end = 1
        for k in range(self.global_path_step):
            self.goal_state[k] = self.desired_global_path[0][int(num_list[k])]
        

    def global_path_callback(self, data):
        if (len(data.data) != 0):
            self.ref_path_set = True
            size = len(data.data)/5
            self.desired_global_path[1] = size
            for i in range(int(size)):
                self.desired_global_path[0][i,
                                            0] = data.data[5*(int(size)-i)-5]
                self.desired_global_path[0][i,
                                            1] = data.data[5*(int(size)-i)-4]
                self.desired_global_path[0][i,
                                            2] = data.data[5*(int(size)-i)-2]
                self.desired_global_path[0][i,
                                            3] = data.data[5*(int(size)-i)-1]

    def quaternion_to_yaw(self, quaternion):
        # 创建四元数对象
        r = Rotation.from_quat(quaternion)
        # 提取yaw (绕z轴的旋转角)
        yaw = r.as_euler('xyz', degrees=False)[2]
        return yaw

    def rcvGoalCallBack(self, msg):
        self.goal_position_reached = False  # 重新置位
        self.goal_yaw_reached = False
        # 输出路径包含的点数量
        print('******************rcvGoalCallBack New Goal Pose*******************')

        self.goal_position = msg.pose.position
        print('Goal Position:')
        print(self.goal_position.x)
        print(self.goal_position.y)
        print(self.goal_position.z)
        # 获取四元数
        orientation_q = msg.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y,
                      orientation_q.z, orientation_q.w]
        self.goal_yaw = self.quaternion_to_yaw(quaternion)

        self.rover_goal_status.x = msg.pose.position.x
        self.rover_goal_status.y = msg.pose.position.y
        self.rover_goal_status.z = msg.pose.position.z
        self.rover_goal_status.orientation_x = msg.pose.orientation.x
        self.rover_goal_status.orientation_y = msg.pose.orientation.y
        self.rover_goal_status.orientation_z = msg.pose.orientation.z
        self.rover_goal_status.orientation_w = msg.pose.orientation.w


if __name__ == '__main__':
    rospy.init_node("local_planner")
    phri_planner = Local_Planner()

    rospy.spin()
