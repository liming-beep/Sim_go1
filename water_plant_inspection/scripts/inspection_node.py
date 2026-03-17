#!/usr/bin/env python3
"""
水厂巡检节点
按顺序导航至各巡检点，到达后停留并记录日志，最终返回起点完成闭合巡检。

启动流程：
  Terminal 1: roslaunch water_plant_inspection simulation.launch
              (等10s) 按 '2' 站立 -> (等2s) 按 '5' 切换 MoveBase
  Terminal 2: roslaunch water_plant_inspection inspection.launch
"""

import rospy
import yaml
import math
import os
import datetime
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


class InspectionNode:
    def __init__(self):
        rospy.init_node('inspection_node')

        # 先把所有属性初始化为安全默认值，防止 run() 访问未定义属性
        self.state       = 'done'
        self.robot_x     = 0.0
        self.robot_y     = 0.0
        self.idx         = 0
        self.arrive_time = None
        self.points      = []
        self.log_f       = None

        config_file = rospy.get_param('~config_file', '')
        if not config_file or not os.path.exists(config_file):
            rospy.logfatal(f"[巡检] Config file not found: '{config_file}'")
            return

        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)['inspection']

        self.threshold     = config['arrival_threshold']
        self.stop_dur      = config['stop_duration']
        self.startup_delay = config.get('startup_delay', 5.0)
        self.points        = config['points']

        # log_file 路径：支持绝对路径，$(find ...) 语法由 roslaunch 展开后传入
        log_path = config.get('log_file', '/tmp/inspection_log.txt')
        # 如果路径里还有未展开的 $(find ...)，退回到 /tmp
        if '$(' in log_path:
            log_path = '/tmp/inspection_log.txt'
            rospy.logwarn("[巡检] log_file 路径未展开，使用 %s", log_path)
        log_dir = os.path.dirname(log_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.log_f = open(log_path, 'a', buffering=1)

        self.goal_pub = rospy.Publisher('/goal_point', PointStamped, queue_size=1, latch=True)
        rospy.Subscriber('/state_estimation', Odometry, self._odom_cb)

        rospy.loginfo("[巡检] 等待机器人位姿 /state_estimation ...")
        rospy.wait_for_message('/state_estimation', Odometry)
        rospy.loginfo("[巡检] 机器人就绪，等待导航栈 %.0fs ...", self.startup_delay)
        rospy.sleep(self.startup_delay)

        self._log("巡检任务开始")
        rospy.loginfo("=" * 50)
        rospy.loginfo("[巡检] 任务开始，共 %d 个巡检点", len(self.points))
        rospy.loginfo("=" * 50)

        self.state = 'navigating'
        self._send_goal()

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _send_goal(self):
        if self.idx >= len(self.points):
            self._finish()
            return
        pt = self.points[self.idx]
        msg = PointStamped()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.point.x = float(pt['x'])
        msg.point.y = float(pt['y'])
        msg.point.z = 0.0
        self.goal_pub.publish(msg)
        rospy.loginfo("[巡检] 导航至 [%d] %s  (%.1f, %.1f)",
                      pt['id'], pt['name'], pt['x'], pt['y'])

    def _log(self, text):
        if self.log_f is None:
            return
        ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        self.log_f.write(f"[{ts}] {text}\n")

    def _finish(self):
        rospy.loginfo("=" * 50)
        rospy.loginfo("[巡检] 巡检任务全部完成！")
        rospy.loginfo("=" * 50)
        self._log("巡检任务全部完成！")
        if self.log_f:
            self.log_f.close()
        self.state = 'done'

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 'navigating':
                pt   = self.points[self.idx]
                dx   = self.robot_x - pt['x']
                dy   = self.robot_y - pt['y']
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < self.threshold:
                    self.state       = 'arrived'
                    self.arrive_time = rospy.Time.now()
                    arrive_msg = f"已到达 {pt['name']}"
                    rospy.loginfo("[巡检] ✓ %s  (距离 %.2fm)", arrive_msg, dist)
                    self._log(arrive_msg)

            elif self.state == 'arrived':
                elapsed = (rospy.Time.now() - self.arrive_time).to_sec()
                if elapsed >= self.stop_dur:
                    self.idx  += 1
                    self.state = 'navigating'
                    self._send_goal()

            elif self.state == 'done':
                break

            rate.sleep()


if __name__ == '__main__':
    try:
        node = InspectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
