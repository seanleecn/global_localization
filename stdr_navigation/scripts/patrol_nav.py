#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import random
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class PatrolNav():  # 定义一个PatrolNav类

    # 初始化函数
    def __init__(self):
        rospy.init_node('patrol_nav_node', anonymous=False)  # 注册和初始化node
        rospy.on_shutdown(self.shutdown)  # 结束时调用shutdown函数

        # 从launch文件获取参数
        self.rest_time = rospy.get_param("~rest_time", 5)
        self.keep_patrol = rospy.get_param("~keep_patrol", False)
        self.random_patrol = rospy.get_param("~random_patrol", False)
        self.patrol_type = rospy.get_param("~patrol_type", 0)
        self.patrol_loop = rospy.get_param("~patrol_loop", 2)
        self.patrol_time = rospy.get_param("~patrol_time", 5)

        # 设置导航点
        self.locations = dict()
        self.locations['one'] = Pose(
            Point(21.2, 28.1, 0), Quaternion(0.0, 0.0, -0.7, 0.7))
        self.locations['two'] = Pose(
            Point(20.2, 6.1, 0), Quaternion(0.0, 0.0, -0.7,  0.7))
        self.locations['three'] = Pose(
            Point(28.4, 4.9, 0), Quaternion(0, 0, 0.08, 0.9))
        self.locations['four'] = Pose(
            Point(28.6, 14.9, 0), Quaternion(0, 0, 0.7, 0.7))
        self.locations['five'] = Pose(
            Point(20.9, 15.7, 0), Quaternion(0, 0, 0.99, -0.01))

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                       'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        # 订阅move_base action server初始化一个SimpleActionClient
        # 可以获取到move_base在导航过程中的状态反馈，根据反馈的状态来决定何时发送下一个目标点的坐标
        self.move_base = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move_base server")

        # Variables to keep track of success rate, running time etc.
        loop_cnt = 0
        n_goals = 0
        n_successes = 0
        target_num = 0
        running_time = 0
        location = ""
        start_time = rospy.Time.now()
        locations_cnt = len(self.locations)
        sequeue = ['one', 'two', 'three', 'four', 'five']

        rospy.loginfo("开始巡航")
        while not rospy.is_shutdown():
            # judge if set keep_patrol is true
            # 固定圈数巡航的情况
            if self.keep_patrol == False:
                if self.patrol_type == 0:  # patrol_type=0,即按圈巡航
                    if target_num == locations_cnt:
                        if loop_cnt < self.patrol_loop-1:
                            target_num = 0
                            loop_cnt += 1
                            rospy.logwarn("Left patrol loop cnt: %d",
                                          self.patrol_loop-loop_cnt)
                        else:
                            rospy.logwarn("Now patrol loop over, exit...")
                            rospy.signal_shutdown('Quit')
                            break
            # 无限巡航的情况
            else:
                # 随机目标点巡航
                if self.random_patrol == False:
                    if target_num == locations_cnt:
                        target_num = 0
                else:
                    target_num = random.randint(0, locations_cnt-1)

            # Get the next location in the current sequence
            # 取出目标点,发送给move_base
            location = sequeue[target_num]
            rospy.loginfo("前往: " + str(location))
            self.send_goal(location)  # 用send_goal函数发送目标点的坐标

            # Increment the counters
            target_num += 1
            n_goals += 1

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(
                rospy.Duration(300))
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR:Timed out achieving goal")
            else:
                # 从move_base 的client中获得当前的导航状态
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    n_successes += 1
                    rospy.loginfo("Goal succeeded!")
                else:
                    rospy.logerr("Goal failed with error code:" +
                                 str(goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs/60.0

            # Print a summary success/failure and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " +
                          str(self.trunc(running_time, 1)) + " min")
            # 到达一个目标点后会停止一段时间,默认为5s
            rospy.sleep(self.rest_time)

            if self.keep_patrol == False and self.patrol_type == 1:  # use patrol_time
                if running_time >= self.patrol_time:
                    rospy.logwarn(
                        "Now reach patrol_time, back to original position...")
                    self.send_goal('six')
                    rospy.signal_shutdown('Quit')

    def send_goal(self, locate):
        # Set up the next goal location
        self.goal = MoveBaseGoal()  # 初始化一个对象
        self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # send goal to move_base用move_base的send_goal函数发送
        self.move_base.send_goal(self.goal)

    def trunc(self, f, n):
        # 为了截短字节的长度
        # Truncates/pads a float f to n decimal places without rounding
        slen = len('%.*f' % (n, f))
        return float(str(f)[:slen])

    def shutdown(self):
        rospy.logwarn("巡航停止")  # 可以增加保存数据等功能


if __name__ == '__main__':
    try:
        PatrolNav()  # 调用类的初始化函数__init__
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")
