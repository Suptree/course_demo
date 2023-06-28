#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import ColorRGBA
from gazebo_msgs.msg import ModelStates
import math
import tf
from geometry_msgs.msg import Twist


class CooperativeNavigation:

    def __init__(self):

        # ファイル読み込み
        self.agent1 = np.loadtxt(
            '/home/nishilab/catkin_ws/src/cource_demo/src/domaru_demo/agent1.csv', delimiter=',')
        self.agent1_goal_index = 1
        self.agent1_goal_pos_x = self.agent1[self.agent1_goal_index][0]
        self.agent1_goal_pos_y = self.agent1[self.agent1_goal_index][1]

        self.agent2 = np.loadtxt(
            '/home/nishilab/catkin_ws/src/cource_demo/src/domaru_demo/agent2.csv', delimiter=',')
        self.agent2_goal_index = 1
        self.agent2_goal_pos_x = self.agent2[self.agent2_goal_index][0]
        self.agent2_goal_pos_y = self.agent2[self.agent2_goal_index][1]

        # 環境から情報習得
        self.sub = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.Callback)

        self.agent1_pub = rospy.Publisher(
            '/hero_0/cmd_vel', Twist, queue_size=1)
        self.agent1_pub_led = rospy.Publisher(
            '/hero_0/led', ColorRGBA, queue_size=1)
        self.agent2_pub = rospy.Publisher(
            '/hero_1/cmd_vel', Twist, queue_size=1)
        self.agent2_pub_led = rospy.Publisher(
            '/hero_1/led', ColorRGBA, queue_size=1)

        self.dist_thr = 0.1
        self.vConst = 0.1

    def Callback(self, model_status):

        agent1_index = model_status.name.index('hero_0')
        agent2_index = model_status.name.index('hero_1')

        agent1_pose = model_status.pose[agent1_index]
        agent2_pose = model_status.pose[agent2_index]

        agent1_pos = agent1_pose.position
        agent1_ori = agent1_pose.orientation
        agent1_angles = tf.transformations.euler_from_quaternion(
            (agent1_ori.x, agent1_ori.y, agent1_ori.z, agent1_ori.w))
        self.agent1_theta = agent1_angles[2]

        agent2_pos = agent2_pose.position
        agent2_ori = agent2_pose.orientation
        agent2_angles = tf.transformations.euler_from_quaternion(
            (agent2_ori.x, agent2_ori.y, agent2_ori.z, agent2_ori.w))
        self.agent2_theta = agent2_angles[2]

        agent1_distance = math.sqrt((agent1_pos.x-self.agent1_goal_pos_x)**2
                                    + (agent1_pos.y-self.agent1_goal_pos_y)**2)
        agent2_distance = math.sqrt((agent2_pos.x-self.agent2_goal_pos_x)**2
                                    + (agent2_pos.y-self.agent2_goal_pos_y)**2)

        agent1_msg = Twist()
        agent2_msg = Twist()

        if (agent1_distance <= self.dist_thr):
            # print("Goal!!")
            color = ColorRGBA()
            color.r = 255
            color.g = 255
            color.b = 0
            color.a = 255
            self.agent1_pub_led.publish(color)

            # if( ゴールが配列最後のとき) その場でとまる
            if (self.agent1_goal_index == len(self.agent1)-1):
                agent1_msg.linear.x = 0.0
                agent1_msg.angular.z = 1.0
            else:
                self.agent1_goal_index += 1
                self.agent1_goal_pos_x = self.agent1[self.agent1_goal_index][0]
                self.agent1_goal_pos_y = self.agent1[self.agent1_goal_index][1]
        else:
            color = ColorRGBA()
            color.r = 0
            color.g = 255
            color.b = 255
            color.a = 255

            self.agent1_pub_led.publish(color)

            v = self.vConst
            yaw = math.atan2(self.agent1_goal_pos_y-agent1_pos.y,
                             self.agent1_goal_pos_x-agent1_pos.x)
            u = yaw - self.agent1_theta
            bound = math.atan2(math.sin(u), math.cos(u))
            w = min(1.0, max(-1.0, bound))
            agent1_msg.linear.x = v
            agent1_msg.angular.z = w

        self.agent1_pub.publish(agent1_msg)

        if (agent2_distance <= self.dist_thr):
            # print("Goal!!")
            color = ColorRGBA()
            color.r = 255
            color.g = 255
            color.b = 0
            color.a = 255
            self.agent2_pub_led.publish(color)

            # if( ゴールが配列最後のとき) その場でとまる
            if (self.agent2_goal_index == len(self.agent2)-1):
                agent2_msg.linear.x = 0.0
                agent2_msg.angular.z = 1.0
            else:
                self.agent2_goal_index += 1
                self.agent2_goal_pos_x = self.agent2[self.agent2_goal_index][0]
                self.agent2_goal_pos_y = self.agent2[self.agent2_goal_index][1]
        else:
            color = ColorRGBA()
            color.r = 0
            color.g = 0
            color.b = 255
            color.a = 255

            self.agent2_pub_led.publish(color)

            v = self.vConst
            yaw = math.atan2(self.agent2_goal_pos_y-agent2_pos.y,
                             self.agent2_goal_pos_x-agent2_pos.x)
            u = yaw - self.agent2_theta
            bound = math.atan2(math.sin(u), math.cos(u))
            w = min(1.0, max(-1.0, bound))
            agent2_msg.linear.x = v
            agent2_msg.angular.z = w

        self.agent2_pub.publish(agent2_msg)


if __name__ == '__main__':
    rospy.init_node('pose_reading')
    wayN = CooperativeNavigation()
    rospy.spin()
