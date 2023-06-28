#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import ColorRGBA
from gazebo_msgs.msg import ModelStates
import math
import tf
import time
from geometry_msgs.msg import Twist
import random


class WaypointNavigation:
    MAX_FORWARD_SPEED = 0.1
    MAX_ROTATION_SPEED = 1.0
    cmdmsg = Twist()
    index = 0

    # Tunable parameters
    wGain = 10
    vConst = 0.5
    distThr = 0.1
    pheroThr = 0.3

    def __init__(self):
        # Initialise pheromone values
        self.pheromone_value = [0.0] * 9
        self.sum_pheromone_value = 0.0

        self.robot_theta = 0

        # Goal
        # goal_r = 0.8
        # goal_radius = 2.0 * math.pi * random.random()
        # 01rint("goal_raius = {}".format(math.degrees(goal_radius)))
        # self.goal_pos_x = goal_r * math.cos(goal_radius)
        # self.goal_pos_y = goal_r * math.sin(goal_radius)
        # print("Goal Position = ({}, {})".format(
        #     self.goal_pos_x, self.goal_pos_y))
        # ファイル読み込み
        self.agent1 = np.loadtxt(
            '/home/nishilab/catkin_ws/src/cource_demo/src/domaru_demo/agent1.csv', delimiter=',')
        self.goal_index = 1
        self.goal_pos_x = self.agent1[self.goal_index][0]
        self.goal_pos_y = self.agent1[self.goal_index][1]

        self.agent2 = np.loadtxt(
            '/home/nishilab/catkin_ws/src/cource_demo/src/domaru_demo/agent2.csv', delimiter=',')
        self.agent2_goal_index = 1
        self.agent2_goal_pos_x = self.agent2[self.agent2_goal_index][0]
        self.agent2_goal_pos_y = self.agent2[self.agent2_goal_index][1]

        self.sub = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.Callback)
        self.pub = rospy.Publisher('/hero_0/cmd_vel', Twist, queue_size=1)
        self.pub_led = rospy.Publisher('/hero_0/led', ColorRGBA, queue_size=1)
        self.agent2_pub = rospy.Publisher('/hero_1/cmd_vel', Twist, queue_size=1)
        self.agent2_pub_led = rospy.Publisher('/hero_1/led', ColorRGBA, queue_size=1)
        self.reset_timer = time.process_time()
        self.beta_const = 1.2
        self.sensitivity = 1.2
        self.BIAS = 0.25
        self.V_COEF = 0.6  # self.v_range[0]
        self.W_COEF = 0.4  # self.w_range[0]

        print(self.agent1[0][1])

    def Callback(self, model_status):

        robot_index = model_status.name.index('hero_0')
        agent2_index = model_status.name.index('hero_1')

        pose = model_status.pose[robot_index]
        agent2_pose = model_status.pose[agent2_index]
        # twist = model_status.twist[robot_index]




        pos = pose.position
        ori = pose.orientation

        angles = tf.transformations.euler_from_quaternion(
            (ori.x, ori.y, ori.z, ori.w))

        self.theta = angles[2]

        # P controller
        v = 0
        w = 0
        # print("pos: {}".format(pos))
        # Index for # of goals
        distance = math.sqrt((pos.x-self.goal_pos_x)**2
                             + (pos.y-self.goal_pos_y)**2)
        # print('distance : ' + str(distance))
        # Reset condition reset (to prevent unwanted reset due to delay of position message subscription)
        step_timer = time.process_time()
        reset_time = step_timer - self.reset_timer

        # print("pheromone_value = {}".format(self.pheromone_value))
        msg = Twist()
        if (distance <= self.distThr):
            # print("Goal!!")
            color = ColorRGBA()
            color.r = 255
            color.g = 255
            color.b = 0
            color.a = 255
            self.pub_led.publish(color)

            self.is_goal = True
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            self.goal_index += 1
            self.goal_pos_x = self.agent1[self.goal_index][0]
            self.goal_pos_y = self.agent1[self.goal_index][1]

            # self.reset()

        # Adjust velocities
        elif (distance > self.distThr):
            # print("not pheromone\n")
            color = ColorRGBA()
            color.r = 0
            color.g = 0
            color.b = 255
            color.a = 255
            self.pub_led.publish(color)

            v = self.vConst
            yaw = math.atan2(self.goal_pos_y-pos.y, self.goal_pos_x-pos.x)
            u = yaw - self.theta
            bound = math.atan2(math.sin(u), math.cos(u))
            w = min(1.0, max(-1.0, bound))
            msg.linear.x = v
            # print(v)
            # print(" , ")
            msg.angular.z = w
            # print(w)
            self.reset_flag = False

        # distance_to_obs = [1.0]*len(self.obstacle)
        # for i in range(len(distance_to_obs)):
        #     distance_to_obs[i] = sqrt((pos.x-self.obstacle[i][0])**2+(pos.y-self.obstacle[i][1])**2)
        # if (distance_to_obs[0] < 0.3 or distance_to_obs[1] < 0.3 or distance_to_obs[2] < 0.3 or distance_to_obs[3] < 0.3) and reset_time > 1:
        #     msg = Twist()
        #     self.is_collided = True
        #     self.reset()

        if msg.linear.x > self.MAX_FORWARD_SPEED:
            msg.linear.x = self.MAX_FORWARD_SPEED
        # Publish velocity
        self.pub.publish(msg)

        self.prev_x = pos.x
        self.prev_y = pos.y

        # Reporting
        # print("Distance to goal {}".format(distance))
        # print('Callback: x=%2.2f, y=%2.2f, dist=%4.2f, cmd.v=%2.2f, cmd.w=%2.2f' %(pos.x,pos.y,distance,v,w))


if __name__ == '__main__':
    rospy.init_node('pose_reading')
    wayN = WaypointNavigation()
    rospy.spin()
