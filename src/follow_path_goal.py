#!/usr/bin/python3

import tf
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from numpy import argmin

class PIDController:
    def __init__(self) -> None:
        rospy.init_node('follow_path_goal', anonymous=True)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.k_i = 0.0
        self.k_p = 0.6
        self.k_d = 0.3

        self.goal_k_i = 0.0
        self.goal_k_p = 0.49
        self.goal_k_d = 0.3

        self.dt = 0.005
        self.v = 0.5
        self.D = 2
        rate = 1/self.dt

        self.errors = []
        self.r = rospy.Rate(rate)

    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges
        d = min(rng)
        return d

    def get_heading(self):
        msg = rospy.wait_for_message("/odom", Odometry)
        orientation = msg.pose.pose.orientation

        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x, orientation.y, orientation.z, orientation.w
        ))
        return yaw

    def find_obstacle(self):
        data = rospy.wait_for_message('/scan', LaserScan)
        distance = data.ranges[0]
        msg = rospy.wait_for_message("/odom", Odometry)
        position = msg.pose.pose.position
        goal_angle = int(math.degrees(math.atan2(3 - position.y, (-1)-position.x)))
        #goal_angle = (2*math.pi + current_angle)%(2*math.pi)
        heading_angle = int(math.degrees(self.get_heading()))
        move_to_goal = data.ranges[goal_angle]
        distance_ref = math.sqrt( (3 - position.x)**2 + ((-1)-position.y)**2 )

        if distance > self.D and goal_angle == heading_angle:
            #We are moving to the goal
            return False
        elif move_to_goal >= distance_ref:
            return False
        else:
            #We are near the wall
            return True

    def follow_goal(self):
        msg = rospy.wait_for_message("/odom", Odometry)
        position = msg.pose.pose.position
        goal_angle = int(math.degrees(math.atan2(3 - position.y, (-1)-position.x)))
        #goal_angle = (2*math.pi + current_angle)%(2*math.pi)
        heading_angle = int(math.degrees(self.get_heading()))

        sum_i_angle = 0
        prev_angle_error = 0

        move_cmd = Twist()

        while not self.find_obstacle():
            diff_a = goal_angle - heading_angle
            if diff_a > math.pi:
                error_angle = -2 * math.pi + diff_a
            elif diff_a < -1 * math.pi:
                error_angle = 2 * math.pi + diff_a
            
            sum_i_angle += error_angle * self.dt

            P = self.goal_k_p * error_angle
            I = self.goal_k_i * sum_i_angle
            D = self.goal_k_d * (error_angle - prev_angle_error)

            move_cmd.angular.z = P + I + D 
            prev_angle_error = error_angle

            if abs(move_cmd.angular.z) < math.radians(30):
                move_cmd.linear.x = self.v
            else:
                move_cmd.linear.x = self.v/4

            self.cmd_vel.publish(move_cmd)

            msg_p = rospy.wait_for_message("/odom", Odometry)
            position_p = msg_p.pose.pose.position
            #current_angle_p = math.atan2(3 - position_p.y, (-1)-position_p.x)
            #goal_angle_p = (2*math.pi + current_angle_p)%(2*math.pi)
            distance_ref = math.sqrt( (3 - position_p.x)**2 + ((-1)-position_p.y)**2 )
            if distance_ref < 0.3:
                self.cmd_vel.publish(Twist())
                rospy.spin()
            self.r.sleep()

        self.cmd_vel.publish(Twist())

    def run(self):
        while not rospy.is_shutdown():
            
            # Follow Wall
            d = self.distance_from_wall()
            sum_i_theta = 0
            prev_theta_error = 0
            
            move_cmd = Twist()
            move_cmd.angular.z = 0
            move_cmd.linear.x = self.v

            while self.find_obstacle():
                error = d - self.D
                self.errors.append(error)
                sum_i_theta = error * self.dt

                P = self.k_p * error
                I = self.k_i * sum_i_theta
                D = self.k_d * (error - prev_theta_error)

                rospy.loginfo(f"P : {P} I : {I} D : {D}")
                move_cmd.angular.z = P + I + D 
                prev_theta_error = error
                if abs(move_cmd.angular.z) < math.radians(30):
                    move_cmd.linear.x = self.v
                else:
                    move_cmd.linear.x = self.v/4

                self.cmd_vel.publish(move_cmd)
                d = self.distance_from_wall()

                self.r.sleep()
            self.cmd_vel.publish(Twist())

            # Follow Goal
            self.follow_goal()
            

if __name__=='__main__':
    pidc = PIDController()
    pidc.run()