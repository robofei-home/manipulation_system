#!/usr/bin/env python

# Author: Lucas Lervolino Gazignato

import sys
import copy
import math

import rospy
import tf
from tf import transformations

import moveit_commander

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory

from dynamixel_workbench_msgs.srv import DynamixelCommand

from manip3.srv import Manip3

class Manipulator:
    LEFT_GRIP_OPENED = 3844
    LEFT_GRIP_CLOSED = 3302
    RIGHT_GRIP_OPENED = 265
    RIGHT_GRIP_CLOSED = 802

    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(10)
        self.group.set_pose_reference_frame('manip_link')

        self.joint_trajectory = rospy.Publisher('/manip3/joint_trajectory', JointTrajectory, queue_size=10)
        self.update_start_state = rospy.Publisher('/rviz/moveit/update_start_state', Empty, queue_size=10)

        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.display_planned_path_callback)
        rospy.Subscriber('/manip3/feedback', Empty, self.feedback_callback)

        self.gripper = rospy.ServiceProxy('/manip3/dynamixel_command', DynamixelCommand)

        rospy.Service('manipulator', Manip3, self.handler)

        self.tf = tf.TransformListener()
        self.tf.waitForTransform('manip_link', 'base_link', rospy.Time(), rospy.Duration(1.0))

        self.is_moving = False
        self.plan = None

        rospy.loginfo('[manip3] Going Home in 5 seconds...')
        rospy.sleep(5)
        self.reset_manipulator()

    def display_planned_path_callback(self, data):
        self.plan = data.trajectory[0].joint_trajectory

    def execute(self):
        self.joint_trajectory.publish(self.plan)
        self.is_moving = True

    def feedback_callback(self, data):
        self.is_moving = False
        if self.update_start_state.get_num_connections() > 0:
            self.update_start_state.publish()

    def execute_plan(self):
        self.group.plan()
        rospy.sleep(2)
        if self.plan != None:
            self.execute()
            while self.is_moving:
                pass
            rospy.sleep(2)
            self.plan = None
            return True
        else:
            return False

    def execute_cartesian_plan(self, waypoints):
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions = True)
        rospy.loginfo('[manip3] Cartesian path fraction: %.2f.' % fraction)
        rospy.sleep(2)
        if self.plan != None and fraction > 0.9:
            self.execute()
            while self.is_moving:
                pass
            rospy.sleep(5)
            self.plan = None
            return True
        else:
            self.plan = None
            return False

    def advance(self, waypoints, avoid_col = False):
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions = avoid_col)
        rospy.loginfo('[manip3] Cartesian path fraction: %.2f.' % fraction)
        rospy.sleep(2)
        if self.plan != None and fraction > 0.9:
            self.execute()
            while self.is_moving:
                pass
            rospy.sleep(5)
            self.plan = None
            return True
        else:
            self.plan = None
            return False

    def reset_manipulator(self):
        self.group.set_named_target('home')
        success = self.execute_plan()
        self.open_gripper()
        return success

    def home(self):
        self.group.set_named_target('home')
        success = self.execute_plan()
        return success

    def open_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_OPENED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_OPENED)
        rospy.sleep(5)
        return True

    def close_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_CLOSED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_CLOSED)
        rospy.sleep(5)
        return True

    def handler(self, request):
        type = request.type.lower()
        goal = request.goal

        pose = Pose()
        quaternion = tf.transformations.quaternion_from_euler(goal.rx, goal.ry, goal.rz)
        pose.position.x = goal.x
        pose.position.y = goal.y
        pose.position.z = goal.z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        if type == 'reset':
            success = self.reset_manipulator()
        elif type == 'home':
            success = self.home()
        elif type == 'open':
            success = self.open_gripper()
        elif type == 'close':
            success = self.close_gripper()
        elif type == 'pick':
            self.open_gripper()
            target_pose = copy.deepcopy(pose)
            target_pose.position.x -= 0.07 * math.cos(goal.rz)
            target_pose.position.y -= 0.07 * math.sin(goal.rz)
            # target_pose.position.z += 0.1
            self.group.set_pose_target(target_pose)
            success = self.execute_plan()
            if success:
                target_pose = self.tf.transformPose('manip_link', self.group.get_current_pose()).pose
                target_pose.position.x += 0.07 * math.cos(goal.rz)
                target_pose.position.y += 0.07 * math.sin(goal.rz)
                # target_pose.position.z -= 0.1
                success = self.advance([target_pose])
                if success:
                    self.close_gripper()
                if not success:
                    self.group.set_named_target('home')
                    self.execute_plan()
                    self.open_gripper()
                    self.group.set_pose_target(pose)
                    success = self.execute_plan()
                    if success:
                        self.close_gripper()
            if not success:
                self.group.set_pose_target(pose)
                success = self.execute_plan()
                if success:
                    self.close_gripper()
            self.group.set_named_target('home')
            self.execute_plan()

        elif type == 'place':
            target_pose = copy.deepcopy(pose)
            target_pose.position.z += 0.15
            self.group.set_pose_target(target_pose)
            self.execute_plan()
            target_pose = self.tf.transformPose('manip_link', self.group.get_current_pose()).pose
            target_pose.position.z -= 0.15
            success = self.execute_cartesian_plan([target_pose])
            if not success:
                self.group.set_pose_target(pose)
                success = self.execute_plan()
            if success:
                self.open_gripper()
                target_pose = self.tf.transformPose('manip_link', self.group.get_current_pose()).pose
                target_pose.position.x -= 0.09 * math.cos(goal.rz)
                target_pose.position.y -= 0.09 * math.sin(goal.rz)
                success = self.execute_cartesian_plan([target_pose])
                if not success:
                    target_pose = self.tf.transformPose('manip_link', self.group.get_current_pose()).pose
                    target_pose.position.z += 0.1
                    success = self.execute_cartesian_plan([target_pose])
                self.group.set_named_target('home')
                success = self.execute_plan()

        elif type == 'point':
            self.close_gripper()
            angle = pose.position.x
            joint_goal = self.group.get_current_joint_values()
            joint_goal[0] = angle
            joint_goal[1] = 0.0
            joint_goal[2] = 0.0
            joint_goal[3] = -2.35
            joint_goal[4] = 0.0
            joint_goal[5] = -0.35
            self.group.set_joint_value_target(joint_goal)
            success = self.execute_plan()
        else:
            self.group.set_pose_target(pose)
            success = self.execute_plan()

        return 'SUCCEEDED' if success else 'FAILED'

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator', log_level=rospy.INFO)
    Manipulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
