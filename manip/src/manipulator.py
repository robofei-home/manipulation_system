#!/usr/bin/env python

import sys
import copy
import math

import rospy
import tf
from tf import transformations

import moveit_commander 
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory

from manip3.srv import Manip3

class Manipulator:
    LEFT_GRIP_OPENED = 3844
    LEFT_GRIP_CLOSED = 3302
    RIGHT_GRIP_OPENED = 265
    RIGHT_GRIP_CLOSED = 802

    __last_joint_state = None

    def __init__(self):

        self.__joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.__joint_state_cb, queue_size=1)

        self.group = moveit_commander.MoveGroupCommander('hera_arm')
        self.hand = moveit_commander.MoveGroupCommander('gripper')

        # self.group.set_planning_time(10)
        self.group.set_pose_reference_frame('base_link')
        # self.hand.set_planning_time(10)
        self.hand.set_pose_reference_frame('base_link')

        self.__hand_traj_client = SimpleActionClient("/manip/gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.__arm_traj_client = SimpleActionClient("/manip/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)

        if self.__hand_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
            rospy.logfatal("Failed to connect to /gripper_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /gripper_controller/follow_joint_trajectory in 4sec.")
                                              
        if self.__arm_traj_client.wait_for_server(timeout=rospy.Duration(4.0)) is False:
            rospy.logfatal("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")
            raise Exception("Failed to connect to /arm_controller/follow_joint_trajectory in 4sec.")

                   

        self.joint_trajectory = rospy.Publisher('/manip/joint_trajectory', JointTrajectory, queue_size=10)
        self.update_start_state = rospy.Publisher('/rviz/moveit/update_start_state', Empty, queue_size=10)

        rospy.wait_for_service('/get_planning_scene', 10.0)
        self.__get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.__pub_planning_scene = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10, latch=True)

        # rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.display_planned_path_callback)
        # rospy.Subscriber('/manip/feedback', Empty, self.feedback_callback)

        rospy.Service('manipulator', Manip3, self.handler)

        self.tf = tf.TransformListener()
        self.tf.waitForTransform('base_link', 'base', rospy.Time(), rospy.Duration(1.0))                                       
                                              

        self.is_moving = False
        self.plan = None

        rospy.loginfo('[manip] Going Home in 5 seconds...')
        # rospy.sleep(5)
        # self.close_gripper()
        # rospy.sleep(5)
        # rospy.loginfo('[manip] Abrindo ...')
        # self.open_gripper()
        # rospy.sleep(5)
        # self.home()
        # rospy.loginfo('Era para ter ido')

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
        rospy.loginfo('[manip] Cartesian path fraction: %.2f.' % fraction)
        rospy.sleep(2)
        if self.plan != None and fraction > 0.9:
            self.group.execute(plan, wait=True)
           
            rospy.sleep(5)
            self.plan = None
            return True
        else:
            self.plan = None
            return False

    def advance(self, waypoints, avoid_col = False):
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions = avoid_col)
        rospy.loginfo('[manip] Cartesian path fraction: %.2f.' % fraction)
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
    def __joint_state_cb(self, msg):
        self.__last_joint_state = msg


    def reset_manipulator(self):
        self.group.set_named_target('home')
        plan = self.group.plan()
        if not self.group.execute(plan, wait=True):
            return False
        self.open_gripper()
        return success

    def home(self):
        self.group.set_named_target('home')
        plan = self.group.plan()
        if not self.group.execute(plan, wait=True):
            return False
        return True

    def open_gripper(self):
        self.hand.set_named_target('open_gripper')
        plan = self.hand.plan()
        if not self.hand.execute(plan, wait=True):
            return False
        return True

    def close_gripper(self):
        self.hand.set_named_target('close_gripper')
        plan = self.hand.plan()
        success = self.hand.execute(plan, wait=True)
        return success

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
            
            target_pose = copy.deepcopy(pose)
            self.group.set_pose_target(target_pose)
            plan = self.group.plan()
            success = self.group.execute(plan, wait=True)
            # target_pose = self.tf.transformPose('base_link', self.group.get_current_pose()).pose
            
            # success = self.execute_cartesian_plan([target_pose])
                
           
            rospy.sleep(5)
            self.close_gripper()
                
            
            rospy.sleep(10)
            self.home() 

            return 'SUCCEEDED' if success else 'FAILED'    
            
        # elif type == 'place':
        #     target_pose = copy.deepcopy(pose)
        #     target_pose.position.z += 0.15
        #     self.group.set_pose_target(target_pose)
        #     self.execute_plan()
        #     target_pose = self.tf.transformPose('manip_link', self.group.get_current_pose()).pose
        #     target_pose.position.z -= 0.15
        #     success = self.execute_cartesian_plan([target_pose])
        #     if not success:
        #         self.group.set_pose_target(pose)
        #         success = self.execute_plan()
        #     if success:
        #         self.open_gripper()
        #         target_pose = self.tf.transformPose('manip_link', self.group.get_current_pose()).pose
        #         target_pose.position.x -= 0.09 * math.cos(goal.rz)
        #         target_pose.position.y -= 0.09 * math.sin(goal.rz)
        #         success = self.execute_cartesian_plan([target_pose])
        #         if not success:
        #             target_pose = self.tf.transformPose('manip_link', self.group.get_current_pose()).pose
        #             target_pose.position.z += 0.1
        #             success = self.execute_cartesian_plan([target_pose])
        #         self.group.set_named_target('home')
        #         success = self.execute_plan()
        
        # elif type == 'point':
        #     self.close_gripper()
        #     angle = pose.position.x
        #     joint_goal = self.group.get_current_joint_values()
        #     joint_goal[0] = angle
        #     joint_goal[1] = 0.0
        #     joint_goal[2] = 0.0
        #     joint_goal[3] = -2.35
        #     joint_goal[4] = 0.0
        #     joint_goal[5] = -0.35
        #     self.group.set_joint_value_target(joint_goal)
        #     success = self.execute_plan()
        # else:
        #     self.group.set_pose_target(pose)
        #     success = self.execute_plan()
        #
        # return 'SUCCEEDED' if success else 'FAILED'


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator', log_level=rospy.INFO)
    Manipulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
