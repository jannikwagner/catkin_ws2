#!/usr/bin/env python

import math
import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

# Added by us
from moveit_msgs.msg import PickupActionGoal, PickupAction, PlaceAction
from play_motion_msgs.msg import PlayMotionActionGoal

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


def callback_pickup(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)



class StateMachine(object):
    def __init__(self):
        
        self.node_name = "Student SM"
        self.manipulation_client_name = "manipulation_client"

        # Access rosparams
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        rospy.loginfo("%s: pick_srv_nm: %s", self.node_name, self.pick_srv_nm)
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
        self.mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')

        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.pickup_pose_top = rospy.get_param(self.manipulation_client_name + '/pickup_marker_pose')  # TODO replace with rospy.get_name() and adjust launch
        self.place_pose_top = rospy.get_param(self.manipulation_client_name + '/place_marker_pose')
        self.aruco_pose_top = rospy.get_param(self.manipulation_client_name + '/marker_pose_topic') # Publish a PoseStamped with the cube pose here so that we can pick in manip_client (Line 160)

        self.cube_pose_str = rospy.get_param(rospy.get_name() + '/cube_pose')
        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')
        self.cube_pose =  self.extract_cube_pose(self.cube_pose_str, self.robot_base_frame)
        rospy.loginfo("%s: cube_pose: %s", self.node_name, self.cube_pose_str)
        # cube pose = "0.50306828716, 0.0245718046511, 0.915538062216, 0.0144467629456, 0.706141958739, 0.707257659069, -0.0306827123383"
        rospy.loginfo("%s: cube_pose: %s", self.node_name, self.cube_pose)

        # Subscribe to topics
        # rospy.Subscriber("/pickup/goal", PickupActionGoal, callback_pickup)

        # Wait for service providers
        rospy.wait_for_service(self.mv_head_srv_nm, timeout=30)
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        rospy.wait_for_service(self.place_srv_nm, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=1)

        # Set up action clients
        # rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        # self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        # if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
        #     rospy.logerr("%s: Could not connect to play_motion action server", self.node_name)
        #     exit()
        # rospy.loginfo("%s: Connected to play_motion action server", self.node_name)

        # rospy.loginfo("%s: Waiting for pickup action server...", self.node_name)
        # self.pickup_ac = SimpleActionClient("/pickup", PickupAction)
        # if not self.pickup_ac.wait_for_server(rospy.Duration(1000)):
        #     rospy.logerr("%s: Could not connect to pickup action server", self.node_name)
        #     exit()
        # rospy.loginfo("%s: Connected to pickup action server", self.node_name)

        # rospy.loginfo("%s: Waiting for place action server...", self.node_name)
        # self.place_ac = SimpleActionClient("/place", PlaceAction)
        # if not self.place_ac.wait_for_server(rospy.Duration(1000)):
        #     rospy.logerr("%s: Could not connect to place action server", self.node_name)
        #     exit()
        # rospy.loginfo("%s: Connected to place action server", self.node_name)

        # Init state machine
        self.state = 0
        rospy.sleep(3)
        self.check_states()

    def check_states(self):
        ERROR_STATE = "error"
        SUCCESS_STATE = "success"

        while not rospy.is_shutdown() and self.state != SUCCESS_STATE:
            rospy.loginfo("%s: check_states while loop", self.node_name)
            
            # State 0:  Tuck arm 
            if self.state == 0:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

                if success_tucking:
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    self.state = 1
                else:
                    self.play_motion_ac.cancel_goal()
                    rospy.logerr("%s: play_motion failed to tuck arm, reset simulation", self.node_name)
                    self.state = ERROR_STATE

                rospy.sleep(1)

            # State 1: Pick up cube
            if self.state == 1:

                self.aruco_pose_pub.publish(self.cube_pose) # Publish the aruco pose for manip client to receive


                rospy.loginfo("%s: Pick up goal", self.node_name)
                pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
                
                request = SetBoolRequest(True)
                pick_srv_req = pick_srv(request)

                
                if pick_srv_req.success == True:
                    self.state = 2
                    rospy.loginfo("%s: Pickup succeeded!", self.node_name)
                else:
                    rospy.loginfo("%s: Pickup failed!", self.node_name)
                    self.state = ERROR_STATE

                rospy.sleep(3)


            # State 2: Rotate and move the robot "manually"
            if self.state == 2:
                move_msg = Twist()
                rate_val = 10
                rate = rospy.Rate(rate_val)

                # Move straight to other table
                move_msg.angular.z = 0
                move_msg.linear.x = -1
                cnt = 0
                rospy.loginfo("%s: Moving towards door", self.node_name)
                while not rospy.is_shutdown() and cnt < 50:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                # rotate
                move_msg.angular.z = 0.5
                move_msg.linear.x = 0
                rotation_steps = int(2 * math.pi * rate_val)
                cnt = 0
                rospy.loginfo("%s: Rotating", self.node_name)
                while not rospy.is_shutdown() and cnt < rotation_steps:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 3
                rospy.sleep(1)

            # State 3: Place cube
            if self.state == 3:

                # self.aruco_pose_pub.publish(self.cube_pose) # Publish the aruco pose for manip client to receive

                rospy.loginfo("%s: Place goal", self.node_name)
                place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
                
                request = SetBoolRequest(True)
                place_srv_req = place_srv(request)

                
                if place_srv_req.success == True:
                    self.state = SUCCESS_STATE
                    rospy.loginfo("%s: Place succeeded!", self.node_name)
                else:
                    rospy.loginfo("%s: Place failed!", self.node_name)
                    self.state = ERROR_STATE

                rospy.sleep(3)


            # State 2:  Move the robot "manually" to chair
            if self.state == -1:
                move_msg = Twist()
                move_msg.angular.z = -1

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards table", self.node_name)
                while not rospy.is_shutdown() and cnt < 5:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                move_msg.linear.x = 1
                move_msg.angular.z = 0
                cnt = 0
                while not rospy.is_shutdown() and cnt < 15:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                self.state = 3
                rospy.sleep(1)

            # State 3:  Lower robot head service
            if self.state == -1:
            	try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.mv_head_srv_nm, MoveHead)
                    move_head_req = move_head_srv("down")
                    
                    if move_head_req.success == True:
                        self.state = 4
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        self.state = 5

                    rospy.sleep(3)
                
                except rospy.ServiceException, e:
                    print "Service call to move_head server failed: %s"%e

            # Error handling
            if self.state == ERROR_STATE:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

    def extract_cube_pose(self, cube_pose_string, frame):
        px, py, pz, rx, ry, rz, rw = (float(val) for val in cube_pose_string.split(", "))
        cube_pose = PoseStamped()
        cube_pose.header.frame_id = frame
        cube_pose.pose.position.x = px
        cube_pose.pose.position.y = py
        cube_pose.pose.position.z = pz
        cube_pose.pose.orientation.x = rx
        cube_pose.pose.orientation.y = ry
        cube_pose.pose.orientation.z = rz
        cube_pose.pose.orientation.w = rw
        return cube_pose
	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		#StateMachine()
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
