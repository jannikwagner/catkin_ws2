# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

# Added by us
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped

class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS

class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING

def extract_cube_pose(cube_pose_string, frame):
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

class pickcube(pt.behaviour.Behaviour):

    """
    Picks up cube from known position
    Locks thread while service executed
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising pick behaviour.")
        
        self.node_name = "BT Student"

        # Get rosparams
        self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/marker_pose_topic')
        self.robot_base_frame = rospy.get_param(rospy.get_name() + '/robot_base_frame')

        # Once we detect cube, this should be adjusted to get the position from the detection
        self.cube_pose_str = rospy.get_param(rospy.get_name() + '/cube_pose')
        self.cube_pose =  extract_cube_pose(self.cube_pose_str, self.robot_base_frame)
        
        #rospy.loginfo("%s: cube_pose: %s", self.node_name, self.cube_pose_str)
        # cube pose = "0.50306828716, 0.0245718046511, 0.915538062216, 0.0144467629456, 0.706141958739, 0.707257659069, -0.0306827123383"
        #rospy.loginfo("%s: cube_pose: %s", self.node_name, self.cube_pose)

        # Wait for servers
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

        #Initialise publisher
        
        # This should be done by the detection system
        #self.aruco_pose_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=1)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(pickcube, self).__init__("Pick cube!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # This should be done by the detection system
            #self.aruco_pose_pub.publish(self.cube_pose) 

            # command
            request = SetBoolRequest(True)
            rospy.loginfo("%s: Sending request to server", self.node_name)
            self.pick_srv_req = self.pick_srv(request)
            rospy.loginfo("%s: Request completed", self.node_name)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.pick_srv_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.pick_srv_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class placecube(pt.behaviour.Behaviour):

    """
    Places cube
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising placement behaviour.")
        
        self.node_name = "BT Student"

        # Get rosparams
        self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')


        # Wait for servers
        rospy.wait_for_service(self.place_srv_nm, timeout=30)
        self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(placecube, self).__init__("Place cube!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:
            rospy.loginfo("%s: Place goal", self.node_name)
            request = SetBoolRequest(True)
            self.place_srv_req = self.place_srv(request)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.place_srv_req.success:
            rospy.loginfo("%s: Place succeeded!", self.node_name)
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.place_srv_req.success:
            rospy.loginfo("%s: Place failed!", self.node_name)

            # #Try again if failed?
            # rospy.loginfo("%s: Resetting tried to try placing again!", self.node_name)
            # self.tried = False

            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class detectcube(pt.behaviour.Behaviour):

    """
    Detects cube
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """
    def aruco_pose_cb(self, aruco_pose_msg):
        self.aruco_pose_updated = True

    def __init__(self):

        rospy.loginfo("Initialising detection behaviour.")
        
        self.node_name = "BT Student"

        # Get rosparams
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/marker_pose_topic')
        
        self.place_pose_updated = False
        self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.aruco_pose_cb)


        # execution checker
        self.done = False

        # become a behaviour
        super(detectcube, self).__init__("Detect cube!")

    def update(self):
        
        if self.done:
            return pt.common.Status.SUCCESS

        self.place_pose_updated = False
        rospy.loginfo("Waiting 5 seconds to see if the cube is detected")
        rospy.sleep(5)
        
        if self.place_pose_updated:
            self.done = True
            return pt.common.Status.SUCCESS
        
        return pt.common.Status.FAILURE

        



# class movetopickpose(pt.behaviour.Behaviour):

class movetoplacepose(pt.behaviour.Behaviour):

    """
    Dynamically travels to the published place pose
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """
    def place_pose_cb(self, place_pose_msg):
        self.place_pose = place_pose_msg
        self.place_pose_rcv = True

    def __init__(self):

        rospy.loginfo("Initialising place pose movement behaviour.")

        self.node_name = "BT Student"

        # Get rosparams
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        # Set up action servers


        # Set up subscriber
        self.place_pose_rcv = False
        self.place_pose_subs = rospy.Subscriber(self.place_pose_top, PoseStamped, self.place_pose_cb)


        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movetoplacepose, self).__init__("Move to place pose!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            while not rospy.is_shutdown() and self.place_pose_rcv == False:
                rospy.loginfo("%s: Waiting for the place pose in movetoplacepose behaviour", self.node_name)
                rospy.sleep(1.0)
            
            # FIGURE OUT HOW TO MOVE TO self.place_pose
            #rospy.loginfo("WE HAVE ACQUIRED THE PLACE POSE")

            self.movement_success = True

            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.movement_success:
            rospy.loginfo("%s: Detection succeeded!", self.node_name)
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.movement_success:
            rospy.loginfo("%s: Detection failed!", self.node_name)

            # #Try again if failed?
            # rospy.loginfo("%s: Resetting tried to try placing again!", self.node_name)
            # self.tried = False

            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            rospy.loginfo("Moving head down")
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING