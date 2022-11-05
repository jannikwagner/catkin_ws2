#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from reactive_sequence import RSequence
import sys
import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

# Added by us
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        # # go to door until at door
        # b0 = pt.composites.Selector(
        #   name="Go to door fallback", 
        #   children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
        # )
        # # tuck the arm
        # b1 = tuckarm()

        # # go to table
        # b2 = pt.composites.Selector(
        #   name="Go to table fallback",
        #   children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
        # )

        # # move to chair
        # b3 = pt.composites.Selector(
        #   name="Go to chair fallback",
        #   children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
        # )

        # # lower head
        # b4 = movehead("down")

        # # become the tree
        # tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
        
        tuck_arm = TuckArm()

        mh_down = MoveHeadBehavior("down")
        mh_up = MoveHeadBehavior("up")

        localize = Localize()

        rotate = pt.composites.Selector(
            name="Rotate",
            children=[counter(60, "Rotated?"), go("Rotate!", 0, 1)]
        )

        navigate = Navigation()

        mtpickp = MoveToPickPose()

        pick_cube = PickCube()

        # b3 = pt.composites.Selector(
        #     name="Back up to table",
        #     children=[counter(40, "At table?"), go("Back up to table!", -1, 0)]
        # )

        # b4 = pt.composites.Selector(
        #     name="Turn towards table",
        #     children=[counter(30, "Facing table?"), go("Turn towards table!", 0, 1)]
        # )

        mtplacep = MoveToPlacePose()

        place_cube = PlaceCube()

        detect_cube = DetectCube()

        go_back = pt.composites.Selector(
            name="Back up to table",
            children=[counter(40, "At table?"), go("Back up to table!", -1, 0)]
        )

        rotate_back = pt.composites.Selector(
            name="Turn towards table",
            children=[counter(30, "Facing table?"), go("Turn towards table!", 0, 1)]
        )

        back_to_table = RSequence(name="Move back", children=[go_back, rotate_back])

        conditional_reset = pt.composites.Selector(name="Coditional Reset", children=[detect_cube, back_to_table])

        end_behavior = EndBehavior()

        tree = RSequence(name="Main sequence", children=[
            tuck_arm,
            mh_up,
            localize,
            rotate,
            navigate,
            mtpickp,
            mh_down,
            pick_cube,
            mtplacep,
            place_cube])
        #tree = RSequence(name="Main sequence", children=[b0, btest])
        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)    

# Behaviours

class map_init(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self):

        rospy.loginfo("Initialising map_init behaviour.")

        # /static_map gives OccupancyGrid (supplied by /map_server node)
        # /set_map takes an OccupancyGrid (passed to /amcl node)

        
        # become a behaviour
        super(map_init, self).__init__("map_init")


    def update(self):
        pass


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
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        # self.cmd_vel_top = "/key_vel"
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

class TuckArm(pt.behaviour.Behaviour):

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
        super(TuckArm, self).__init__("Tuck arm!")

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

class PickCube(pt.behaviour.Behaviour):

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

        # Wait for servers
        rospy.wait_for_service(self.pick_srv_nm, timeout=30)
        self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(PickCube, self).__init__("Pick cube!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

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

class PlaceCube(pt.behaviour.Behaviour):

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
        super(PlaceCube, self).__init__("Place cube!")

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

            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class DetectCube(pt.behaviour.Behaviour):

    """
    Detects cube (and should likely publish the position somewhere for pick to acquire)
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising detection behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(DetectCube, self).__init__("Detect cube!")

    def detect_cb(self, *args):
        # if position is published, it is detected
        # rospy.loginfo("detect callback")
        self.detected = True
    
    def setup(self, timeout):
        # Get rosparams
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/marker_pose_topic')
        self.done = False
        
        return super(DetectCube, self).setup(timeout)
    
    def update(self):
        if self.done:
            if self.detected:
                return pt.common.Status.SUCCESS
            else :
                return pt.common.Status.FAILURE

        self.detected = False
        self.aruco_pose_sub = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.detect_cb)
        rospy.sleep(1)
        self.aruco_pose_sub.unregister()
        self.done = True
        if self.detected:
            rospy.loginfo("%s: Detected!", self.node_name)
            return pt.common.Status.SUCCESS

        else :
            rospy.loginfo("%s: Not detected!", self.node_name)
            return pt.common.Status.FAILURE

class MoveToPlacePose(pt.behaviour.Behaviour):

    """
    TODO
    """

    def __init__(self):

        rospy.loginfo("Initialising place pose movement behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(MoveToPlacePose, self).__init__("Place cube!")
    
    def setup(self, timeout):
        # Get rosparams
        # Get rosparams
        self.place_pose_top = rospy.get_param(rospy.get_name() + '/place_pose_topic')

        # Set up action servers
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # Set up subscriber
        self.place_pose_rcv = False
        self.place_pose_subs = rospy.Subscriber(self.place_pose_top, PoseStamped, self.place_pose_cb)
        self.done = False
        
        return super(MoveToPlacePose, self).setup(timeout)

    def place_pose_cb(self, place_pose_msg):
        self.place_pose = place_pose_msg
        self.place_pose_rcv = True

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS

        while not rospy.is_shutdown() and self.place_pose_rcv == False:
            rospy.loginfo("%s: Waiting for the place pose in movetoplacepose behaviour", self.node_name)
            rospy.sleep(1.0)

        mbgoal = MoveBaseGoal(target_pose=self.place_pose)
        
        rospy.loginfo("Sending place pose goal...")
        self.move_base_ac.send_goal(mbgoal)
        self.move_base_ac.wait_for_result()
        #if self.move_base_ac.
        rospy.loginfo("Done sending place pose goal!")
        self.done = True

        return pt.common.Status.SUCCESS

class MoveToPickPose(pt.behaviour.Behaviour):

    """
    TODO
    """

    def __init__(self):

        rospy.loginfo("Initialising place pose movement behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(MoveToPickPose, self).__init__("Place cube!")
    
    def setup(self, timeout):
        # Get rosparams
        # Get rosparams
        self.pick_pose_top = rospy.get_param(rospy.get_name() + '/pick_pose_topic')

        # Set up action servers
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # Set up subscriber
        self.pick_pose_rcv = False
        self.pick_pose_subs = rospy.Subscriber(self.pick_pose_top, PoseStamped, self.pick_pose_cb)
        self.done = False
        
        return super(MoveToPickPose, self).setup(timeout)

    def pick_pose_cb(self, pick_pose_msg):
        self.pick_pose = pick_pose_msg
        self.pick_pose_rcv = True

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS

        while not rospy.is_shutdown() and self.pick_pose_rcv == False:
            rospy.loginfo("%s: Waiting for the place pose in movetoplacepose behaviour", self.node_name)
            rospy.sleep(1.0)
    
        mbgoal = MoveBaseGoal(target_pose=self.pick_pose)
        
        rospy.loginfo("Sending place pose goal...")
        self.move_base_ac.send_goal(mbgoal)
        self.move_base_ac.wait_for_result()
        #if self.move_base_ac.
        rospy.loginfo("Done sending place pose goal!")
        self.done = True

        return pt.common.Status.SUCCESS



class MoveHeadBehavior(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
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
        super(MoveHeadBehavior, self).__init__("Move head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class EndBehavior(pt.behaviour.Behaviour):

    def __init__(self):

        rospy.loginfo("Initialising EndBehavior.")

        super(EndBehavior, self).__init__("EndBehavior")

    def update(self):

        rospy.loginfo("This is the end.")
        sys.exit()
        return pt.common.Status.RUNNING

class Localize(pt.behaviour.Behaviour):

    """
    Updates measurements
    """

    def __init__(self):

        rospy.loginfo("Initialising localization behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(Localize, self).__init__("Localize!")

    def setup(self, timeout):
        # Get rosparams
        self.service_name = rospy.get_param(rospy.get_name() + '/global_loc_srv')

        # Wait for servers
        rospy.wait_for_service(self.service_name, timeout=30)
        self.service = rospy.ServiceProxy(self.service_name, Empty)

        # execution checker
        self.done = False
        
        return super(Localize, self).setup(timeout)

    def initialise(self):
        return super(Localize, self).initialise()
    
    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        
        # try if not tried
        elif not self.done:

            # command
            self.req = self.service()
            self.done = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

class Navigation(pt.behaviour.Behaviour):

    """
    TODO
    """

    def __init__(self):

        rospy.loginfo("Initialising Navigation behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(Navigation, self).__init__("Navigate!")

    def setup(self, timeout):
        # Get rosparams
        self.service_name = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')

        # Wait for servers
        rospy.wait_for_service(self.service_name, timeout=30)
        self.service = rospy.ServiceProxy(self.service_name, Empty)

        # execution checker
        self.done = False
        
        return super(Navigation, self).setup(timeout)

    def initialise(self):
        return super(Navigation, self).initialise()
    
    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        
        # try if not tried
        elif not self.done:

            # command
            self.req = self.service()
            self.done = True

            # tell the tree you're running
            return pt.common.Status.RUNNING


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

