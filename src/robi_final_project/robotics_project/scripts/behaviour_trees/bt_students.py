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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

import numpy as np

class BehaviourTree(ptr.trees.BehaviourTree):

    def __init__(self):

        rospy.loginfo("Initialising behaviour tree")

        tuck_arm = TuckArm()

        mh_up = MoveHeadBehavior("up")

        pose = "pick"

        generate_particles = GenerateParticles()

        clear_costmaps = ClearCostmaps()

        r1_counter = counter(60, "Rotated?")
        rotate = pt.composites.Selector(
            name="Rotate",
            children=[r1_counter, Go("Rotate!", 0, 1)]
        )

        mtpickp = Navigation(pose)

        mh_down = MoveHeadBehavior("down")

        pick_cube = PickCube()

        mh_up2 = MoveHeadBehavior("up")

        r2_counter = counter(60, "Rotated?")
        rotate2 = pt.composites.Selector(
            name="Rotate",
            children=[r2_counter, Go("Rotate!", 0, 1)]
        )

        mtplacep = Navigation("place")

        place_cube = PlaceCube()

        mh_down2 = MoveHeadBehavior("down")

        detect_cube = DetectCube()

        reset_from_2 = ResetBehavior([
            tuck_arm,
            mh_up,
            # generate_particles,
            # clear_costmaps,
            r1_counter,
            mtpickp,
            mh_down,
            pick_cube,
            mh_up2,
            r2_counter,
            mtplacep,
            mh_down2,
            place_cube,
            detect_cube])

        conditional_reset = pt.composites.Selector(name="Coditional Reset", children=[detect_cube, reset_from_2])

        kidnap_check = NotKidnapped()

        end_behavior = EndBehavior()

        tree = RSequence(name="Main sequence", children=[
            kidnap_check,
            tuck_arm,
            mh_up,
            generate_particles,
            clear_costmaps,
            rotate,
            mtpickp,
            mh_down,
            pick_cube,
            mh_up2,
            rotate2,
            mtplacep,
            mh_down2,
            place_cube,
            conditional_reset])
        #tree = RSequence(name="Main sequence", children=[b0, btest])
        super(BehaviourTree, self).__init__(tree)

        # execute the behaviour tree
        rospy.sleep(5)
        self.setup(timeout=10000)
        while not rospy.is_shutdown(): self.tick_tock(1)    

# Behaviours

class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.reset()
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)
    
    def reset(self):
        self.i = 0

    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class Go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        self.linear = linear
        self.angular = angular

        # become a behaviour
        super(Go, self).__init__(name)

    def setup(self, timeout):
        # action space
        self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')

        # init pub
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = self.linear
        self.move_msg.angular.z = self.angular
        return super(Go, self).setup(timeout)

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
        self.reset()

        # become a behaviour
        super(TuckArm, self).__init__("Tuck arm!")
    
    def reset(self):
        self.sent_goal = False
        self.finished = False

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


class TuckArm2(pt.behaviour.Behaviour):

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
        self.reset()

        # become a behaviour
        super(TuckArm2, self).__init__("Tuck arm!")
    
    def reset(self):
        self.sent_goal = False
        self.finished = False

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

        # if still trying
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS


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
        self.reset()

        # become a behaviour
        super(PickCube, self).__init__("Pick cube!")

    def reset(self):
        self.tried = False
        self.done = False

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

        # if still trying - should never be the case
        else:
            raise Exception("COWABUNGA")
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
        self.reset()

        # become a behaviour
        super(PlaceCube, self).__init__("Place cube!")

    def reset(self):
        self.tried = False
        self.done = False

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

        # if still trying - should never be the case
        else:
            raise Exception("COWABUNGA")
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
        rospy.loginfo("detect callback")
        self.detected = True
    
    def setup(self, timeout):
        # Get rosparams
        self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/marker_pose_topic')
        self.reset()
        
        return super(DetectCube, self).setup(timeout)

    def reset(self):
        self.done = False
        self.detection_outcome = False
    
    def update(self):
        if self.done:
            if self.detection_outcome:
                return pt.common.Status.SUCCESS
            else :
                return pt.common.Status.FAILURE

        self.detected = False
        self.aruco_pose_sub = rospy.Subscriber(self.aruco_pose_top, PoseStamped, self.detect_cb)
        rospy.sleep(1)
        self.aruco_pose_sub.unregister()
        self.done = True
        if self.detected:
            self.detection_outcome = True
            rospy.loginfo("%s: Detected!", self.node_name)
            return pt.common.Status.SUCCESS

        else :
            rospy.loginfo("%s: Not detected!", self.node_name)
            return pt.common.Status.FAILURE

class Navigation(pt.behaviour.Behaviour):

    """
    TODO
    """

    def __init__(self, to="pick"):

        rospy.loginfo("Initialising Navigation to %s.", to)

        PLACE_POSE_TOPIC_PARAM = '/place_pose_topic'
        PICK_POSE_TOPIC_PARAM = '/pick_pose_topic'
        self.mapping = {"place": PLACE_POSE_TOPIC_PARAM, "pick": PICK_POSE_TOPIC_PARAM}
        assert to in self.mapping

        self.to = to
        self.param_name = self.mapping[to]
        
        self.node_name = "BT Student"

        # become a behaviour
        super(Navigation, self).__init__("Place cube!")
    
    def setup(self, timeout):
        # Get rosparams
        self.pose_top = rospy.get_param(rospy.get_name() + self.param_name)

        # Set up action servers
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)

        # Set up subscriber
        self.pose_subs = rospy.Subscriber(self.pose_top, PoseStamped, self.pose_cb)

        # execution
        self.reset()

        return super(Navigation, self).setup(timeout)

    def reset(self):
        self.done = False
        self.started = False
        self.pose_rcv = False

    def pose_cb(self, pose_msg):
        self.pose = pose_msg
        self.pose_rcv = True

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        
        if not self.started:
            if not rospy.is_shutdown() and not self.pose_rcv:
                rospy.loginfo("%s: Waiting for the %s pose in movetoplacepose behaviour", self.node_name, self.to)
                return pt.common.Status.RUNNING

            mbgoal = MoveBaseGoal(target_pose=self.pose)
            
            rospy.loginfo("Sending %s pose goal...", self.to)
            self.move_base_ac.send_goal(mbgoal)
            self.started = True
            return pt.common.Status.RUNNING
        
        if not self.move_base_ac.get_result():
            return pt.common.Status.RUNNING
        
        # self.move_base_ac.wait_for_result()
        # if self.move_base_ac.

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
        self.reset()

        # become a behaviour
        super(MoveHeadBehavior, self).__init__("Move head!")

    def reset(self):
        self.tried = False
        self.done = False

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

class GenerateParticles(pt.behaviour.Behaviour):

    """
    Updates measurements
    """

    def __init__(self):

        rospy.loginfo("Initialising generate particles behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(GenerateParticles, self).__init__("Generate particles!")

    def setup(self, timeout):
        # Get rosparams
        self.particle_service_name = rospy.get_param(rospy.get_name() + '/global_loc_srv')

        # Wait for servers
        rospy.wait_for_service(self.particle_service_name, timeout=30)
        self.particle_service = rospy.ServiceProxy(self.particle_service_name, Empty)

        # execution checker
        self.reset()
        
        return super(GenerateParticles, self).setup(timeout)

    def reset(self):
        self.done = False

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        # rospy.loginfo("  %s [GenerateParticles::initialise()]" % self.name)
        return super(GenerateParticles, self).initialise()
    
    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        
        # try if not tried
        elif not self.done:

            # command
            self.particle_service()
            self.done = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

class ClearCostmaps(pt.behaviour.Behaviour):

    """
    Updates measurements
    """

    def __init__(self):

        rospy.loginfo("Initialising clear costmaps behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(ClearCostmaps, self).__init__("Clear Costmaps!")

    def setup(self, timeout):
        # Get rosparams
        self.clear_costmap_srv_nm = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')

        # Wait for services
        rospy.wait_for_service(self.clear_costmap_srv_nm, timeout=30)
        self.clear_costmap = rospy.ServiceProxy(self.clear_costmap_srv_nm, Empty)

        # execution checker
        self.reset()
        
        return super(ClearCostmaps, self).setup(timeout)

    def reset(self):
        self.done = False

    def initialise(self):
        return super(ClearCostmaps, self).initialise()
    
    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        
        # try if not tried
        elif not self.done:

            # command
            self.clear_costmap()
            self.done = True

            # tell the tree you're running
            return pt.common.Status.RUNNING


class ResetBehavior(pt.behaviour.Behaviour):
    """
    Reset
    """

    def __init__(self, behaviors):

        rospy.loginfo("Initialising ResetBehavior behaviour.")
        
        self.node_name = "BT Student"
        self.behaviors = behaviors

        # become a behaviour
        super(ResetBehavior, self).__init__("ResetBehavior!")

    def setup(self, timeout):
        # execution checker
        self.reset()
        
        return super(ResetBehavior, self).setup(timeout)

    def reset(self):
        pass

    def initialise(self):
        return super(ResetBehavior, self).initialise()
    
    def update(self):
        print("resetting")
        for behavior in self.behaviors:
            behavior.reset()

        # tell the tree you're running
        return pt.common.Status.RUNNING

class NotKidnapped(pt.behaviour.Behaviour):
    """
    Reset
    """

    def __init__(self):

        rospy.loginfo("Initialising NotKidnapped behaviour.")
        
        self.node_name = "BT Student"

        # become a behaviour
        super(NotKidnapped, self).__init__("NotKidnapped!")

    def setup(self, timeout):
        # Get rosparams
        self.estimate_top = rospy.get_param(rospy.get_name() + "/amcl_estimate")

        # Set up subscriber
        self.estimate_subs = rospy.Subscriber(self.estimate_top, PoseWithCovarianceStamped, self.pose_cb)
        self.pose_rcv = False

        return super(NotKidnapped, self).setup(timeout)

    def pose_cb(self, pose_msg):
        self.pose = pose_msg
        self.pose_rcv = True
        covariance = self.pose.pose.covariance
        rospy.loginfo("\n%s",np.array(covariance).reshape(6,6))

    def reset(self):
        pass

    def initialise(self):
        return super(NotKidnapped, self).initialise()
    
    def update(self):
        # index 1 = uncertainty in x
        # index 7 = uncertainty in y
        # final index = uncertainty in z-rotation
        low_cov = (0.0014848754549490195, -7.93474742977196e-06, 0.0, 0.0, 0.0, 0.0, -7.934747433324674e-06, 0.015494525441120288, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.004123908494110791)
        high_cov = (0.039370549659337506, -0.010388675767197597, 0.0, 0.0, 0.0, 0.0, -0.010388675767197597, 0.021546009985087267, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01857535537232525)
        if not self.pose_rcv:
            return pt.common.Status.SUCCESS

        else:
            
            return pt.common.Status.SUCCESS

        


if __name__ == "__main__":

    rospy.init_node('main_state_machine')
    try:
        BehaviourTree()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

