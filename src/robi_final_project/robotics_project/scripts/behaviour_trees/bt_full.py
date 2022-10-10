import py_trees as pt, py_trees_ros as ptr

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# go to door until at door
		b0 = pt.composites.Selector(
			name="Go to door fallback", 
			children=[Counter(30, "At door?"), Go("Go to door!", 1, 0)]
		)

		# tuck the arm
		b1 = TuckArm()

		# go to table
		b2 = pt.composites.Selector(
			name="Go to table fallback",
			children=[Counter(5, "At table?"), Go("Go to table!", 0, -1)]
		)

		# move to chair
		b3 = pt.composites.Selector(
			name="Go to chair fallback",
			children=[Counter(13, "At chair?"), Go("Go to chair!", 1, 0)]
		)

		# lower head
		b4 = LowerHead()

		# become the tree
		tree = pt.composites.Sequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)


class Counter(pt.behaviour.Behaviour):

	def __init__(self, n, name):

		# counter
		self.i = 0
		self.n = n

		# become a behaviour
		super(Counter, self).__init__(name)

	def update(self):

		# count until n
		while self.i <= self.n:

			# increment count
			self.i += 1

			# return failure :(
			return pt.common.Status.FAILURE

		# succeed after counter done :)
		return pt.common.Status.SUCCESS


class Go(pt.behaviour.Behaviour):

	def __init__(self, name, linear, angular):

		# action space
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

		# command
		self.move_msg = Twist()
		self.move_msg.linear.x = linear
		self.move_msg.angular.z = angular

		# become a behaviour
		super(Go, self).__init__(name)

	def update(self):

		# send the message
		rate = rospy.Rate(10)
		self.cmd_vel_pub.publish(self.move_msg)
		rate.sleep()

		# tell the tree that you're running
		return pt.common.Status.RUNNING


class TuckArm(pt.behaviour.Behaviour):

	def __init__(self):

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

		# if I'm still trying :|
		else:
			return pt.common.Status.RUNNING
		


class LowerHead(pt.behaviour.Behaviour):

	def __init__(self):

		# server
		mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
		self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		rospy.wait_for_service(mv_head_srv_nm, timeout=30)

		# execution checker
		self.tried = False
		self.tucked = False

		# become a behaviour
		super(LowerHead, self).__init__("Lower head!")

	def update(self):

		# try to tuck head if haven't already
		if not self.tried:

			# command
			self.move_head_req = self.move_head_srv("down")
			self.tried = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# react to outcome
		else: return pt.common.Status.SUCCESS if self.move_head_req.success else pt.common.Status.FAILURE
