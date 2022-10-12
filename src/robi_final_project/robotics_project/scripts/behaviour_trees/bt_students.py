#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# # go to door until at door
		# b0 = pt.composites.Selector(
		# 	name="Go to door fallback", 
		# 	children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		# )
		# # tuck the arm
		# b1 = tuckarm()

		# # go to table
		# b2 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		# )

		# # move to chair
		# b3 = pt.composites.Selector(
		# 	name="Go to chair fallback",
		# 	children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		# )

		# # lower head
		# b4 = movehead("down")

		# # become the tree
		# tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])

		
		
		b0 = tuckarm() # Tucks arm down
		b1 = movehead("down") # Lowers head to see cube
		b2 = pickcube() # Picks up the cube
		pick = RSequence(name="Full pick sequence", children=[b0, b1, b2])


		b3 = pt.composites.Selector(name="Back up to table 2", children=[counter(40, "At table?"), go("Back up to table!", -1, 0)])
		b4 = pt.composites.Selector(name="Turn towards table 2", children=[counter(30, "Facing table?"), go("Turn towards table!", 0, 1)])
		move_to_place = pt.composites.Sequence(name="Move from pick to place", children=[b3, b4])
		
		place = placecube()

		b5 = pt.composites.Selector(name="Turn towards table 1", children=[counter(30, "Facing table?"), go("Turn towards table!", 0, -1)])
		b6 = pt.composites.Selector(name="Walk up to table 1", children=[counter(30, "At table?"), go("Advance to table!", 1, 0)])
		move_to_pick = pt.composites.Sequence(name="Move from place to pick", children=[b5, b6])
		detect_cube = detectcube()
		check_and_return = pt.composites.Selector(name="Return if cube not in front of us",children=[detect_cube, move_to_pick])

		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, place])
		#tree = RSequence(name="Main sequence", children=[pick, move_to_place, place, check_and_return])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
