#!/usr/bin/env python

import rospy
import py_trees

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    DTParam, \
    NodeType, \
    ParamType

# import messages and services
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Float32, Bool

from duckietown_msgs.msg import Twist2DStamped

# Define the states (or behaviors)
class StartState(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(StartState, self).__init__(name)

    def update(self):
        rospy.loginfo("FSM: Starting")
        return py_trees.common.Status.SUCCESS

class MoveState(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MoveState, self).__init__(name)

    def update(self):
        rospy.loginfo("FSM: Following the lane")
        # Normal move state

        return py_trees.common.Status.RUNNING

class StopState(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(StopState, self).__init__(name)

    def update(self):
        rospy.loginfo("FSM: Stopping")
        return py_trees.common.Status.SUCCESS
    
class TurnState(py_trees.behaviour.Behaviour):
    def __init__(self,name):
        super(TurnState, self).__init__(name)
    
    def update(self):
        rospy.loginfo("FSM: Turning")
        return py_trees.common.Status.Running
        
class BehaviourTreeControllerNode(DTROS):
    def __init__(self, node_name, components):
        super(BehaviourTreeControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self.tree = py_trees.trees.BehaviourTree(components)
        
        # Subscribers
        # Subscriber PID movement controller 
        self.controlledMovementSubscriber = rospy.Subscriber('~control/parameters', Float32MultiArray, self.controller_callback, queue_size=100)
        # Subscriber for turning controller 
        self.turningSubscriber = rospy.Subscriber('~turning/parameters', Float32MultiArray, self.turning_callback, queue_size=100)
        # Subsciber for stop controller
        self.stopSubscriber = rospy.Subscriber('~stop/detected_line', Bool, self.stop, queue_size=1)

        def stop(self):
            pass
    
        def turning_callback(self):
            pass
            
        def controller_callback(self):
            pass

        # Publishers
        self.movement_publisher = rospy.Publisher('~car_cmd',Twist2DStamped,queue_size=1) 


# Create and run the behavior tree
def main():
    # Define the nodes in the tree
    start = StartState(name="Start")
    move = MoveState(name="Move")
    turn = TurnState(name="Turn")
    stop = StopState(name="Stop")
    root = py_trees.composites.Sequence(name="FSM Root")
    root.add_children([start, move, turn, stop])

    rospy.init_node('fsm_example')
    # Add nodes to the tree
    tree_node = BehaviourTreeControllerNode(node_name="behaviour_tree_node", components=root)
    tree_node.register_states()
    
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
