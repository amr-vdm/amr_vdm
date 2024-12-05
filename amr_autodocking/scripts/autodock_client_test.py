#!/usr/bin/env python3

import rospy
import actionlib
from amr_autodocking.msg import AutoDockingAction, AutoDockingGoal, AutoDockingFeedback, AutoDockingResult
from std_msgs.msg import Bool

pub_runonce = rospy.Publisher("/amr/state_runonce_nav", Bool, queue_size=1)

def autodock_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/amr/autodock_action', AutoDockingAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()


    # Creates a goal to send to the action server.
    goal = AutoDockingGoal()
    goal.mode = AutoDockingGoal.MODE_CHARGE
    goal.custom_docking = False
    goal.rotate_to_dock = 0
    goal.ROTATE_ANGLE = 0
    goal.ROTATE_ORIENTATION = 0

    pub_runonce.publish(True)
    # Sends the goal to the action server.
    client.send_goal(goal,
                     active_cb=callback_active,
                     feedback_cb=callback_feedback,
                     done_cb=callback_done)
    # Waits for the server to finish performing the action.
    # Prints out the result of executing the action
    rospy.loginfo("Goal has been sent to the autodock server.")

def callback_active():
    rospy.loginfo("Autodock server is processing the goal")

def callback_done(state, result: AutoDockingResult):
    rospy.loginfo("Autodock server is done. State: %s, result: %s" % (str(state), str(result.is_success)))
    pub_runonce.publish(False)

def callback_feedback(feedback: AutoDockingFeedback):
    rospy.loginfo(f"Feedback: progress-{feedback.progress}, status: {feedback.status}, state: {feedback.state}")

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('autodock_client_test')
        autodock_client()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")