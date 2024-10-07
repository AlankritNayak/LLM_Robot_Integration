#!/home/ubuntu/Desktop/ros/workspaces/LLMBot/venv/bin/python
import rospy
import actionlib
from llmbot_remote.msg import llmbot_gripperAction, llmbot_gripperResult
import sys
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander

class GripperServer:
    result = llmbot_gripperResult()

    def __init__(self, name):
        self._action_name = name
        moveit_commander.roscpp_initialize(sys.argv)
        self.gripper_move_group: MoveGroupCommander = moveit_commander.MoveGroupCommander("gripper")
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                llmbot_gripperAction, 
                                                execute_cb=self.execute_cb, 
                                                auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        success = True
        rospy.loginfo('Received gripper goal: %s', goal.gripper_state)
        
        # Handle gripper state
        self.gripper_move_group.set_named_target(goal.gripper_state)
        
        gripper_success = self.gripper_move_group.go(wait=True)
        self.gripper_move_group.stop()

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted', self._action_name)
            self._as.set_preempted()
            success = False

        if success and gripper_success:
            self.result.success = True
            rospy.loginfo('%s: Succeeded', self._action_name)
            self._as.set_succeeded(self.result)
        else:
            rospy.loginfo('%s: Failed', self._action_name)
            self._as.set_aborted()

if __name__ == '__main__':
    rospy.init_node('gripper_server')
    server = GripperServer('gripper_server')
    rospy.spin()
