#!/home/ubuntu/Desktop/ros/workspaces/LLMBot/venv/bin/python
import rospy
import actionlib
from llmbot_remote.msg import llmbot_armAction, llmbot_armResult, llmbot_armFeedback
import sys
import moveit_commander
import math
from moveit_commander.move_group import MoveGroupCommander
from enum import Enum


# Define the ArmState enum
class ArmState(str, Enum):
    PLANNING = "Planning"
    PLANNING_SUCCESS = "PlanningSuccess"
    EXECUTING = "Executing"
    EXECUTION_COMPLETED = "ExecutionCompleted"
    EXECUTION_FAILED = "ExecutionFailed"
    PLANNING_FAILED = "PlanningFailed"
    PREEMPTED = "Preempted"


class ArmServer:
    result = llmbot_armResult()
    feedback = llmbot_armFeedback()

    def __init__(self, name):
        self._action_name = name
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_move_group: MoveGroupCommander = moveit_commander.MoveGroupCommander("arm")
        self.arm_move_group.set_end_effector_link("j2n6s300_end_effector")
        self.arm_move_group.set_goal_position_tolerance(0.1)
        self.arm_move_group.set_planning_time(60)  # Increase planning time to improve success rate
        self.arm_move_group.set_num_planning_attempts(10)  # Increase planning attempts
    
        
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                llmbot_armAction, 
                                                execute_cb=self.execute_cb, 
                                                auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        success = True
        rospy.loginfo('Received Cartesian goal: x=%f, y=%f, z=%f, ox=%f, oy=%f, oz=%f, ow=%f',
                    goal.x, goal.y, goal.z, goal.ox, goal.oy, goal.oz, goal.ow)

        if not (math.isnan(goal.x) or math.isnan(goal.y) or math.isnan(goal.z) or
                math.isnan(goal.ox) or math.isnan(goal.oy) or math.isnan(goal.oz) or math.isnan(goal.ow)):
            # Set the target pose for the end effector
            pose_stamped = self.arm_move_group.get_current_pose()
            pose_stamped.header.frame_id = "root"
            pose_stamped.header.stamp = rospy.Time.now()

            pose_goal = pose_stamped.pose
            pose_goal.position.x = goal.x
            pose_goal.position.y = goal.y
            pose_goal.position.z = goal.z
            pose_goal.orientation.x = goal.ox
            pose_goal.orientation.y = goal.oy
            pose_goal.orientation.z = goal.oz
            pose_goal.orientation.w = goal.ow

            self.arm_move_group.set_pose_target(pose_goal)

            # Provide feedback: planning started
            self.feedback.state = ArmState.PLANNING.value
            self.feedback.message = "Trajectory planning has started."
            self._as.publish_feedback(self.feedback)

            # Plan the motion first
            plan_result = self.arm_move_group.plan()
            print(f"Plan result: {plan_result}")
            # plan_result is a tuple, the first element is success, the second is the actual trajectory
            arm_success, trajectory, planning_time, error_code= plan_result
            

            if arm_success:
                rospy.loginfo('%s: Planning succeeded', self._action_name)

                # Provide feedback: planning succeeded
                self.feedback.state = ArmState.PLANNING_SUCCESS.value
                self.feedback.message = "Planning succeeded, executing trajectory."
                self._as.publish_feedback(self.feedback)

                # Execute the planned trajectory
                arm_execution_success = self.arm_move_group.execute(trajectory, wait=True)

                # Ensure that the robot stops after the movement
                self.arm_move_group.stop()
                self.arm_move_group.clear_pose_targets()

                if arm_execution_success:
                    # Get the current pose after the movement
                    final_pose = self.arm_move_group.get_current_pose().pose

                    # Populate the result with the final position and orientation
                    self.result.final_x = final_pose.position.x
                    self.result.final_y = final_pose.position.y
                    self.result.final_z = final_pose.position.z
                    self.result.final_ox = final_pose.orientation.x
                    self.result.final_oy = final_pose.orientation.y
                    self.result.final_oz = final_pose.orientation.z
                    self.result.final_ow = final_pose.orientation.w

                    # Provide feedback: execution completed
                    self.feedback.state = ArmState.EXECUTION_COMPLETED.value
                    self.feedback.message = "Execution completed successfully."
                    self._as.publish_feedback(self.feedback)

                    rospy.loginfo('%s: Succeeded', self._action_name)
                    self.result.success = True
                else:
                    rospy.loginfo('%s: Execution failed', self._action_name)
                    success = False
                    self.result.success = False

                    # Provide feedback: execution failed
                    self.feedback.state = ArmState.EXECUTION_FAILED.value
                    self.feedback.message = "Execution failed."
                    self._as.publish_feedback(self.feedback)
                    self._as.set_aborted()
            else:
                rospy.loginfo('%s: Planning failed', self._action_name)

                # Provide feedback: planning failed
                self.feedback.state = ArmState.PLANNING_FAILED.value
                self.feedback.message = "Planning failed."
                self._as.publish_feedback(self.feedback)

                success = False
                self.result.success = False
                self._as.set_aborted()

        else:
            rospy.loginfo('Invalid goal coordinates or orientation')
            success = False

        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted', self._action_name)
            self._as.set_preempted()

            # Provide feedback: preempted
            self.feedback.state = ArmState.PREEMPTED.value
            self.feedback.message = "Action preempted."
            self._as.publish_feedback(self.feedback)

            success = False

        if success:
            self.result.success = True
            rospy.loginfo('%s: Succeeded', self._action_name)
            self._as.set_succeeded(self.result)
        else:
            self.result.success = False
            rospy.loginfo('%s: Failed', self._action_name)
            self._as.set_aborted()

if __name__ == '__main__':
    rospy.set_param('use_sim_time', True)
    rospy.init_node('arm_server')
    server = ArmServer('arm_server')

    rospy.spin()
