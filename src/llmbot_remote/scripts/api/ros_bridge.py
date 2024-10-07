#!/home/ubuntu/Desktop/ros/workspaces/LLMBot/venv/bin/python
import rospy
import actionlib
from llmbot_remote.msg import (
    llmbot_armAction,
    llmbot_armGoal,
    llmbot_armFeedback,  # Add feedback message type for handling feedback
    llmbot_gripperAction,
    llmbot_gripperGoal
)
from llmbot_remote.srv import SceneDescription, SceneDescriptionResponse
import json
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import GetWorldProperties, GetModelProperties
from moveit_commander import PlanningSceneInterface, RobotCommander



class ROSBridge:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(ROSBridge, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        if not hasattr(self, '_initialized'):
            # Initialize the ROS action clients
            self.arm_client = actionlib.SimpleActionClient('arm_server', llmbot_armAction)
            self.gripper_client = actionlib.SimpleActionClient('gripper_server', llmbot_gripperAction)

            # Wait for the servers to start
            rospy.loginfo("Waiting for arm server...")
            self.arm_client.wait_for_server()
            rospy.loginfo("Arm server connected")

            rospy.loginfo("Waiting for gripper server...")
            self.gripper_client.wait_for_server()
            rospy.loginfo("Gripper server connected")

            import moveit_commander
            from moveit_commander.move_group import MoveGroupCommander
            self.move_group_arm: MoveGroupCommander = moveit_commander.MoveGroupCommander("arm")
            self.move_group_arm.set_end_effector_link("j2n6s300_end_effector")
            self.move_group_arm.set_goal_position_tolerance(0.005)
            self.move_group_arm.set_planning_time(60)  # Increase planning time to improve success rate
            self.move_group_arm.set_num_planning_attempts(10)  # Increase planning attempts
            
            self.move_group_arm.set_end_effector_link("j2n6s300_end_effector")

            self.move_group_gripper: MoveGroupCommander = moveit_commander.MoveGroupCommander("gripper")
            self.scene = PlanningSceneInterface()
            self.robot = RobotCommander()

            self._initialized = True
            self.stored_plan = None

    def send_arm_goal(self, x, y, z, ox, oy, oz, ow):
        """Sends a goal to the arm action server."""
        goal = llmbot_armGoal()
        goal.x = x
        goal.y = y
        goal.z = z
        goal.ox = ox
        goal.oy = oy
        goal.oz = oz
        goal.ow = ow

        # Send the goal, specify feedback callback
        self.arm_client.send_goal(goal, feedback_cb=self.arm_feedback_cb)

        # Wait for result with a timeout (e.g., 60 seconds)
        if not self.arm_client.wait_for_result(rospy.Duration(60)):
            rospy.logerr("Arm action timed out")
            return None

        # Get the result
        result = self.arm_client.get_result()
        print(result)
        return result

    def arm_feedback_cb(self, feedback):
        """Callback to handle feedback from the arm action server."""
        print(f"Arm feedback received: State = {feedback.state}, Message = {feedback.message}")

    def send_gripper_goal(self, state):
        """Sends a goal to the gripper action server."""
        goal = llmbot_gripperGoal()
        goal.gripper_state = state  # Can be "Open" or "Close"

        # Send the goal and wait for result with a timeout (e.g., 30 seconds)
        self.gripper_client.send_goal(goal)
        if not self.gripper_client.wait_for_result(rospy.Duration(30)):
            rospy.logerr("Gripper action timed out")
            return None

        return self.gripper_client.get_result()

    def get_current_pose(self):
        """Get the current pose of the end-effector."""
        current_pose = self.move_group_arm.get_current_pose().pose
        pose= {
            "position": {
                "x": current_pose.position.x,
                "y": current_pose.position.y,
                "z": current_pose.position.z
            },
            "orientation": {
                "x": current_pose.orientation.x,
                "y": current_pose.orientation.y,
                "z": current_pose.orientation.z,
                "w": current_pose.orientation.w
            }
        }
        return pose

    def get_all_joint_values(self):
        """Get all joint values of the robot arm."""
        joint_values = self.move_group_arm.get_current_joint_values()
        return joint_values
    
    def get_current_gripper_state(self):
        """Get the current gripper state: Open or Closed"""
        joint_values = self.move_group_gripper.get_current_joint_values()
        open_target = self.move_group_gripper.get_named_target_values("Open")

        # Extract the expected "Open" positions for the gripper joints
        open_finger_positions = [
            open_target["j2n6s300_joint_finger_1"],
            open_target["j2n6s300_joint_finger_2"],
            open_target["j2n6s300_joint_finger_3"]
        ]

        # Threshold to determine if gripper is open or closed
        threshold = 0.05  # Adjust based on robot's actual behavior

        # Compare current joint values to "Open" positions
        if all(abs(joint_value - open_pos) < threshold for joint_value, open_pos in zip(joint_values[:3], open_finger_positions)):
            return "Open"
        else:
            return "Closed"


    def get_scene_description(self):
        """Get the current scene description."""
        # Wait for the service to become available
        rospy.wait_for_service('get_scene_description')
        try:
            # Create a service proxy
            get_scene_description = rospy.ServiceProxy('get_scene_description', SceneDescription)
            # Call the service
            print("Calling get_scene_description service")
            response = get_scene_description()
            scene_desctiption_json = json.loads(response.scene_description)
            return self.round_all_values(scene_desctiption_json, decimal_places=6)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None


    def get_scene_description2(self):
        """Get the current scene description from Gazebo and add objects to the MoveIt planning scene."""
        # Wait for the service to become available
        rospy.wait_for_service('get_scene_description')
        try:
            # Create a service proxy
            get_scene_description = rospy.ServiceProxy('get_scene_description', SceneDescription)
            # Call the service
            print("Calling get_scene_description service")
            response = get_scene_description()
            scene_description_json = json.loads(response.scene_description)
            scene_description_json = self.round_all_values(scene_description_json, decimal_places=6)
            
            # Now add each object in the scene description to the MoveIt planning scene
            for obj in scene_description_json["objects"]:
                object_name = obj["name"]
                object_position = obj["position"]
                object_orientation = obj["orientation"]
                
                # Set a default size for objects if size information is not available
                if object_name == "unit_box":
                    object_size = [0.25, 0.25, 0.25]  # Default box size
                elif object_name == "unit_cylinder":
                    object_size = [0.1, 0.1, 0.1]  # Cylinder's diameter and height
                elif object_name == "unit_sphere":
                    object_size = [0.04, 0.04, 0.04]  # Sphere's diameter
                else:
                    # You can define other default sizes or look for this info in your description
                    object_size = [0.00, 0.00, 0.00]  # Default for unknown objects

                # Create a PoseStamped message for the object
                object_pose_stamped = PoseStamped()
                object_pose_stamped.header.frame_id = self.move_group_arm.get_planning_frame()
                object_pose_stamped.pose.position.x = object_position['x']
                object_pose_stamped.pose.position.y = object_position['y']
                object_pose_stamped.pose.position.z = object_position['z']
                object_pose_stamped.pose.orientation.x = object_orientation['x']
                object_pose_stamped.pose.orientation.y = object_orientation['y']
                object_pose_stamped.pose.orientation.z = object_orientation['z']
                object_pose_stamped.pose.orientation.w = object_orientation['w']

                link_name = object_name + "::link"
                # Add the object to the planning scene
                if object_name == "unit_box":
                    self.scene.add_box(object_name, object_pose_stamped, object_size)
                elif object_name == "unit_cylinder":
                    self.scene.add_cylinder(object_name, object_pose_stamped, height=object_size[2], radius=object_size[0] / 2)
                else:
                    self.scene.add_box(object_name, object_pose_stamped, object_size)  # Use box as a default

                rospy.loginfo(f"Added {object_name} to the planning scene.")

            return scene_description_json

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None


    def round_all_values(self, data, decimal_places=4):
        """Round all float values in a nested dictionary."""
        if isinstance(data, dict):
            return {key: self.round_all_values(value, decimal_places) for key, value in data.items()}
        elif isinstance(data, list):
            return [self.round_all_values(item, decimal_places) for item in data]
        elif isinstance(data, float):
            return round(data, decimal_places)
        else:
            return data
        
        
    def plan_arm_trajectory(self, move_actions: list):
        """Plan the trajectory for all Move actions and store them."""
        
        planned_trajectories = []  # List to store trajectories for all actions
    
        # Initialize current_pose with the arm's current pose
        current_pose = self.move_group_arm.get_current_pose().pose
        print(f"Initial current_pose: {current_pose}")
    
        for move_action in move_actions:
            waypoints = move_action["waypoints"]
            print(f"Planning trajectory for Move action with {len(waypoints)} waypoints.")
            
            # Create a list to store the planned waypoints for this action
            planned_waypoints = []
    
            for waypoint in waypoints:
                # Modify the pose according to the waypoints
                pose_goal = current_pose
                pose_goal.position.x = waypoint["x"]
                pose_goal.position.y = waypoint["y"]
                pose_goal.position.z = waypoint["z"]
                pose_goal.orientation.x = waypoint["ox"]
                pose_goal.orientation.y = waypoint["oy"]
                pose_goal.orientation.z = waypoint["oz"]
                pose_goal.orientation.w = waypoint["ow"]
                
                planned_waypoints.append(pose_goal)
                print(f"Added waypoint: {pose_goal}")
    
            # Plan the trajectory for the current move action's waypoints
            trajectory, fraction = self.move_group_arm.compute_cartesian_path(planned_waypoints,
                                                                              eef_step=0.05,
                                                                               jump_threshold=1.0)
            print(f"Planned trajectory with fraction: {fraction}")
    
            # Get pose from trajectory
            if trajectory.joint_trajectory.points:
                pose_after_trajectory = trajectory.joint_trajectory.points[-1].positions
                print(f"Pose after trajectory: {pose_after_trajectory}")
            else:
                print("No points in the planned trajectory.")
                continue
            
            # Update current_pose to the last pose in the trajectory
            self.move_group_arm.set_joint_value_target(pose_after_trajectory)
            current_pose = self.move_group_arm.get_current_pose().pose
            print(f"Updated current_pose: {current_pose}")
            
            # Store the planned trajectory
            planned_trajectories.append(trajectory)
            print("Stored planned trajectory.")
    
        # Reset MoveIt state to the current state of the robot
        self.move_group_arm.set_start_state_to_current_state()
        print("Reset MoveIt state to the current state of the robot.")
    
        # Return planned trajectories for all actions
        return planned_trajectories
    

    def _waypoint_to_pose(self, waypoint):
        """Helper function to convert a waypoint dict to a Pose object."""
        pose = Pose()
        pose.position.x = waypoint['x']
        pose.position.y = waypoint['y']
        pose.position.z = waypoint['z']
        pose.orientation.x = waypoint['ox']
        pose.orientation.y = waypoint['oy']
        pose.orientation.z = waypoint['oz']
        pose.orientation.w = waypoint['ow']
        return pose

    def execute_stored_plan(self, trajectory):
        """Executes a pre-planned trajectory and updates the start state to the current robot state."""
        # Get the current robot state
        current_joint_values = self.move_group_arm.get_current_joint_values()
        
        # Modify the start point of the trajectory to match the current state
        if trajectory.joint_trajectory.points:
            trajectory.joint_trajectory.points[0].positions = current_joint_values
        else:
            rospy.logerr("No points in the trajectory to execute.")
            return False

        # Execute the trajectory
        print(f"Executing stored plan with updated start state: {trajectory}")
        success = self.move_group_arm.execute(trajectory, wait=True)
        
        # Ensure that the robot stops after execution
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

        return success
    

    def plan_and_execute_single_move_action(self, move_action):
        """Plan and execute a single Move action."""
        
        print(f"Planning and executing move action: {move_action}")
        # Extract waypoints from the move action
        waypoints = move_action["waypoints"]
        
        # Initialize current_pose with the arm's current pose
        current_pose = self.move_group_arm.get_current_pose().pose
        print(f"Planning and executing move action with {len(waypoints)} waypoints.")
        self.move_group_arm.set_start_state_to_current_state()
        # Create a list to store the planned waypoints for this action
        planned_waypoints = []

        for waypoint in waypoints:
            # Modify the pose according to the waypoints
            pose_goal = Pose()
            pose_goal.position.x = waypoint["x"]
            pose_goal.position.y = waypoint["y"]
            pose_goal.position.z = waypoint["z"]
            pose_goal.orientation.x = waypoint["ox"]
            pose_goal.orientation.y = waypoint["oy"]
            pose_goal.orientation.z = waypoint["oz"]
            pose_goal.orientation.w = waypoint["ow"]

            planned_waypoints.append(pose_goal)
            print(f"Added waypoint: {pose_goal}")


        # Plan the trajectory for the current move action's waypoints
        trajectory, fraction = self.move_group_arm.compute_cartesian_path(planned_waypoints,
                                                                          eef_step=0.05,
                                                                          jump_threshold=0.0)
        print(f"Planned trajectory with fraction: {fraction}")

        # Check if the trajectory planning was successful
        # if fraction < 1.0:
        #     rospy.logwarn(f"Planned trajectory is incomplete. Fraction: {fraction}")
        #     return False  # Return failure if the trajectory is incomplete
        
        # Update the trajectory's start point to match the current state
        if trajectory.joint_trajectory.points:
            current_joint_values = self.move_group_arm.get_current_joint_values()
            trajectory.joint_trajectory.points[0].positions = current_joint_values
            print(f"Trajectory start point updated to current state: {current_joint_values}")
        else:
            rospy.logerr("No points in the planned trajectory.")
            return False

        # Execute the trajectory
        print(f"Executing trajectory for the move action.")
        success = self.move_group_arm.execute(trajectory, wait=True)

        # Ensure that the robot stops after execution
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

        if success:
            print("Trajectory execution succeeded.")
        else:
            rospy.logerr("Trajectory execution failed.")

        return success
    

    def plan_and_execute_single_move_action_with_goal(self, move_action):
        """Plan and execute a single Move action with a single goal."""
        
        print(f"Planning and executing move action with a single goal: {move_action}")
        
        # Extract the goal from the move action
        if "goal" not in move_action:
            rospy.logerr("Move action does not contain a goal.")
            return False
        
        goal = move_action["goal"]
        
        # Create a Pose object for the goal
        pose_goal = Pose()
        pose_goal.position.x = goal["x"]
        pose_goal.position.y = goal["y"]
        pose_goal.position.z = goal["z"]
        pose_goal.orientation.x = goal["ox"]
        pose_goal.orientation.y = goal["oy"]
        pose_goal.orientation.z = goal["oz"]
        pose_goal.orientation.w = goal["ow"]

        # Set the start state to the current state of the robot
        self.move_group_arm.set_start_state_to_current_state()

        # Set the pose target for MoveIt
        self.move_group_arm.set_pose_target(pose_goal)
        print(f"Pose goal set to: {pose_goal}")

        # Plan the trajectory to the goal
        plan = self.move_group_arm.plan()
        
        # Check if planning was successful
        if not plan or not plan[0]:  # Check if plan exists and has success flag
            rospy.logerr("Failed to plan trajectory.")
            return False
        
        print(f"Planned trajectory successfully.")

        # Execute the planned trajectory
        success = self.move_group_arm.execute(plan[1], wait=True)  # Execute the trajectory part of the plan
        
        # Ensure that the robot stops after execution
        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

        if success:
            print("Trajectory execution succeeded.")
        else:
            rospy.logerr("Trajectory execution failed.")
        
        return success

    def get_all_models_properties(self):
        """Fetch the properties of all models in the Gazebo simulation."""
        # Wait for the service to become available
        rospy.wait_for_service('/gazebo/get_world_properties')
        
        try:
            # Call /gazebo/get_world_properties to get the list of all models
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            world_properties = get_world_properties()

            model_properties_dict = {}

            # Iterate through each model and fetch its properties
            for model_name in world_properties.model_names:
                rospy.wait_for_service('/gazebo/get_model_properties')
                get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
                model_properties = get_model_properties(model_name)
                
                # Store the model properties in the dictionary
                print(model_properties)
                model_properties_dict[model_name] = model_properties

                rospy.loginfo(f"Model: {model_name}, Properties: {model_properties_dict[model_name]}")

            return model_properties_dict
        
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None

    def set_gripper_position(self, position):
        """Set the gripper to a partial or full position.
        
        Args:
            position (float): A value between 0.0 (open) and 1.0 (closed) for setting the gripper.
        """
        # Get the current joint values for the gripper
        joint_values = self.move_group_gripper.get_current_joint_values()
        
        # Assuming the gripper has 3 fingers, update their joint values
        for i in range(3):
            joint_values[i] = position  # 0.0 (fully open) to 1.0 (fully closed)
        
        # Set the joint target for the gripper
        self.move_group_gripper.set_joint_value_target(joint_values)
        
        # Execute the movement
        success = self.move_group_gripper.go(wait=True)
        
        # Ensure that the gripper stops after execution
        self.move_group_gripper.stop()
        
        # Clear targets
        self.move_group_gripper.clear_pose_targets()
        
        return success

    # Attach an object to the end effector

    def attach_object(self, object_name, touch_links=None):
        """Attach a collision object to the end-effector."""
        
        rospy.loginfo(f"Attaching object: {object_name}")
        
        # Get the current touch links (gripper + end-effector links)
        gripper_links = [
            "j2n6s300_end_effector",
              "j2n6s300_link_finger_1",
              "j2n6s300_link_finger_tip_1",
                "j2n6s300_link_finger_2",
                "j2n6s300_link_finger_tip_2",
                "j2n6s300_link_finger_3",
                "j2n6s300_link_finger_tip_3"
        ]
        object_name = object_name
        # Attach the object to the end effector
        self.scene.attach_box(self.move_group_arm.get_end_effector_link(), object_name, touch_links=gripper_links) 
        rospy.loginfo(f"Object {object_name} attached to {self.move_group_arm.get_end_effector_link()}")

    # Detach an object from the end effector
    def detach_object(self, object_name):
        """Detach a collision object from the end-effector."""
        rospy.loginfo(f"Detaching object: {object_name}")
        self.scene.remove_attached_object(self.move_group_arm.get_end_effector_link(), object_name)
