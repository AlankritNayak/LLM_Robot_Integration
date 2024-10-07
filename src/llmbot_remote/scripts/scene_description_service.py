#!/home/ubuntu/Desktop/ros/workspaces/LLMBot/venv/bin/python
import rospy
from gazebo_msgs.msg import ModelStates
from llmbot_remote.srv import SceneDescription, SceneDescriptionResponse  # Import the custom service
import json

class SceneDescriptionService:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('scene_description_service')

        # Subscribe to the /gazebo/model_states topic
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        # Create a service to respond with the current scene description
        self.service = rospy.Service('get_scene_description', SceneDescription, self.handle_scene_description_request)

        # Initialize variable to hold the latest model states
        self.latest_model_states = None
        rospy.loginfo("Scene description service is ready.")

    def model_states_callback(self, data):
        # Store the latest model states
        self.latest_model_states = data

    def handle_scene_description_request(self, req):
        # Ensure we have model state data
        if self.latest_model_states is None:
            rospy.logwarn("No model state data available yet.")
            return SceneDescriptionResponse("No data available")

        # Prepare scene data
        scene_data = {"objects": [], "robot": {}}
        
        for i in range(len(self.latest_model_states.name)):
            model_name = self.latest_model_states.name[i]
            model_pose = self.latest_model_states.pose[i]
            model_twist = self.latest_model_states.twist[i]

            # Extract position, orientation, and twist (velocity)
            position = model_pose.position
            orientation = model_pose.orientation
            linear_velocity = model_twist.linear
            angular_velocity = model_twist.angular

            scene_data["objects"].append({
                "name": model_name,
                "position": {
                    "x": position.x,
                    "y": position.y,
                    "z": position.z
                },
                "orientation": {
                    "x": orientation.x,
                    "y": orientation.y,
                    "z": orientation.z,
                    "w": orientation.w
                },
                "linear_velocity": {
                    "x": linear_velocity.x,
                    "y": linear_velocity.y,
                    "z": linear_velocity.z
                },
                "angular_velocity": {
                    "x": angular_velocity.x,
                    "y": angular_velocity.y,
                    "z": angular_velocity.z
                }
            })

        # Convert the scene data to a JSON string
        scene_description = json.dumps(scene_data, indent=2)

        # Return the scene description as the response
        return SceneDescriptionResponse(scene_description)

if __name__ == '__main__':
    try:
        # Initialize and start the service
        service = SceneDescriptionService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
