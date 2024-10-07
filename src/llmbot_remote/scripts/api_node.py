#!/home/ubuntu/Desktop/ros/workspaces/LLMBot/venv/bin/python
import rospy
import uvicorn
from api.api import app

if __name__ == "__main__":
    rospy.init_node('api_node')
    print("Starting FastAPI server...")
    rospy.loginfo("Starting FastAPI server...")
    uvicorn.run(app, host="0.0.0.0", port=8000)  # Start FastAPI server
    rospy.spin()  # Keep ROS node alive
