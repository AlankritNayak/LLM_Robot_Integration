#!/home/ubuntu/Desktop/ros/workspaces/LLMBot/venv/bin/python
from typing import Union
from fastapi import FastAPI, Depends
from api.models.requests import UserMessage
from api.llm_bridge import llm_send_message, build_user_prompt, llm_send_message_remote
from api.ros_bridge import ROSBridge
import time
from functools import lru_cache
import uuid
from .system_prompts import sample_task_plan, sample_task_plan2, sample_task_plan3, task_plan4

task_plan_storage = {}
trajectory_storage = {}


app = FastAPI()


@lru_cache
def get_ros_bridge():
    return ROSBridge()


# Create an instances of the ROSBridge class on startup
# ros_bridge = get_ros_bridge()

@app.get("/")
def read_root():
    return {"Hello": "Robotics"}

# @app.post("/message")
# def read_item(message: UserMessage, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
#     response = llm_send_message(message.message)
#     x, y, z = response["coordinates"]["x"], response["coordinates"]["y"], response["coordinates"]["z"]
#     ox, oy, oz, ow = response["orientation"]["ox"], response["orientation"]["oy"], response["orientation"]["oz"], response["orientation"]["ow"]
#     gripper_state = response["gripper"]
#     ros_bridge.send_arm_goal(x, y, z, ox, oy, oz, ow)
#     ros_bridge.send_gripper_goal(gripper_state)
#     return response

sample_llm_response = {
    "actions": [
        {
            "action": "Move",
            "values": [0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 1.0]
        },
        # Move to position facing down with quaternion orientation
        {
            "action": "Move",
            "values": [-0.02, 0.3001, 0.100001, 1.0, 0.0, 0.0, 0.0]  # Gripper facing down
        }
   
    ]
}

@app.post("/task")
def task(message: UserMessage, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    print(f"Performing message: {message.message}")
    # response = llm_send_message(message.message)
    response = sample_llm_response
    print(f"Performing task: {response}")
    for action in response["actions"]:
        print(f"Performing action: {action}")
        if action["action"] == "Gripper":
            gripper_state = action["values"][0]
            ros_bridge.send_gripper_goal(gripper_state)
        elif action["action"] == "Move":
            x, y, z, ox, oy, oz, ow = action["values"]
            ros_bridge.send_arm_goal(x, y, z, ox, oy, oz, ow)
        time.sleep(20)
    return response


@app.post("/test")
def test(message: UserMessage, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    print(f"Performing message: {message.message}")
    user_prompt = build_user_prompt(message.message)
    llm_response = llm_send_message(user_prompt)
    print(f"LLM Response: {llm_response}")
    return {"user_prompt": user_prompt, "llm_response": llm_response}


@app.post("/plan_trajectory")
def plan_trajectory(waypoints: list, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    success = ros_bridge.plan_arm_trajectory(waypoints)
    if success:
        return {"message": "Trajectory planned successfully", "status": "Planned"}
    else:
        return {"message": "Failed to plan trajectory", "status": "Failed"}
    

@app.post("/execute_trajectory")
def execute_trajectory(ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    success = ros_bridge.execute_stored_plan()
    if success:
        return {"message": "Trajectory executed successfully", "status": "Executed"}
    else:
        return {"message": "Failed to execute trajectory or no plan stored", "status": "Failed"}

@app.post("/generate_task_plan")
def generate_task_plan(message: UserMessage, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    task_plan_id = str(uuid.uuid4())  # Generate a unique ID for the task plan
    prompt = build_user_prompt(message.message)  # Build the user prompt
    print(f"Prompt generated: {prompt}")
   
    response = llm_send_message_remote(prompt)  # Generate task plan from LLM

    # Store the task plan without the 'validated' key
    task_plan_storage[task_plan_id] = {
        "task_plan": response,
        "trajectories": []  # Store trajectories here after validation
    }

    return {"task_plan_id": task_plan_id, "task_plan": response}

@app.post("/validate_task_plan/{task_plan_id}")
def validate_task_plan(task_plan_id: str, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    # Fetch the task plan based on the task_plan_id
    task_plan = task_plan_storage.get(task_plan_id)
    
    # Extract all 'Move' actions from the task plan
    move_actions = [action for action in task_plan["task_plan"]["actions"] if action["type"] == "Move"]

    if not move_actions:
        return {"status": "failed", "message": "No 'Move' actions found in the task plan."}

    # Plan trajectories for all 'Move' actions
    trajectories = ros_bridge.plan_arm_trajectory(move_actions)
    trajectory_storage[task_plan_id] = trajectories
    print(f"Trajectories: {trajectories}")
    return {"status": "success", "message": "Trajectories planned successfully"}

@app.post("/execute_task_plan/{task_plan_id}")
def execute_task_plan(task_plan_id: str, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    if task_plan_id not in task_plan_storage:
        return {"error": "Invalid task plan ID"}

    task_plan_data = task_plan_storage[task_plan_id]
    
    # Fetch the corresponding pre-planned trajectories
    trajectories = trajectory_storage.get(task_plan_id)
    
    if not trajectories:
        return {"error": "No trajectories found for the task plan."}

    move_index = 0  # To track which trajectory corresponds to which Move action

    # Loop through each action in the task plan
    for action in task_plan_data["task_plan"]["actions"]:
        print(f"Executing action: {action}")
        if action["type"] == "Gripper":
            gripper_state = action["action"]  # Either "Open" or "Close"
            ros_bridge.send_gripper_goal(gripper_state)
        elif action["type"] == "Move":
            # Retrieve the pre-planned trajectory for this specific Move action
            if move_index < len(trajectories):
                trajectory = trajectories[move_index]
                move_index += 1
                ros_bridge.execute_stored_plan(trajectory)
            else:
                return {"error": f"Trajectory for Move action {move_index} not found."}

    return {"message": "Task plan executed successfully"}


@app.post("/plan_and_execute_task_plan/{task_plan_id}")
def plan_and_execute_task_plan(task_plan_id: str, ros_bridge: ROSBridge = Depends(get_ros_bridge)):
    """Executes a task plan stored with the provided task_plan_id."""
    

    # Check if the task plan exists
    if task_plan_id not in task_plan_storage:
        return {"error": "Invalid task plan ID"}

    task_plan_data = task_plan_storage[task_plan_id]
    
    # Fetch the stored task plan
    task_plan = task_plan_data["task_plan"]
    
    # Execute the actions in the task plan
    for action in task_plan["actions"]:
        print(f"Executing action: {action}")
        if action["type"] == "Gripper":
            # Handle Gripper action
            gripper_state = action["action"]
            print(f"Executing Gripper action: {gripper_state}")
            # if gripper_state == "Open":
            #     ros_bridge.set_gripper_position(0.0)
            # elif gripper_state == "Close":
            #     ros_bridge.set_gripper_position(0.8)
            ros_bridge.send_gripper_goal(gripper_state)

        elif action["type"] == "Attach":
            # Attach the object to the end effector
            object_name = action["object"]
            ros_bridge.attach_object(object_name)
        elif action["type"] == "Detach":
            # Detach the object from the end effector
            object_name = action["object"]
            ros_bridge.detach_object(object_name)
        elif action["type"] == "Move":
            # Handle Move action
            print(f"Executing Move action: {action}")
            move_success = ros_bridge.plan_and_execute_single_move_action_with_goal(action)
            if not move_success:
                return {"status": "failed", "message": "Move action execution failed"}

    return {"status": "success", "message": "Task plan executed successfully"}
