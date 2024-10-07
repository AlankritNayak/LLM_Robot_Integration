# Project Title

This project is a part of my Master Thesis titled: Using LLMs for End-User Friendly Robot Control

## Prerequisites

- ROS Noetic (Robot Operating System) installed
- Virtual environment set up
- Install requirements using `requirements.txt`:
    ```sh
    pip install -r requirements.txt
    ```

## Setup and Launch

1. Activate the virtual environment and source the setup file:
    ```sh
    source venv/bin/activate && source devel/setup.bash
    ```

2. Start the ROS core:
    ```sh
    roscore
    ```

3. Launch the robot in Gazebo:
    ```sh
    roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300
    ```

4. Launch the MoveIt configuration for the robot in Gazebo:
    ```sh
    roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch
    ```

5. Relay the joint states topic:
    ```sh
    rosrun topic_tools relay /j2n6s300/joint_states /joint_states
    ```

6. Launch the llmbot remote:
    ```sh
    roslaunch llmbot_remote llmbot_remote.launch
    ```

## Usage

The last command will launch a FastAPI server on port 8000. You can interact with the server using the following endpoints:

- Generate a task plan:
    ```http
    POST /generate_task_plan
    ```

- Plan and execute the task plan:
    ```http
    POST /execute_task_plan/{task_plan_id}
    ```
