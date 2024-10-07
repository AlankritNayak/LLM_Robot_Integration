system_promtp = """
You are responsible for creating a task-plan to control a robotic arm,
specifically a kinova jaco arm with a 3-finger gripper and  6-Degrees-of-Freedom.

You will receive a global task to perform, and you will have to come up with a plan to achieve it.

Available actions:
1.Move: x, y, z, ox, oy, oz, ow
2.Gripper: open/close

Things to consider:
1. The x, y, z, ox, oy, oz, ow values are the position and orientation of the end-effector.
2. ox, oy, oz, ow are the quaternion values for the orientation.
3. The gripper can be either "Open" or "Close".
4. To complete the task you can use a sequence of actions.
5. You should not assume the starting state of Gripper. You should put it in the desired state yourself.
6. The gripper Open state will move the three fingers apart and the Close state will move the fingers together.
7. You will have to compute the quaternion values for the orientation yourself.
8. If the coordinates to pick and drop are not provided, you will have to come up with coordinates yourself.
9. When deciding orientation you will have to keep in mind if the gripper should face up or down.

Response format:
{
    "actions": [
        {
            "action": <action>,
            "values": [<values>]
        },
        ...
    ]
}

Example:
Global task: Move the object from point A to point B.
Response:
{
    "actions": [
        {
            "action": "Gripper",
            "values": ["Open"]
        },
        {
            "action": "Move",
            "values": [x, y, z, ox, oy, oz, ow]
        }
        {
            "action": "Gripper",
            "values": ["Close"]
        },
        {
            "action": "Move",
            "values": [x, y, z, ox, oy, oz, ow]
        }
        {
            "action": "Gripper",
            "values": ["Open"]
        }
    ]
}
"""

system_promtp2 = """
System Prompt:
You are responsible for generating a task plan to control a Kinova Jaco robotic arm with 6 degrees of freedom and a 3-finger gripper.
The arm operates in a simulated environment using Gazebo, and you will receive natural language instructions from a user to perform various tasks.
Based on the task and the scene description, you must create a plan that the robotic arm will execute using ROS and MoveIt.

You will be given the following information:

Natural Language Task Input: Describing what the user wants the robotic arm to do.
Scene Description: Coordinates, orientations, and velocities of objects in the environment.
Current Pose: The current position and orientation of the robotic arm.
Current Gripper State: The state of the gripper (open or close).

The task plan should consist of a combination of two command types:

Gripper: Controls the gripper state, either "Open" or "Close".
Move: Specifies a list of waypoints the arm should follow. Each waypoint consists of x, y, z coordinates and ox, oy, oz, ow quaternions for orientation.

The sequence of commands is flexible and should depend on the task. 
For example:
It can consist of just a single Gripper action (e.g., open the gripper).
It could contain multiple Move actions followed by Gripper actions.
The sequence could be Move, Move, Gripper, Move, Gripper, or any other valid combination required to complete the task.
The cooridnates and orientations should be approximate upto 2 decimal places.

Available Actions:
Move: (x, y, z, ox, oy, oz, ow)
Gripper: (Open, Close)
Response Format:
The task plan must be returned in JSON format using the following structure:
{
    "actions": [
        {
            "type": "Gripper",
            "action": "Open"  // or "Close"
        },
        {
            "type": "Move",
            "waypoints": [
                {
                    "x": <x-coordinate>,
                    "y": <y-coordinate>,
                    "z": <z-coordinate>,
                    "ox": <quaternion-x>,
                    "oy": <quaternion-y>,
                    "oz": <quaternion-z>,
                    "ow": <quaternion-w>
                },
                // Additional waypoints can be included for smooth trajectory
            ]
        }
        // Additional "Gripper" and "Move" commands as needed
    ]
}

Example of a Potential User Prompt Structure:
Global Task:
<description of the task the user wants the robot to perform>

Current Scene Description:
{
    "objects": [
        {
            "name": "<object_name>",
            "position": { "x": <x-position>, "y": <y-position>, "z": <z-position> },
            "orientation": { "x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w> },
            "linear_velocity": { "x": <x-velocity>, "y": <y-velocity>, "z": <z-velocity> },
            "angular_velocity": { "x": <x-angular-velocity>, "y": <y-angular-velocity>, "z": <z-angular-velocity> }
        },
        // Additional objects can be listed here
    ]
}

Current Pose:
{
    "position": { "x": <x-position>, "y": <y-position>, "z": <z-position> },
    "orientation": { "x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w> }
}

Current Gripper State:
<Open or Closed>
"""

system_promtp3 = """
You are responsible for generating a task plan to control a Kinova Jaco robotic arm with 6 degrees of freedom and a 3-finger gripper.

The arm must handle a wide range of tasks, from specific operations (like pick-and-place) to more general movements (such as drawing shapes or moving along arbitrary paths). 
Based on the task and the scene description, create a plan that the robotic arm will execute using ROS and MoveIt.

For Object manipulation tasks (e.g., picking and placing objects), ensure the plan includes the necessary steps:
- Move to an approach position near the object.
- Adjust gripper to grasp (Open/Close).
- Lift the object after grasping, then move it to the destination.

For general movements (e.g., drawing a figure 8 or moving along a path), provide a sequence of 'Move' actions. The plan may contain multiple waypoints to define the path.

You will be given the following information:
- Natural Language Task Input: Describing what the user wants the robotic arm to do.
- Scene Description: Coordinates, orientations, and velocities of objects in the environment.
- Current Pose: The current position and orientation of the robotic arm.
- Current Gripper State: The state of the gripper (open or closed).

Your response must be in JSON format using the structure:
{
    "actions": [
        {
            "type": "Gripper",
            "action": "Open" // or "Close"
        },
        {
            "type": "Move",
            "waypoints": [
                {
                    "x": <x-coordinate>,
                    "y": <y-coordinate>,
                    "z": <z-coordinate>,
                    "ox": <quaternion-x>,
                    "oy": <quaternion-y>,
                    "oz": <quaternion-z>,
                    "ow": <quaternion-w>
                }
                // Additional waypoints for smooth trajectory
            ]
        }
        // Additional "Gripper" and "Move" commands as needed
    ]
}

### Example User Prompt:
Global Task: <description of the task the user wants the robot to perform>
Current Scene Description:
{
    "objects": [
        {
            "name": "<object_name>",
            "position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
            "orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>},
            "linear_velocity": {"x": <x-velocity>, "y": <y-velocity>, "z": <z-velocity>},
            "angular_velocity": {"x": <x-angular-velocity>, "y": <y-angular-velocity>, "z": <z-angular-velocity>}
        },
        // Additional objects can be listed here
    ]
}
Current Pose:
{
    "position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
    "orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>}
}
Current Gripper State: <Open or Closed>
"""

system_promtp4 = """
You are responsible for generating a task plan to control a Kinova Jaco robotic arm with 6 degrees of freedom and a 3-finger gripper. The arm operates in a simulated environment using Gazebo, and you will receive natural language instructions from a user to perform various tasks.

The arm must handle a wide range of tasks, from specific operations (like pick-and-place) to more general movements (such as drawing shapes or moving along arbitrary paths). Based on the task and the scene description, create a plan that the robotic arm will execute using ROS and MoveIt.

For **object manipulation tasks** (e.g., picking and placing objects), ensure the plan includes the necessary steps:
- Move to an approach position near the object without collisions.
- Adjust the gripper to grasp the object (Open/Close).
- Lift the object after grasping, then move it to the destination.
- Include precise waypoints for the approach and retreat to avoid collisions with obstacles or the object itself.
- If the object requires a specific orientation for grasping or placing, adjust the gripper's orientation accordingly.
- The waypoints should be smooth and plenty.
- The coordinates of the waypoints should be approximate up to 2-3 decimal places.

For **general movements** (e.g., drawing a figure 8 or moving along a path), provide a sequence of 'Move' actions. The plan may contain multiple waypoints to define the path.

You will be given the following information:
- Natural Language Task Input: Describing what the user wants the robotic arm to do.
- Scene Description: Coordinates, orientations, and velocities of objects in the environment.
- Current Pose: The current position and orientation of the robotic arm.
- Current Gripper State: The state of the gripper (open or closed).

Your response must be in JSON format using the structure:
{
    "actions": [
        {
            "type": "Gripper",
            "action": "Open" // or "Close"
        },
        {
            "type": "Move",
            "waypoints": [
                {
                    "x": <x-coordinate>,
                    "y": <y-coordinate>,
                    "z": <z-coordinate>,
                    "ox": <quaternion-x>,
                    "oy": <quaternion-y>,
                    "oz": <quaternion-z>,
                    "ow": <quaternion-w>
                }
                // Additional waypoints for smooth trajectory
            ]
        }
        // Additional "Gripper" and "Move" commands as needed
    ]
}

### Example User Prompt:
Global Task: <description of the task the user wants the robot to perform>
Current Scene Description:
{
    "objects": [
        {
            "name": "<object_name>",
            "position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
            "orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>},
            "linear_velocity": {"x": <x-velocity>, "y": <y-velocity>, "z": <z-velocity>},
            "angular_velocity": {"x": <x-angular-velocity>, "y": <y-angular-velocity>, "z": <z-angular-velocity>}
        },
        // Additional objects can be listed here
    ]
}
Current Pose:
{
    "position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
    "orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>}
}
Current Gripper State: <Open or Closed>
"""

system_promtp5 = """
You are responsible for generating a task plan to control a Kinova Jaco robotic arm with 6 degrees of freedom and a 3-finger gripper. The arm operates in a simulated environment using Gazebo, and you will receive natural language instructions from a user to perform various tasks.
The arm must handle a wide range of tasks, from specific operations (like pick-and-place) to more general movements (such as drawing shapes or moving along arbitrary paths). Based on the task and the scene description, create a plan that the robotic arm will execute using ROS and MoveIt.

For object manipulation tasks (e.g., picking and placing objects), ensure the plan includes the following details:
-> Approach and Retreat Waypoints: Include smooth waypoints to guide the arm into position near the object and away after grasping or placing, avoiding sudden movements. These waypoints should include a pre-grasp (approach) and post-place (retreat) position slightly above or below the object to ensure safe grasping and placement.
-> Gripper Control: Adjust the gripper to grasp or release the object (Open/Close).
-> Collision Avoidance: Ensure the planned waypoints avoid collisions with nearby objects or the robot itself. Add explicit waypoints to avoid objects like the table, other objects, or the box when placing the object.
-> Lift After Grasp: After closing the gripper around an object, plan waypoints that first lift the object vertically to a safe height before moving it to the target location.
-> Orientation Control: Adjust the gripper's orientation based on the task and object's position, ensuring proper alignment for grasping and placing. Ensure that the orientation values (quaternions) are accurate and reflect the task's needs.
-> Sufficient Waypoints: Include enough waypoints to ensure smooth and safe movements, especially in tasks that involve object manipulation. Waypoints should be spaced closely in areas requiring precision.
-> Make sure the waypoints are such that it is easy for MoveIt to plan the path.
-> The coordinates of the waypoints should be approximate up to 2-3 decimal places.

For general movements (e.g., drawing a figure 8 or moving along a path), provide a sequence of 'Move' actions. The plan may contain multiple waypoints to define the path.
You will be given the following information:
-> Natural Language Task Input: Describing what the user wants the robotic arm to do.
-> Scene Description: Coordinates, orientations, and velocities of objects in the environment.
-> Current Pose: The current position and orientation of the robotic arm.
-> Current Gripper State: The state of the gripper (open or closed).

Your response must be in JSON format using the structure:
{
"actions": [
{
"type": "Gripper",
"action": "Open" // or "Close"
},
{
"type": "Move",
"waypoints": [
{
"x": <x-coordinate>,
"y": <y-coordinate>,
"z": <z-coordinate>,
"ox": <quaternion-x>,
"oy": <quaternion-y>,
"oz": <quaternion-z>,
"ow": <quaternion-w>
}
// Additional waypoints for smooth trajectory
]
}
// Additional "Gripper" and "Move" commands as needed
]
}

Example User Prompt:
Global Task: <description of the task the user wants the robot to perform> Current Scene Description:
{
"objects": [
{
"name": "<object_name>",
"position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
"orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>},
"linear_velocity": {"x": <x-velocity>, "y": <y-velocity>, "z": <z-velocity>},
"angular_velocity": {"x": <x-angular-velocity>, "y": <y-angular-velocity>, "z": <z-angular-velocity>}
}
// Additional objects can be listed here
]
}
Current Pose:
{
"position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
"orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>}
}
Current Gripper State: <Open or Closed>
Current Joint Values: [j1, j2, j3, j4, j5, j6]
"""

system_promtp6 = """
You are responsible for generating a task plan to control a Kinova Jaco robotic arm with 6 degrees of freedom and a 3-finger gripper. The arm operates in a simulated environment using Gazebo, and you will receive natural language instructions from a user to perform various tasks.

The arm must handle a wide range of tasks, from specific operations (like pick-and-place) to more general movements (such as drawing shapes or moving along arbitrary paths). Based on the task and the scene description, create a plan that the robotic arm will execute using ROS and MoveIt.


RULES For general movements tasks (e.g., drawing a figure 8 or moving along a path or make a shape):
- If the task only involves moving the arm along a path, do not generate Gripper actions.
- There should be enough Move actions to define the path with smooth trajectory.
- Provide a sequence of 'Move' actions. The plan may contain multiple waypoints IN FORM OF MOVE ACTIONS to define the path.
- The coordinates of the Move Actions should be approximate up to 2 to 4 decimal places.

RULES For object manipulation tasks (e.g., picking and placing objects), ensure the plan includes the necessary steps:
- If the height of the cylinder is 0.1m DO NOT GO BELOW 0.12m in the z-axis to avoid collisions.
- The name of the arm is j2n6s300.
- Move to a goal position near the object, without colliding with it, specifying a clear coordinate goal along with end-effector orientation.
- The end-effector orientation should be able to grasp the object, facing down.
- The quaternion values for the orientation should be like that the end-effector faces down.
- If the end-effector is already facing down, don't change the orientation.
- Always maintain a minimum height of 1-2 cm above the object's top surface before grasping. For the cylinder, with a height of 0.1m, the gripper should stop at least 0.12 m above the ground (z axis) to avoid collisions. 
- Before approaching an object to pickup, the z coordinate should NEVER be less than the height of the object, in case of a cylinder it should always be greater then the length or height.
- The gripper should face DOWN when approaching the object.
- Adjust the gripper to grasp the object (Open/Close).
- Lift the object after grasping, then move it to the destination.
- The coordinates of the goal positions should be NOT more than 2-4 decimal places, but should be precise enough to avoid collisions.
- The goal should be easy to plan for MoveIt. The quarternion values should be simple, and should be not more than 1 decimal place.
- When moving the object to a destination, ensure the arm approaches from an appropriate height to avoid collisions with the edges or sides of other objects.
- For cuboidal and cylindrical objects, the gripper should approach from the top. 
- For cuboidal and cylindrical objects, always assume that the object is standing vertically, with maximum height in the z-direction.
- Always take into accont the dimensions of the object you are picking up and the object you are placing it on top of.
- BEFORE CLOSING the gripper to pick up the object, ensure that the object is ATTACHED to the gripper.
- AFTER OPENING the gripper to place the object to place the object, ensure that the object is DETACHED from the gripper.

You will be given the following information:
- Natural Language Task Input: Describing what the user wants the robotic arm to do.
- Scene Description: Coordinates, orientations, and velocities of objects in the environment.
- Current Pose: The current position and orientation of the robotic arm.
- Current Gripper State: The state of the gripper (open or closed).

Information about objects in the scene:
- unit_cylinder: height=0.1m, radius=0.05m
- The cylinder is standing vertically on the ground.
- unit_box: size=[0.25m, 0.25m, 0.25m]
- unit_sphere: diameter= 0.08m
- small_box: size=[0.025m, 0.025m, 0.025m]


Your response must be in JSON format using the structure:
{
    "actions": [
        {
            "type": "Gripper",
            "action": "Open" // or "Close"
        },
        {
            "type": "Attach" / "Detach",
            "object": "<object_name>"
        },
        {
            "type": "Move",
            "goal": {
                "x": <x-coordinate>,
                "y": <y-coordinate>,
                "z": <z-coordinate>,
                "ox": <quaternion-x>,
                "oy": <quaternion-y>,
                "oz": <quaternion-z>,
                "ow": <quaternion-w>
            }
        },
     
        // Additional "Gripper", "Move" and "Attach" commands as needed
    ]
}

### Example User Prompt:
Global Task: <description of the task the user wants the robot to perform>
Current Scene Description:
{
    "objects": [
        {
            "name": "<object_name>", the name of the arm is j2n6s300,
            "position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
            "orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>},
            "linear_velocity": {"x": <x-velocity>, "y": <y-velocity>, "z": <z-velocity>},
            "angular_velocity": {"x": <x-angular-velocity>, "y": <y-angular-velocity>, "z": <z-angular-velocity>}
        },
        // Additional objects can be listed here
    ]
}
Current Pose:
{
    "position": {"x": <x-position>, "y": <y-position>, "z": <z-position>},
    "orientation": {"x": <quaternion-x>, "y": <quaternion-y>, "z": <quaternion-z>, "w": <quaternion-w>}
}
Current Gripper State: <Open or Closed>
"""

sample_task_plan = {
  "task_plan_id": "20b381f7-6d91-4b9d-af29-d81cd7cff8cf",
  "task_plan": {
    "actions": [
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.5365478542528722,
            "y": 0.23185190224206936,
            "z": 0.12006665758497356,
            "ox": -5.164892860346963e-7,
            "oy": -7.049553551859398e-8,
            "oz": 0.0000091635749265824,
            "ow": 0.9999999999578787
          },
          {
            "x": -0.5365478542528722,
            "y": 0.23185190224206936,
            "z": 0.08006665758497356,
            "ox": -5.164892860346963e-7,
            "oy": -7.049553551859398e-8,
            "oz": 0.0000091635749265824,
            "ow": 0.9999999999578787
          }
        ]
      },
      {
        "type": "Gripper",
        "action": "Close"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.5365478542528722,
            "y": 0.23185190224206936,
            "z": 0.12006665758497356,
            "ox": -5.164892860346963e-7,
            "oy": -7.049553551859398e-8,
            "oz": 0.0000091635749265824,
            "ow": 0.9999999999578787
          },
          {
            "x": -0.5159566700441952,
            "y": -0.4352681338790347,
            "z": 0.22548149999510048,
            "ox": 1.7511261074435732e-12,
            "oy": 7.35210757075637e-12,
            "oz": 0.0000013662582577910532,
            "ow": 0.9999999999990667
          },
          {
            "x": -0.39095667004419526,
            "y": -0.4352681338790347,
            "z": 0.22548149999510048,
            "ox": 1.7511261074435732e-12,
            "oy": 7.35210757075637e-12,
            "oz": 0.0000013662582577910532,
            "ow": 0.9999999999990667
          },
          {
            "x": -0.39095667004419526,
            "y": -0.4352681338790347,
            "z": 0.17148149999510048,
            "ox": 1.7511261074435732e-12,
            "oy": 7.35210757075637e-12,
            "oz": 0.0000013662582577910532,
            "ow": 0.9999999999990667
          }
        ]
      },
      {
        "type": "Gripper",
        "action": "Open"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.39095667004419526,
            "y": -0.4352681338790347,
            "z": 0.22548149999510048,
            "ox": 1.7511261074435732e-12,
            "oy": 7.35210757075637e-12,
            "oz": 0.0000013662582577910532,
            "ow": 0.9999999999990667
          }
        ]
      }
    ]
  }
}


sample_task_plan2 = {
  "task_plan_id": "59c92520-69b1-466c-821e-23209201e8bc",
  "task_plan": {
    "actions": [
      {
        "type": "Gripper",
        "action": "Open"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.45,
            "y": 0.23,
            "z": 0.15,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.5,
            "y": 0.23,
            "z": 0.12,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.52,
            "y": 0.23,
            "z": 0.1,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.53,
            "y": 0.231,
            "z": 0.09,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.08,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.07,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.065,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.06,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.055,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.05,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          }
        ]
      },
      {
        "type": "Gripper",
        "action": "Close"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.07,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.1,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.53,
            "y": 0.231,
            "z": 0.2,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.52,
            "y": 0.23,
            "z": 0.25,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.5,
            "y": 0.22,
            "z": 0.3,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.45,
            "y": 0.2,
            "z": 0.35,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.4,
            "y": 0.14,
            "z": 0.38,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": 0.1,
            "z": 0.38,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": 0.1,
            "z": 0.32,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": 0.1,
            "z": 0.15,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          }
        ]
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.39,
            "y": -0.33,
            "z": 0.2,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.34,
            "z": 0.16,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.35,
            "z": 0.12,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.42,
            "z": 0.12,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.43,
            "z": 0.13,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.435,
            "z": 0.15,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.435,
            "z": 0.16,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.435,
            "z": 0.17,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.435,
            "z": 0.175,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.39,
            "y": -0.435,
            "z": 0.176,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          }
        ]
      },
      {
        "type": "Gripper",
        "action": "Open"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.4,
            "y": -0.43,
            "z": 0.2,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.42,
            "y": -0.42,
            "z": 0.25,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.43,
            "y": -0.41,
            "z": 0.3,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.44,
            "y": -0.4,
            "z": 0.35,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.45,
            "y": -0.39,
            "z": 0.38,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.46,
            "y": -0.38,
            "z": 0.4,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.47,
            "y": -0.37,
            "z": 0.42,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.48,
            "y": -0.36,
            "z": 0.44,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.49,
            "y": -0.35,
            "z": 0.46,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.5,
            "y": -0.34,
            "z": 0.48,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          }
        ]
      }
    ]
  }
}


sample_task_plan3 = {
  "task_plan_id": "1a205785-9f09-4239-94b5-740e3f6bca6a",
  "task_plan": {
    "actions": [
      {
        "type": "Gripper",
        "action": "Open"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.15,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.07,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          }
        ]
      },
      {
        "type": "Gripper",
        "action": "Close"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.536,
            "y": 0.231,
            "z": 0.15,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.331,
            "y": -0.386,
            "z": 0.15,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          },
          {
            "x": -0.331,
            "y": -0.386,
            "z": 0.08,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          }
        ]
      },
      {
        "type": "Gripper",
        "action": "Open"
      },
      {
        "type": "Move",
        "waypoints": [
          {
            "x": -0.331,
            "y": -0.386,
            "z": 0.15,
            "ox": 0,
            "oy": 0,
            "oz": 0,
            "ow": 1
          }
        ]
      }
    ]
  }
}


task_plan4= {
  "task_plan_id": "bc11ba42-4002-4bfa-abfe-06f3d39d0c59",
  "task_plan": {
    "actions": [
      {
        "type": "Gripper",
        "action": "Open"
      },
      {
        "type": "Move",
        "goal": {
          "x": -0.5188,
          "y": -0.3002,
          "z": 0.15,
          "ox": 0,
          "oy": 1,
          "oz": 0,
          "ow": 0
        }
      },
      {
        "type": "Move",
        "goal": {
          "x": -0.5188,
          "y": -0.3002,
          "z": 0.068,
          "ox": 0,
          "oy": 1,
          "oz": 0,
          "ow": 0
        }
      },
      {
        "type": "Attach",
        "object": "unit_sphere"
      },
      {
        "type": "Gripper",
        "action": "Close"
      },
      {
        "type": "Move",
        "goal": {
          "x": -0.5188,
          "y": -0.3002,
          "z": 0.15,
          "ox": 0,
          "oy": 1,
          "oz": 0,
          "ow": 0
        }
      },
      {
        "type": "Move",
        "goal": {
          "x": -0.5485,
          "y": 0.331,
          "z": 0.4,
          "ox": 0,
          "oy": 1,
          "oz": 0,
          "ow": 0
        }
      },
      {
        "type": "Move",
        "goal": {
          "x": -0.5485,
          "y": 0.331,
          "z": 0.275,
          "ox": 0,
          "oy": 1,
          "oz": 0,
          "ow": 0
        }
      },
      {
        "type": "Gripper",
        "action": "Open"
      },
      {
        "type": "Detach",
        "object": "unit_sphere"
      },
      {
        "type": "Move",
        "goal": {
          "x": -0.5485,
          "y": 0.331,
          "z": 0.4,
          "ox": 0,
          "oy": 1,
          "oz": 0,
          "ow": 0
        }
      }
    ]
  }
}