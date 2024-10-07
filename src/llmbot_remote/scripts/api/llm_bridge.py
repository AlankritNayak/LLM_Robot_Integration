#!/home/ubuntu/Desktop/ros/workspaces/LLMBot/venv/bin/python
"""
This module provides a LLM bridge for the LLMBot remote control.
It provides methods to interact with LLM models.
"""

import ollama
import json
from api.ros_bridge import ROSBridge
import json
from openai import OpenAI
from openai.types import ResponseFormatJSONObject 
from .system_prompts import (
    system_promtp,
    system_promtp2,
    system_promtp3,
    system_promtp4,
    system_promtp5,
    system_promtp6
)


class OpenAIClientSingleton:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(OpenAIClientSingleton, cls).__new__(cls)
            cls._instance.client = OpenAI(
                api_key=""
            )
        return cls._instance

    @staticmethod
    def get_instance():
        return OpenAIClientSingleton().client

def build_user_prompt(initial_user_input: str):
    scene_description = ROSBridge().get_scene_description()
    current_pose = ROSBridge().get_current_pose()
    gripper_state = ROSBridge().get_current_gripper_state()
    joint_values = ROSBridge().get_all_joint_values()
    model_properties = ROSBridge().get_all_models_properties()


    user_prompt = f"""
    Global task: 
    {initial_user_input}
    Current Scene Description: 
    {scene_description}
    Current Pose: 
    {current_pose}
    Current Gripper State: 
    {gripper_state}
    """

    return user_prompt

model = "llama3.1:8b"
model2 = "gemma2"
model3 = "phi3:14b"
model4 = "mistral-nemo"
model5 = "gpt-4o"

def llm_send_message(message: str):
    print(f"Sending message to LLM: {message}")
    response = ollama.generate(
        model=model4,
        prompt=message,
        system=system_promtp4,
        format="json"
    )


    response_json = json.loads(response["response"])
    return response_json


def llm_send_message_remote(message: str):
    client = OpenAIClientSingleton.get_instance()
    response = client.chat.completions.create(
        model=model5,
        response_format={"type": "json_object"},
        messages=[
            {"role": "system", "content": system_promtp6},
            {"role": "user", "content": message}
        ]
    )   
    print(response)
    response_json = json.loads(response.choices[0].message.content)
    return response_json