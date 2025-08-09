from launch import LaunchDescription
from launch_ros.actions import Node
from llama_bringup.utils import create_llama_launch

def generate_launch_description():


    return LaunchDescription([
        # Node(
        #     package='skull_control_node',
        #     executable='skull_control_node',  # console script, not .py filename
        #     name='skull_control_bt_node',
        #     output='screen',
        # ),
        # Node(
        #     package='skull_control_node',
        #     executable='llm_agent_node',  # console script, not .py filename
        #     name='llm_agent_node',
        #     output='screen',
        # ),
        create_llama_launch(
            # n_batch=8,            # large batch size for throughput
            # n_gpu_layers=0,       # all layers on CPU (set >0 if you have GPU)
            # n_threads=1,          # more CPU threads for parallelism
            n_predict=600,       # maximum output tokens
            n_ctx= 2048,
            model_path="/home/murray/projects/models/SmolLM2.360M.Q4_0.gguf", # model path
            prefix= "\n\nInput:\n",
            suffix= "\n\nOutput:\n",
            stopping_words= ["Input:\n"],
            system_prompt='''
  System: You control an embodied robot with a speech tool named 'say_phrase'.
  You must respond in strict JSON format with the following fields:
  - "thought": string, your internal thought process (not spoken).
  - "final_response": string, what to say to the user (may be empty).
  - "tool_calls": array of {"name": string, "args": object} for tool usage.

  You must use the say_phrase tool to speak to the user.

  Respond appropriately to the user's input. Do not always say "Hi there!". Vary your response based on context.

  Example:
  Input:
  The user asks for the weather.

  Output:
  {"thought": "Check the weather and inform the user.", "final_response": "It's sunny today.", "tool_calls": [{"name": "say_phrase", "args": {"text": "It's sunny today."}}]}

  Input:

'''
        )
    ])
