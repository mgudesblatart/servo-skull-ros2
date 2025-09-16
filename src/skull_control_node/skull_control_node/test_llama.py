import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from llama_msgs.action import GenerateResponse
from ament_index_python.packages import get_package_share_directory
import os

grammar_path = os.path.join(
    get_package_share_directory("skull_control_node"),
    "configs",
    "tool_grammar.gbnf"
)


class TestLlama(Node):
    def __init__(self) -> None:
        super().__init__("test_llama_node")

        self.declare_parameter("prompt", "Hello, Llama!")
        self.prompt = self.get_parameter("prompt").get_parameter_value().string_value

        # create the client
        self.action_client = ActionClient(
            self, GenerateResponse, "/llama/generate_response")

        # create the goal and set the sampling config
        goal = GenerateResponse.Goal()
        goal.prompt = self.prompt
        goal.sampling_config.temp = 0.3
        goal.sampling_config.mirostat = 2
        goal.sampling_config.mirostat_eta = 0.1
        goal.sampling_config.mirostat_tau = 5.0
        goal.sampling_config.top_k = 20
        goal.sampling_config.top_p = 0.8
        goal.sampling_config.grammar_schema = '''
{
  "type": "object",
  "properties": {
    "thoughts": { "type": "string" },
    "tool_calls": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "say_phrase": {
            "type": "object",
            "properties": {
              "msg": { "type": "string" }
            },
            "required": ["msg"]
          }
        },
        "oneOf": [
          {
            "type": "object",
            "required": ["say_phrase"]
          }
        ]
      }
    },
    "final_output": { "type": "string" }
  },
  "required": ["thoughts", "tool_calls", "final_output"]
}
        '''

        # wait for the server and send the goal
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, send_goal_future)
        get_result_future = send_goal_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result: GenerateResponse.Result = get_result_future.result().result
        print("Llama result:", result)


def main(args=None):
    rclpy.init(args=args)
    node = TestLlama()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()