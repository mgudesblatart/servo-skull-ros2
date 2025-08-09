import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from llama_msgs.action import GenerateResponse

class DumbActionClientNode(Node):
    def __init__(self):
        super().__init__('dumb_action_client_node')
        self.action_client = ActionClient(
            self, GenerateResponse, 'generate_response')
        self.prompt = "Say something witty, but not too witty."
        self.send_goal()

    def send_goal(self):
        goal = GenerateResponse.Goal()
        goal.prompt = self.prompt
        goal.sampling_config.temp = 0.2
        self.get_logger().info(f"Waiting for action server...")
        self.action_client.wait_for_server()
        self.get_logger().info(f"Sending goal: {self.prompt}")
        send_goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        self.get_logger().info(f"Result: {result.response.text}")

def main(args=None):
    rclpy.init(args=args)
    node = DumbActionClientNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
