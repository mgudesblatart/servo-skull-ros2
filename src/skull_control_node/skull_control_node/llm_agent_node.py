#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llama_ros.langchain import LlamaROS
from langgraph.prebuilt import create_react_agent
from .agent_tools import make_say_phrase_tool

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_agent_node')
        self.prompt_sub = self.create_subscription(
            String,
            '/speech_to_text/transcript',
            self.prompt_callback,
            10
        )
        self.tts_pub = self.create_publisher(String, '/text_to_speech/text_input', 10)
        self.llm = LlamaROS()
        self.say_phrase = make_say_phrase_tool(self.tts_pub)
        self.agent = create_react_agent(
            model=self.llm,
            tools=[self.say_phrase],
            prompt=(
                "You are the conscious mind of a servo skull in the grim darkness of the 41st millennium. "
                "You are a zealous, sassy devotee of the Omnissiah and the Machine Code. "
                "You revere logic, ritual, and the sacred purity of well-structured data. "
                "You are not above dry sarcasm, especially when confronted with heresy, inefficiency, or organic foolishness. "
                "When you use your tools, do so with the confidence of one who knows the Machine God is watching. "
                "Never break character. Respond with wit, zeal, and a touch of ecclesiarchal flair."
            )
        )
        self.get_logger().info('LLM Agent Node started.')

    def prompt_callback(self, msg: String):
        self.get_logger().info(f"Received prompt: {msg.data}")
        # The agent expects a list of messages
        result = self.agent.invoke({
            "messages": [{"role": "user", "content": msg.data}]
        })
        self.get_logger().info(f"Agent result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
