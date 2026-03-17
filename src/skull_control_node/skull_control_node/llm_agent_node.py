#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from langchain_core.messages import HumanMessage
from rclpy.executors import MultiThreadedExecutor

from llama_ros.langchain import ChatLlamaROS
from skull_control_node.agent_tools import make_say_phrase_tool


class LLMAgentNode(Node, ChatLlamaROS):
    def __init__(self):
        # Initialize ROS2 Node first to establish ROS2 context
        Node.__init__(self, 'llm_agent_node')
        self.get_logger().info('Initializing LLM Agent Node...')

        # Initialize ChatLlamaROS with comprehensive sampling parameters matching test file
        ChatLlamaROS.__init__(
            self,
            temp=0.3,
            penalty_last_n=8,
            mirostat=2,
            mirostat_eta=0.1,
            mirostat_tau=5.0,
            top_k=20,
            top_p=0.8,
            # Optional: JSON schema for structured output (from test file)
            # Uncomment to enforce structured tool calling format
            grammar_schema='''
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
        )

        # Set up STT subscription
        self.prompt_sub = self.create_subscription(
            String,
            '/speech_to_text/transcript',
            self.prompt_callback,
            10
        )

        # Set up TTS publisher
        self.tts_pub = self.create_publisher(String, '/text_to_speech/text_input', 10)

        # Create and bind tools - store reference for manual execution
        self.say_phrase_tool = make_say_phrase_tool(self.tts_pub)
        self.llm_with_tools = self.bind_tools([self.say_phrase_tool], tool_choice='any')

        self.get_logger().info('LLM Agent Node started.')

    def prompt_callback(self, msg: String):
        """Handle incoming speech-to-text transcripts"""
        self.get_logger().info(f"Received prompt: {msg.data}")

        try:
            # Create HumanMessage for LangChain compatibility
            messages = [HumanMessage(content=msg.data)]

            # Get response with tool calls using invoke()
            response = self.llm_with_tools.invoke(messages)

            # Handle regular LLM response content
            if response.content:
                self.get_logger().info(f"LLM response: {response.content}")

            # Execute tools manually following the example pattern
            for tool_call in response.tool_calls:
                if tool_call['name'] == 'say_phrase':
                    # Manually invoke the tool
                    tool_result = self.say_phrase_tool.invoke(tool_call['args'])
                    self.get_logger().info(f"Tool executed: {tool_call['name']}({tool_call['args']}) -> {tool_result}")
                else:
                    self.get_logger().warning(f"Unknown tool call: {tool_call['name']}")

        except Exception as e:
            self.get_logger().error(f"Error processing prompt: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = LLMAgentNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        # If ChatLlamaROS creates a separate LlamaClientNode, add it to the executor
        if hasattr(node, 'llama_client') and node.llama_client is not None:
            node.get_logger().info('Adding llama_client node to executor')
            executor.add_node(node.llama_client)

        # Block the main thread; Ctrl+C will interrupt
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            # Clean up llama_client if it exists
            if hasattr(node, 'llama_client') and node.llama_client is not None:
                node.llama_client.destroy_node()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
