#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from llama_msgs.action import GenerateResponse
from langchain.prompts import PromptTemplate
from langchain_core.output_parsers import StrOutputParser

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from skull_control_node.agent_tools import make_say_phrase_tool
from skull_control_node.chat_engine import LLMChatEngine

class LLMAgentNode(Node):
    def __init__(self):
        super().__init__('llm_agent_node')
        self.get_logger().info('Initializing LLM Agent Node...')
        # Use a Reentrant callback group to avoid deadlocks when sending goals from callbacks
        self.cb_group = ReentrantCallbackGroup()
        self.prompt_sub = self.create_subscription(
            String,
            '/speech_to_text/transcript',
            self.prompt_callback,
            10,
            callback_group=self.cb_group
        )
        self.tts_pub = self.create_publisher(String, '/text_to_speech/text_input', 10)
        self.get_logger().info('LLM Agent Node started.')

        # Instantiate LLMChatEngine and pass a callback group for its ActionClient; disable auto_speak so only explicit tool calls speak
        self.llm = LLMChatEngine(self, temp=0.2, penalty_last_n=8, callback_group=self.cb_group, auto_speak=False, max_tokens=2048, stop_sequences=None)
        # Prompt template: strict JSON schema with explicit tool usage rules
        self.prompt_template = PromptTemplate(
            input_variables=["msg"],
            template=(
                '''
                "{msg}"
                '''
            )
        )
        # Use agent_tools for tool creation
        say_phrase_tool = make_say_phrase_tool(self.tts_pub)

        self.llm_tools = self.llm.bind_tools(
            [say_phrase_tool], tool_choice='any'
        )
        # self.chain = self.prompt_template | self.llm_tools | StrOutputParser()
        # self.chain = None  # Not needed; use self.llm.stream_with_tools directly


    def prompt_callback(self, msg: String):
        self.get_logger().info(f"Received prompt: {msg.data}")
        # Avoid Python .format on templates containing JSON braces; do simple replacement for {msg}
        prompt_text = self.prompt_template.template.replace("{msg}", msg.data)
        for output in self.llm.stream_with_tools({"msg": prompt_text}):
            # Expecting structured output: {"tool_calls": [...], "tool_responses": [...], "final_response": ...}
            tool_responses = output.get("tool_responses", [])
            for resp in tool_responses:
                self.get_logger().info(f"Tool call: {resp}")
            final_response = output.get("final_response")
            if final_response:
                self.get_logger().info(f"Final response: {final_response}")
            if output.get("error"):
                self.get_logger().error(f"LLM error: {output['error']}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = LLMAgentNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        # Block the main thread; Ctrl+C will interrupt
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
