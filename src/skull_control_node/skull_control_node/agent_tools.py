"""
agent_tools.py

Factory functions that create LangChain-compatible tool objects bound to
live ROS publishers. Each factory takes a publisher and returns a @tool
decorated function ready to hand to a LangChain agent executor.

Add new tool factories here as the agent's capabilities grow (gestures,
gaze control, LED animations, etc.).
"""

from langchain_core.tools import tool
from std_msgs.msg import String


def make_say_phrase_tool(tts_pub):
    """
    Returns a LangChain tool that publishes a spoken phrase via the TTS node.

    Args:
        tts_pub: A ROS publisher for /text_to_speech/text_input (String).
    """
    @tool
    def say_phrase(text: str) -> str:
        """Speak the given phrase aloud through the skull's speaker."""
        msg = String()
        msg.data = text
        tts_pub.publish(msg)
        return f"Spoken: {text}"

    return say_phrase

