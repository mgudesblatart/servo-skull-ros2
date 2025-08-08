from langchain_core.tools import tool
from std_msgs.msg import String

def make_say_phrase_tool(tts_pub):
    @tool
    def say_phrase(text: str) -> str:
        """Speak the given phrase aloud."""
        msg = String()
        msg.data = text
        tts_pub.publish(msg)
        return f"Spoken: {text}"
    return say_phrase

# Add more tool factories here as needed, e.g. gesture, gaze, etc.
