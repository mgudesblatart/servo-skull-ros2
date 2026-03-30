# Typed envelope from skull_control to llm_agent.

string channel
string source
string type
string event
string urgency
string text

# JSON-encoded object for optional metadata payload.
string payload_json

# Wall-clock publish timestamp (time.time()).
float64 ts
