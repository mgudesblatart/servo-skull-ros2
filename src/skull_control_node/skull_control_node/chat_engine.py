import rclpy
import json
import inspect
from typing import Any, Dict, List, Callable, Generator, Tuple, Union, Optional
from threading import Condition, RLock, Thread
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from llama_msgs.action import GenerateResponse
from llama_msgs.msg import SamplingConfig, PartialResponse
import logging

class LLMChatEngine:
    def __init__(self,
                 node,
                 temp=0.4,
                 penalty_last_n=64,
                 callback_group: Optional[ReentrantCallbackGroup] = None,
                 auto_speak: bool = True,
                 stop_sequences=None,
                 max_tokens=256,
                 **kwargs):
        self.node = node
        self.logger = logging.getLogger("LLMChatEngine")
        self.logger.setLevel(logging.DEBUG)
        if not self.logger.hasHandlers():
            handler = logging.StreamHandler()
            formatter = logging.Formatter('[%(asctime)s] %(levelname)s %(name)s: %(message)s')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
        self.logger.debug("Initializing LLMChatEngine")
        self.temp = temp
        self.penalty_last_n = penalty_last_n
        self.config = kwargs
        self.tools = {}
        self.auto_speak = auto_speak
        self.sampling_config = self._set_sampling_config(max_tokens)
        # Stop sequences are now configurable
        self._default_stops = stop_sequences if stop_sequences is not None else []
        # Use a node-registered callback group to ensure proper executor scheduling
        self._callback_group = callback_group or self.node.create_callback_group(ReentrantCallbackGroup)
        self.action_client = ActionClient(
            self.node, GenerateResponse, '/llama/generate_response',
            callback_group=self._callback_group
        )
        self.tool_choice = None
        self._action_done = False
        self._action_done_cond = Condition()
        self._action_result = None
        self._action_status = GoalStatus.STATUS_UNKNOWN
        self._partial_results = []
        self._goal_handle = None
        self._goal_handle_lock = RLock()
        # Keep strong references to futures so they are not GC'd
        self._send_goal_future = None
        self._get_result_future = None
        self.logger.debug(f"Sampling config: temp={self.temp}, penalty_last_n={self.penalty_last_n}, config={self.config}")

    def set_callback_group(self, callback_group):
        self.logger.debug("Setting callback group")
        self._callback_group = callback_group

    def register_tool(self, name, func):
        self.logger.debug(f"Registering tool: {name}")
        self.tools[name] = func

    def _set_sampling_config(self, max_tokens):
        self.logger.debug("Setting sampling config")
        sampling_config = SamplingConfig()
        sampling_config.n_prev = 64
        sampling_config.n_probs = 1
        sampling_config.min_keep = 0
        sampling_config.ignore_eos = False
        sampling_config.temp = self.temp
        sampling_config.penalty_last_n = self.penalty_last_n
        # Cap max tokens to avoid long rambles but ensure non-empty output
        if hasattr(sampling_config, 'n_predict'):
            sampling_config.n_predict = max_tokens
        return sampling_config

    def _create_action_goal(self, prompt):
        # self.logger.debug(f"Creating action goal for prompt: {prompt}")
        goal = GenerateResponse.Goal()
        goal.prompt = prompt
        goal.reset = True
        # Add stop sequences if any
        if self._default_stops:
            try:
                goal.stop = list(self._default_stops)
            except Exception:
                pass
        goal.sampling_config = self.sampling_config
        return goal

    def generate_response(self, prompt: str, stream: bool = False) -> Union[Tuple[Any, int], Generator[Any, None, None]]:
        self.logger.debug(f"generate_response called with prompt: {prompt}, stream={stream}")
        goal = self._create_action_goal(prompt)
        self._action_done = False
        self._action_result = None
        self._action_status = GoalStatus.STATUS_UNKNOWN
        self._partial_results = []
        self.logger.debug("Waiting for action server...")
        self.action_client.wait_for_server()
        self.logger.debug("Sending goal async...")
        # Keep a strong reference to the future
        self._send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        def generator():
            self.logger.debug("Streaming generator started")
            while not self._action_done:
                with self._action_done_cond:
                    to_yield = self._partial_results[:]
                    self._partial_results.clear()
                for item in to_yield:
                    self.logger.debug(f"Yielding partial result: {item}")
                    yield item
                if not self._action_done:
                    with self._action_done_cond:
                        if not self._partial_results and not self._action_done:
                            self.logger.debug("Waiting for partial results or action done...")
                            self._action_done_cond.wait()
            with self._action_done_cond:
                for item in self._partial_results:
                    self.logger.debug(f"Yielding final partial result: {item}")
                    yield item
        if stream:
            self.logger.debug("Returning streaming generator")
            return generator()
        else:
            self.logger.debug("Waiting for action to complete (non-streaming)")
            with self._action_done_cond:
                while not self._action_done:
                    self.logger.debug("Waiting for action done...")
                    self._action_done_cond.wait()
            self.logger.debug(f"Action completed: result={self._action_result}, status={self._action_status}")
            return self._action_result, self._action_status

    def _goal_response_callback(self, future):
        self.logger.debug("Goal response callback triggered")
        try:
            goal_handle = future.result()
        except Exception as e:
            self.logger.error(f"Exception obtaining goal handle: {e}")
            with self._action_done_cond:
                self._action_status = GoalStatus.STATUS_ABORTED
                self._action_done = True
                self._action_done_cond.notify()
            return
        with self._goal_handle_lock:
            self._goal_handle = goal_handle
            # Check accepted flag if available
            accepted = getattr(self._goal_handle, 'accepted', True)
            if not accepted:
                self.logger.error("Goal was rejected by the server")
                with self._action_done_cond:
                    self._action_status = GoalStatus.STATUS_ABORTED
                    self._action_done = True
                    self._action_done_cond.notify()
                self._goal_handle = None
                return
            # Keep a strong reference to the future
            self._get_result_future = self._goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self._get_result_callback)
            self.logger.debug("Added get_result_callback to get_result_future")

    def _get_result_callback(self, future):
        self.logger.debug("Get result callback triggered")
        try:
            result_wrapper = future.result()
            self._action_result = result_wrapper.result
            self._action_status = result_wrapper.status
            self.logger.debug(f"Result received: {self._action_result}, status={self._action_status}")
        except Exception as e:
            self.logger.error(f"Exception in get_result_callback: {e}")
        with self._action_done_cond:
            self._action_done = True
            self.logger.debug("Action marked as done, notifying condition")
            self._action_done_cond.notify()
        with self._goal_handle_lock:
            self._goal_handle = None
            self._get_result_future = None
            self._send_goal_future = None
            self.logger.debug("Goal handle and futures cleared")

    def _feedback_callback(self, feedback):
        self.logger.debug(f"Feedback callback: {feedback}")
        try:
            # In rclpy, feedback message is wrapped with .feedback
            self._partial_results.append(feedback.feedback.partial_response)
            self.logger.debug(f"Partial result appended: {feedback.feedback.partial_response}")
        except Exception as e:
            self.logger.error(f"Exception in feedback_callback: {e}")
        with self._action_done_cond:
            self.logger.debug("Notifying condition for new feedback")
            self._action_done_cond.notify()

    def _parse_output(self, text: str) -> Dict[str, Any]:
        """Try to parse model output into our expected JSON schema. Fallback to final_response."""
        t = (text or "").strip()
        # Strip Markdown code fences
        if t.startswith('```'):
            parts = t.strip('`').split('\n')
            # Try to find JSON block
            t = '\n'.join([line for line in parts if not line.startswith('```')])
            t = t.strip()
        # Direct parse
        try:
            obj = json.loads(t)
            if isinstance(obj, dict):
                return obj
        except Exception:
            pass
        # Try to extract JSON object substring
        if '{' in t and '}' in t:
            try:
                start = t.find('{')
                end = t.rfind('}') + 1
                obj = json.loads(t[start:end])
                if isinstance(obj, dict):
                    return obj
            except Exception:
                pass
        # Fallback
        return {"final_response": text}

    def _call_tool(self, tool: Any, args: Dict[str, Any]) -> Any:
        """Invoke a tool that may be a LangChain Tool, a simple callable, or have .invoke/.run."""
        # LangChain Tool with .invoke
        if hasattr(tool, 'invoke') and callable(getattr(tool, 'invoke')):
            return tool.invoke(args)
        # LangChain Tool with .run (expects str)
        if hasattr(tool, 'run') and callable(getattr(tool, 'run')):
            # Prefer text arg if present, otherwise serialize
            payload = args.get('text') if isinstance(args, dict) else args
            if payload is None:
                payload = json.dumps(args)
            return tool.run(payload)
        # Plain callable
        if callable(tool):
            try:
                sig = inspect.signature(tool)
                if len(sig.parameters) == 1 and next(iter(sig.parameters.values())).kind in (
                    inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY
                ) and 'text' not in sig.parameters:
                    # Single arg callable; prefer passing the text or the whole dict
                    payload = args.get('text') if isinstance(args, dict) and 'text' in args else args
                    return tool(payload)
                return tool(**args) if isinstance(args, dict) else tool(args)
            except Exception:
                # Last resort
                payload = args.get('text') if isinstance(args, dict) else args
                return tool(payload)
        raise TypeError(f"Unsupported tool type: {type(tool)}")

    def stream(self, prompt: str) -> Any:
        self.logger.debug(f"stream called")
        # Prefer streaming and reconstructing the final text from feedback tokens
        full_text_parts: List[str] = []
        try:
            for partial in self.generate_response(prompt, stream=True):
                try:
                    text_piece = getattr(partial, 'text', '') or getattr(partial, 'partial_response', '')
                    if text_piece:
                        full_text_parts.append(text_piece)
                except Exception as e:
                    self.logger.error(f"Error collecting partial text: {e}")
        except Exception as e:
            self.logger.error(f"Streaming failed: {e}")
        # If streaming yielded nothing, fall back to non-streaming result text
        full_text = ''.join(full_text_parts)
        if not full_text:
            self.logger.debug("No partial text collected, falling back to non-streaming result")
            result, status = self.generate_response(prompt, stream=False)
            self.logger.debug(f"Stream fallback result: {result}, status: {status}")
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.logger.error("LLM response failed.")
                yield {"error": "LLM response failed."}
                return
            full_text = getattr(result.response, 'text', '') or ''
        # Parse output
        try:
            output = self._parse_output(full_text)
            self.logger.debug(f"Parsed output: {output}")
        except Exception as e:
            self.logger.error(f"Failed to parse output: {e}")
            output = {"final_response": full_text}
        yield output

    def stream_with_tools(self, prompt: Dict[str, Any]) -> Any:
        self.logger.debug(f"stream_with_tools called")
        for output in self.stream(prompt["msg"]):
            self.logger.debug(f"Output from stream: {output}")
            tool_calls = output.get("tool_calls", [])
            tool_responses = []
            if tool_calls:
                self.logger.debug(f"Tool calls found: {tool_calls}")
                for call in tool_calls:
                    name = call.get('name')
                    args = call.get('args', {})
                    # Only override if a specific tool was forced (not 'any')
                    if self.tool_choice and self.tool_choice != 'any':
                        name = self.tool_choice
                    func = self.tools.get(name)
                    if func:
                        try:
                            self.logger.debug(f"Invoking tool: {name} with args: {args}")
                            result = self._call_tool(func, args)
                            tool_responses.append({
                                'name': name,
                                'args': args,
                                'result': result
                            })
                            self.logger.debug(f"Tool result: {result}")
                        except Exception as e:
                            self.logger.error(f"Tool {name} raised exception: {e}")
                            tool_responses.append({
                                'name': name,
                                'args': args,
                                'error': str(e)
                            })
                    else:
                        self.logger.error(f"Tool {name} not registered.")
                        tool_responses.append({
                            'name': name,
                            'args': args,
                            'error': 'Tool not registered.'
                        })
            # Fallback: if no tool calls but we have a final_response and a say_phrase tool, auto speak
            if (not tool_calls) and self.auto_speak and ('final_response' in output) and output.get('final_response'):
                say_tool = self.tools.get('say_phrase') or self.tools.get('speak') or self.tools.get('tts')
                if say_tool:
                    try:
                        text = output.get('final_response', '')
                        self.logger.debug(f"Auto-speaking final_response via TTS tool: {text}")
                        result = self._call_tool(say_tool, {'text': text})
                        tool_responses.append({
                            'name': getattr(say_tool, 'name', 'say_phrase'),
                            'args': {'text': text},
                            'result': result
                        })
                    except Exception as e:
                        self.logger.error(f"Auto speak failed: {e}")
                        tool_responses.append({
                            'name': getattr(say_tool, 'name', 'say_phrase'),
                            'args': {'text': output.get('final_response', '')},
                            'error': str(e)
                        })
            output["tool_responses"] = tool_responses
            self.logger.debug(f"Yielding output with tool responses: {output}")
            yield output

    def bind_tools(self, tools, tool_choice=None):
        self.logger.debug(f"Binding tools: {[getattr(tool, 'name', None) or getattr(tool, '__name__', None) for tool in tools]}, tool_choice={tool_choice}")
        self.tool_choice = tool_choice
        for tool in tools:
            tool_name = getattr(tool, 'name', None) or getattr(tool, '__name__', None)
            if not tool_name:
                self.logger.error(f"Tool object {tool} has no 'name' or '__name__' attribute.")
                raise ValueError(f"Tool object {tool} has no 'name' or '__name__' attribute.")
            self.register_tool(tool_name, tool)
        return self
