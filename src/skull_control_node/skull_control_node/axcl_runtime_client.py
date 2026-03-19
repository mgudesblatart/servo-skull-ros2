"""
axcl_runtime_client.py

Manages the AXCL (llama.cpp-style) inference subprocess lifecycle.

The runtime is launched as a shell process keeping stdin/stdout open
between calls, so there's no process-start overhead per inference.
Character-by-character reads from stdout are fed into a Queue by a
background reader thread, and _read_until_prompt() blocks until it sees
the runtime's prompt marker string, collecting the full response.

Usage:
    client = AxclRuntimeClient(command="./llama-cli ...", ...)
    client.start()                    # Blocks until prompt marker seen
    output = client.generate("Hi")    # Blocks until next prompt marker
    client.close()                    # Sends "q", then terminates process
"""

import os
import queue
import subprocess
import threading
import time


class AxclRuntimeClient:
    def __init__(
        self,
        command: str,
        cwd: str | None = None,
        env_overrides: dict[str, str] | None = None,
        prompt_marker: str = "prompt >>",
        startup_timeout_sec: float = 120.0,
        response_timeout_sec: float = 60.0,
    ) -> None:
        """
        Args:
            command: Shell command string to launch the runtime.
            cwd: Working directory for the subprocess; None = inherit.
            env_overrides: Extra env vars to inject (e.g. SYSTEM_PROMPT).
            prompt_marker: String the runtime prints when ready for input.
            startup_timeout_sec: Max seconds to wait for the initial prompt on start().
            response_timeout_sec: Max seconds to wait for a response per generate() call.
        """
        self.command = command
        self.cwd = os.path.expanduser(cwd) if cwd else None
        self.env_overrides = env_overrides or {}
        self.prompt_marker = prompt_marker
        self.startup_timeout_sec = startup_timeout_sec
        self.response_timeout_sec = response_timeout_sec

        self.process: subprocess.Popen[str] | None = None
        self._reader_thread: threading.Thread | None = None
        # Char-by-char output from the subprocess; None sentinel signals EOF
        self._buffer_queue: queue.Queue[str | None] = queue.Queue()
        # Prevents concurrent generate() calls from interleaving
        self._request_lock = threading.Lock()

    def start(self) -> None:
        """
        Start the runtime subprocess and wait for the prompt marker.
        No-op if the process is already running.
        """
        if self.process is not None and self.process.poll() is None:
            return

        # Recreate the queue so any EOF sentinel from a previous reader thread
        # (which captured the old reference) never lands in this run's queue.
        self._buffer_queue = queue.Queue()

        process_env = os.environ.copy()
        process_env.update(self.env_overrides)

        self.process = subprocess.Popen(
            self.command,
            cwd=self.cwd,
            shell=True,
            env=process_env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Merge stderr so runtime logs appear in stdout
            text=True,
            bufsize=0,  # Unbuffered — needed for char-by-char reads
        )

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self._read_until_prompt(self.startup_timeout_sec)

    def _reader_loop(self) -> None:
        """Background thread: read stdout one character at a time into the queue."""
        assert self.process is not None
        assert self.process.stdout is not None

        # Capture the queue reference at thread-start so that if start() later
        # reassigns self._buffer_queue for a new process run, this thread keeps
        # writing to the queue it was created for and cannot corrupt the new run.
        buf = self._buffer_queue

        while True:
            chunk = self.process.stdout.read(1)
            if chunk == "":
                # EOF — process exited
                buf.put(None)
                return
            buf.put(chunk)

    def _read_until_prompt(self, timeout_sec: float) -> str:
        """
        Drain chars from the buffer queue until the prompt marker appears.
        Raises TimeoutError if the deadline passes, or RuntimeError if the
        process exits before the marker is seen.
        """
        deadline = time.monotonic() + timeout_sec
        output = ""

        while time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            try:
                chunk = self._buffer_queue.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue

            if chunk is None:
                raise RuntimeError(
                    f"Runtime exited before prompt marker '{self.prompt_marker}'. "
                    f"Output so far:\n{output}"
                )

            output += chunk
            if self.prompt_marker in output:
                return output

        raise TimeoutError(
            f"Timed out waiting for runtime prompt marker '{self.prompt_marker}'. "
            f"Output so far:\n{output}"
        )

    def generate(self, prompt: str) -> str:
        """
        Send a prompt to the runtime and return the response text.

        Thread-safe (serialised by _request_lock). Calls start() if the
        process isn't running yet. Strips the trailing prompt marker from
        the returned text.
        """
        with self._request_lock:
            self.start()
            assert self.process is not None
            assert self.process.stdin is not None

            self.process.stdin.write(prompt + "\n")
            self.process.stdin.flush()

            output = self._read_until_prompt(self.response_timeout_sec)
            return self._strip_prompt_marker(output)

    def _strip_prompt_marker(self, output: str) -> str:
        """Remove the trailing prompt marker from a response string."""
        marker_index = output.rfind(self.prompt_marker)
        if marker_index != -1:
            output = output[:marker_index]
        return output.strip()

    def close(self) -> None:
        """
        Gracefully shut down the runtime subprocess.
        Sends "q" on stdin first (llama.cpp convention), then SIGTERM,
        then SIGKILL as a last resort.
        """
        if self.process is None:
            return

        if self.process.poll() is None and self.process.stdin is not None:
            try:
                self.process.stdin.write("q\n")
                self.process.stdin.flush()
            except Exception:
                pass

        try:
            self.process.terminate()
            self.process.wait(timeout=3)
        except Exception:
            try:
                self.process.kill()
                self.process.wait(timeout=1)
            except Exception:
                pass

        self.process = None