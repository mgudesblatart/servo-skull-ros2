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
        self.command = command
        self.cwd = os.path.expanduser(cwd) if cwd else None
        self.env_overrides = env_overrides or {}
        self.prompt_marker = prompt_marker
        self.startup_timeout_sec = startup_timeout_sec
        self.response_timeout_sec = response_timeout_sec
        self.process: subprocess.Popen[str] | None = None
        self._reader_thread: threading.Thread | None = None
        self._buffer_queue: queue.Queue[str | None] = queue.Queue()
        self._request_lock = threading.Lock()

    def start(self) -> None:
        if self.process is not None and self.process.poll() is None:
            return

        process_env = os.environ.copy()
        process_env.update(self.env_overrides)

        self.process = subprocess.Popen(
            self.command,
            cwd=self.cwd,
            shell=True,
            env=process_env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=0,
        )

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self._read_until_prompt(self.startup_timeout_sec)

    def _reader_loop(self) -> None:
        assert self.process is not None
        assert self.process.stdout is not None

        while True:
            chunk = self.process.stdout.read(1)
            if chunk == "":
                self._buffer_queue.put(None)
                return
            self._buffer_queue.put(chunk)

    def _read_until_prompt(self, timeout_sec: float) -> str:
        deadline = time.monotonic() + timeout_sec
        output = ""

        while time.monotonic() < deadline:
            remaining = max(0.05, deadline - time.monotonic())
            try:
                chunk = self._buffer_queue.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue

            if chunk is None:
                raise RuntimeError(f"Runtime exited before prompt marker '{self.prompt_marker}' appeared. Output so far:\n{output}")

            output += chunk
            if self.prompt_marker in output:
                return output

        raise TimeoutError(f"Timed out waiting for runtime prompt marker '{self.prompt_marker}'. Output so far:\n{output}")

    def generate(self, prompt: str) -> str:
        with self._request_lock:
            self.start()
            assert self.process is not None
            assert self.process.stdin is not None

            self.process.stdin.write(prompt + "\n")
            self.process.stdin.flush()

            output = self._read_until_prompt(self.response_timeout_sec)
            return self._strip_prompt_marker(output)

    def _strip_prompt_marker(self, output: str) -> str:
        marker_index = output.rfind(self.prompt_marker)
        if marker_index != -1:
            output = output[:marker_index]
        return output.strip()

    def close(self) -> None:
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
            self.process.kill()

        self.process = None