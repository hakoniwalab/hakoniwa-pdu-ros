import os
import select
import signal
import subprocess
import sys
import textwrap
import time


def _subprocess_environment() -> dict[str, str]:
    env = os.environ.copy()
    python_paths = [path for path in sys.path if path]
    inherited_python_path = env.get("PYTHONPATH")
    if inherited_python_path:
        python_paths.extend(inherited_python_path.split(os.pathsep))
    env["PYTHONPATH"] = os.pathsep.join(dict.fromkeys(python_paths))
    return env


def test_bridge_ctrl_c_stops_endpoint_before_ros_shutdown():
    script = textwrap.dedent(
        """
        import rclpy
        import hakoniwa_pdu_ros.bridge_node as bridge_node


        class Config:
            endpoint_config = "unused-endpoint.json"
            bindings = []


        class FakeEndpointManager:
            def __init__(self, _endpoint_config):
                pass

            def start(self):
                pass

            def stop(self):
                if not rclpy.ok():
                    print("STOP_AFTER_ROS_SHUTDOWN", flush=True)
                    raise RuntimeError("endpoint dispatch stopped after ROS shutdown")
                print("STOP_BEFORE_ROS_SHUTDOWN", flush=True)


        ready = False


        class ReadyExecutor(bridge_node.SingleThreadedExecutor):
            def spin_once(self, timeout_sec=None):
                global ready
                if not ready:
                    print("READY", flush=True)
                    ready = True
                return super().spin_once(timeout_sec=timeout_sec)


        bridge_node.validate_zenoh_io_for_config = lambda _path: None
        bridge_node.load_config = lambda _path: Config()
        bridge_node.PduEndpointManager = FakeEndpointManager
        bridge_node.SingleThreadedExecutor = ReadyExecutor

        bridge_node.run("unused-binding.json")
        print(f"FINAL_RCLPY_OK={rclpy.ok()}", flush=True)
        """
    )

    process = subprocess.Popen(
        [sys.executable, "-c", script],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=_subprocess_environment(),
    )
    assert process.stdout is not None

    lines = []
    deadline = time.monotonic() + 10.0
    ready = False
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        remaining = max(0.0, deadline - time.monotonic())
        readable, _, _ = select.select([process.stdout], [], [], remaining)
        if not readable:
            continue
        line = process.stdout.readline()
        if not line:
            continue
        lines.append(line)
        if "READY" in line:
            ready = True
            break

    if not ready:
        tail, _ = process.communicate(timeout=5.0)
        output = "".join(lines) + tail
        raise AssertionError(f"bridge did not become ready before timeout:\n{output}")

    os.kill(process.pid, signal.SIGINT)
    tail, _ = process.communicate(timeout=10.0)
    output = "".join(lines) + tail

    assert process.returncode == 0, output
    assert "STOP_BEFORE_ROS_SHUTDOWN" in output
    assert "STOP_AFTER_ROS_SHUTDOWN" not in output
    assert "FINAL_RCLPY_OK=False" in output
    assert "Traceback" not in output
    assert "RCLError" not in output
    assert "publisher context is invalid" not in output
