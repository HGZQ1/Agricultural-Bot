"""One persistent Gazebo transport endpoint; physical commands are never retried."""
import json
import os
import re
import subprocess
import threading


def _bindings():
    from gz.transport13 import Node, NodeOptions
    from gz.msgs10.stringmsg_pb2 import StringMsg
    return Node, NodeOptions, StringMsg


class GazeboClient:
    def __init__(self, service, partition, timeout_s=5.0, allow_cli_fallback=False):
        self.service, self.partition = service, partition
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        # Gazebo reads this process's environment during Node initialization.
        # Other ROS/Gazebo processes and world assets are unaffected.
        os.environ['GZ_PARTITION'] = partition
        try:
            node_type, options_type, self.message_type = _bindings()
            options = options_type()
            options.partition = partition
            self.node = node_type(options)
            self.actual_partition = self.node.options.partition
            if self.actual_partition != partition:
                raise RuntimeError('native Gazebo partition differs from configured partition')
            self.backend = 'persistent_python'
        except (ImportError, OSError) as error:
            if not allow_cli_fallback:
                raise RuntimeError('Gazebo Python bindings required; source the ROS Jazzy vendor environment') from error
            self.node = None
            self.actual_partition = partition
            self.backend = 'explicit_cli_fallback'

    def request(self, payload):
        if not self._lock.acquire(timeout=0.5):
            raise RuntimeError('Gazebo transport transaction busy; no command was sent')
        try:
            if self.node is not None:
                request = self.message_type()
                request.data = json.dumps(payload)
                success, response = self.node.request(self.service, request,
                    self.message_type, self.message_type, round(self.timeout_s*1000))
                if not success:
                    raise RuntimeError('Gazebo harvest service did not reply within the bounded transport deadline')
                result = json.loads(response.data)
            else:
                # This backend is selected only once, explicitly at startup.
                # Never retry a physical command after an uncertain response.
                environment = os.environ.copy()
                environment['GZ_PARTITION'] = self.partition
                completed = subprocess.run(['gz', 'service', '-s', self.service,
                    '--reqtype', 'gz.msgs.StringMsg', '--reptype', 'gz.msgs.StringMsg',
                    '--timeout', str(round(self.timeout_s*1000)),
                    '--req', 'data: ' + json.dumps(json.dumps(payload))],
                    capture_output=True, text=True, timeout=self.timeout_s+1.0, env=environment)
                match = re.search(r'^data:\s*("(?:[^"\\]|\\.)*")', completed.stdout, re.MULTILINE)
                if completed.returncode or not match:
                    raise RuntimeError('Gazebo CLI harvest service unavailable: '+completed.stderr.strip()[:150])
                result = json.loads(json.loads(match.group(1)))
            if not isinstance(result, dict):
                raise RuntimeError('Gazebo harvest service returned an invalid result')
            return result
        finally:
            self._lock.release()
