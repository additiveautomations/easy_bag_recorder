from contextlib import contextmanager
import time
from typing import List

import actionlib
from rosbag import Bag
from easy_bag_recorder.msg import RecordAction, RecordGoal


class RecordingError(SystemError):
    pass


class EasyBagRecorder:
    bag_path: str = None

    def __init__(self, action: str = "/record") -> None:
        self.client = actionlib.SimpleActionClient(action, RecordAction)
        self.client.wait_for_server()
        self.recording = False

    def record(self, topics: List[str]):
        assert isinstance(topics, list) and len(topics) > 0

        if self.recording:
            raise RecordingError("Recording already in progress, stop()?")

        self.client.send_goal(RecordGoal(topics=topics))
        self.recording = True

    def stop(self) -> str:
        self.client.cancel_goal()
        self.recording = False

        path = self.client.get_goal_status_text()

        # Wait for cancel to take effect
        while path == "This goal has been accepted by the simple action server":
            time.sleep(0.1)
            path = self.client.get_goal_status_text()

        if not path:
            raise RecordingError("Recording failed")

        self.bag_path = path
        return path

    @contextmanager
    def record_session(self, topics: List[str]):
        self.record(topics)

        try:
            yield
        finally:
            self.stop()

    def get_bag(self) -> Bag:
        return Bag(self.bag_path)
