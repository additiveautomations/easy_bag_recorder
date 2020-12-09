from contextlib import contextmanager
from typing import List

import actionlib
from rosbag import Bag
from easy_bag_recorder.msg import RecordAction, RecordGoal


class EasyBagRecorder:
    bag_path: str = None

    def __init__(self, action: str = "/record") -> None:
        self.client = actionlib.SimpleActionClient(action, RecordAction)
        self.client.wait_for_server()

    @contextmanager
    def record(self, topics: List[str]):
        assert isinstance(topics, list) and len(topics) > 0

        self.client.send_goal(RecordGoal(topics=topics))

        try:
            yield
        finally:
            self.client.cancel_goal()
            path = self.client.get_goal_status_text()

            # Wait for cancel to take effect
            while path == "This goal has been accepted by the simple action server":
                path = self.client.get_goal_status_text()

            if not path:
                raise SystemError("Recording failed")

            self.bag_path = path

    def get_bag(self) -> Bag:
        return Bag(self.bag_path)
