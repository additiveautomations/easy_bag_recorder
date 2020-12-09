# Easy Bag Recorder

Simple ROS action server for recording ROS bags in serial.

Loosely based on the work of Esteve Fernandez on https://github.com/osrf/nodelet_rosbag

The limited scope of this module is to enable a single recording running at any one time. To achieve this the concept of a ROS action has been slightly abused. Preempting (or cancelling) a goal stops the recording.

## Synopsis

Start the action server:
```
$ rosrun easy_bag_recorder easy_bag_recorder _bag_output_dir:=/tmp/
[ INFO] [1607431121.966600102]: Easy Bag Recorder started
[ INFO] [1607431121.966600102]: Writing bags to /tmp
```

Start recording from Python:

```python
import rospy
import actionlib
from rosbag import Bag
from easy_bag_recorder.msg import RecordAction, RecordActionGoal, RecordGoal

rospy.init_node("example")
client = actionlib.SimpleActionClient("/record", RecordAction)
client.wait_for_server()

# starts recording the provided topics
client.send_goal(RecordGoal(topics=["/hello"]))
```

Server output:
```
[ INFO] [1607431121.966600805]: Recording to /tmp/1607431121.bag
[ INFO] [1607431121.975548748]: Recording topics: /hello, 
[ INFO] [1607431121.977324248]: Subscribed to 1 topics
[ INFO] [1607431127.247674839]: Stopping recording
```

Stop recording (Python):

```python
# stops recording
client.cancel_goal();

# Path on file system of the bag
bag_path = client.get_goal_status_text();
my_ros_bag = Bag(bag_path)
```
