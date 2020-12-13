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
from easy_bag_recorder import EasyBagRecorder

rospy.init_node("example")
recorder = EasyBagRecorder()
recorder.record(["/robot/joint_states"])

# ...  do something with the robot
robot.raise_arm()

bag_path = recorder.stop()
bag = recorder.get_bag()  # rosbag.Bag
```

Server output:
```
[ INFO] [1607431121.966600805]: Recording to /tmp/1607431121.bag
[ INFO] [1607431121.975548748]: Recording topics: /hello, 
[ INFO] [1607431121.977324248]: Subscribed to 1 topics
[ INFO] [1607431127.247674839]: Stopping recording
```

Using a context manager:
```python
import rospy
from easy_bag_recorder import EasyBagRecorder

rospy.init_node("example")
recorder = EasyBagRecorder()

with recorder.record_session(["/robot/joint_states"])
    # ...  do something with the robot
    robot.raise_arm()

bag = recorder.get_bag()
```