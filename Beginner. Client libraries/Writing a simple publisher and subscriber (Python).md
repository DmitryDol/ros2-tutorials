# Writing a simple publisher and subscriber (Python)

## Background

In this tutorial, you will create nodes that pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

## Tasks

### 1. Create a package

Open a new terminal and source your ROS 2 installation so that `ros2` commands will work.

Navigate into the `ros2_ws` directory created in a previous tutorial.

Recall that packages should be created in the src directory, not the root of the workspace. So, navigate into `ros2_ws/src`, and run the package creation command:

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```
Your terminal will return a message verifying the creation of your package `py_pubsub` and all its necessary files and folders.

![030](images/030.png)

### 2. Write the publisher node

Navigate into `ros2_ws/src/py_pubsub/py_pubsub`. Recall that this directory is a Python package with the same name as the ROS 2 package it’s nested in.

Download the example talker code by entering the following command:

```bash
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

Now there will be a new file named `publisher_member_function.py` adjacent to `__init__.py`.

Open the file using your preferred text editor.

![031](images/031.png)

### 2.1 Examine the code

The first lines of code after the comments import `rclpy` so its `Node` class can be used.

```bash
import rclpy
from rclpy.node import Node
```

The next statement imports the built-in string message type that the node uses to structure the data that it passes on the topic.

```bash
from std_msgs.msg import String
```

These lines represent the node’s dependencies. Recall that dependencies have to be added to `package.xml`, which you’ll do in the next section.

Next, the `MinimalPublisher` class is created, which inherits from (or is a subclass of) `Node`.

```bash
class MinimalPublisher(Node):
```

Following is the definition of the class’s constructor. `super().__init__` calls the `Node` class’s constructor and gives it your node name, in this case `minimal_publisher`.

`create_publisher` declares that the node publishes messages of type `String` (imported from the `std_msgs.msg` module), over a topic named `topic`, and that the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

Next, a timer is created with a callback to execute every 0.5 seconds. `self.i` is a counter used in the callback.

```bash
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

`timer_callback` creates a message with the counter value appended, and publishes it to the console with `get_logger().info`.

```bash
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

Lastly, the main function is defined.

```bash
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

First the `rclpy` library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.

### 2.2. Add dependencies

Navigate one level back to the `ros2_ws/src/py_pubsub` directory, where the `setup.py`, `setup.cfg`, and `package.xml` files have been created for you.

Open `package.xml` with your text editor.

As mentioned in the previous tutorial, make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags:

```bash
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

After the lines above, add the following dependencies corresponding to your node’s import statements:

```bash
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

This declares the package needs `rclpy` and `std_msgs` when its code is executed.

Make sure to save the file.

![032](images/032.png)

### 2.3. Add an entry point

Open the `setup.py` file. Again, match the `maintainer`, `maintainer_email`, `description` and `license` fields to your `package.xml`:

```bash
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```

Add the following line within the `console_scripts` brackets of the `entry_points` field:

```bash
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

Don’t forget to save.

![033](images/033.png)

### 2.4. Check setup.cfg

The contents of the `setup.cfg` file should be correctly populated automatically, like so:

```bash
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
```

This is simply telling setuptools to put your executables in lib, because ros2 run will look for them there.

### 3. Write the subscriber node

Return to `ros2_ws/src/py_pubsub/py_pubsub` to create the next node. Enter the following code in your terminal:

```bash
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

Now the directory should have these files:

```bash
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

### 3.1. Examine the code

Open the `subscriber_member_function.py` with your text editor.

![034](images/034.png)

The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from the topics tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

```bash
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

The callback definition simply prints an info message to the console, along with the data it received. Recall that the publisher defines `msg.data = 'Hello World: %d' % self.i`

```bash
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

The `main` definition is almost exactly the same, replacing the creation and spinning of the publisher with the subscriber.

```bash
minimal_subscriber = MinimalSubscriber()

rclpy.spin(minimal_subscriber)
```

Since this node has the same dependencies as the publisher, there’s nothing new to add to `package.xml`. The `setup.cfg` file can also remain untouched.

### 3.3. Add an entry point

Reopen `setup.py` and add the entry point for the subscriber node below the publisher’s entry point. The `entry_points` field should now look like this:

```bash
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

Make sure to save the file, and then your pub/sub system should be ready.

### 4. Build and run

You likely already have the `rclpy` and `std_msgs` packages installed as part of your ROS 2 system. It’s good practice to run `rosdep` in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

Still in the root of your workspace, `ros2_ws`, build your new package:

```bash
colcon build --packages-select py_pubsub
```

Open a new terminal, navigate to `ros2_ws`, and source the setup files:

```bash
source install/setup.bash
```

Now run the talker node:

```bash
ros2 run py_pubsub talker
```

The terminal should start publishing info messages every 0.5 seconds, like so:

![035](images/035.png)

Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node:

```bash
ros2 run py_pubsub listener
```

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

![036](images/036.png)

Enter `Ctrl+C` in each terminal to stop the nodes from spinning.

## Summary

You created two nodes to publish and subscribe to data over a topic. Before running them, you added their dependencies and entry points to the package configuration files.
