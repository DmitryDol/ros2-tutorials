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
Your terminal will return a message verifying the creation of your package py_pubsub and all its necessary files and folders.


