# Configuring environment

**Goal**: Learn how to prepare your ROS 2 environment

## 1. Source the setup files

Run this command on every new shell you open to have access to the ROS 2 commands, like so:

```shell
source /opt/ros/humble/setup.bash
```

## 2. Add sourcing to your shell startup script

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

> To undo this, locate your system`s shell startup script and remove the appended source command.

## 3. Check environment variables

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2. If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly set up using the following command:

```shell
printenv | grep -i ROS
```

Check that variables like ROS_DISTRO and ROS_VERSION are set.

```shell
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

### 3.1 The `ROS_DOMAIN_ID` variable

Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command:

```shell
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

### 3.2 The `ROS_LOCALHOST_ONLY` variable

By default, ROS 2 communication is not limited to localhost. ROS_LOCALHOST_ONLY environment variable allows you to limit ROS 2 communication to localhost only. This means your ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network. Using ROS_LOCALHOST_ONLY is helpful in certain settings, such as classrooms, where multiple robots may publish to the same topic causing strange behaviors. You can set the environment variable with the following command:

```shell
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```