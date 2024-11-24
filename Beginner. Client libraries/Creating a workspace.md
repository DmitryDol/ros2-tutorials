# Creating a workspace

## Background

A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

## Tasks

### 1. Source ROS 2 environment

Depending on how you installed ROS 2 (from source or binaries), and which platform you’re on, your exact source command will vary:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Create a new directory

Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name `ros2_ws`, for “development workspace”:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Another best practice is to put any packages in your workspace into the src directory. The above code creates a `src` directory inside `ros2_ws` and then navigates into it.

### 3. Clone a sample repo

Ensure you’re still in the `ros2_ws/src` directory before you clone.

A repo can have multiple branches. You need to check out the one that targets your installed ROS 2 distro. When you clone this repo, add the `-b` argument followed by that branch.

In the `ros2_ws/src` directory, run the following command:

```bash
git clone https://github.com/ros/ros_tutorials.git -b humble
```

Now `ros_tutorials` is cloned in your workspace. The `ros_tutorials` repository contains the `turtlesim` package, which we’ll use in the rest of this tutorial. The other packages in this repository are not built because they contain a `COLCON_IGNORE` file.

So far you have populated your workspace with a sample package, but it isn’t a fully-functional workspace yet. You need to resolve the dependencies first and then build the workspace.

### 4. Resolve dependencies

Before building the workspace, you need to resolve the package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.

From the root of your workspace (`ros2_ws`), run the following command:

```bash
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

If you already have all your dependencies, the console will return:

```bash
#All required rosdeps installed successfully
```

![003](images/003.png)

Packages declare their dependencies in the package.xml file. This command walks through those declarations and installs the ones that are missing.

### 5. Build the workspace with colcon

From the root of your workspace (`ros2_ws`), you can now build your packages using the command:

```bash
colcon build
```

The console will return the following message:

```bash
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]

Summary: 1 package finished [5.58s]
```

Once the build is finished, enter the command in the workspace root (~/ros2_ws):

```bash
ls
```

And you will see that colcon has created new directories:

```bash
build  install  log  src
```

The install directory is where your workspace’s setup files are, which you can use to source your overlay.

### 6. Source the overlay

Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it:

```bash
source /opt/ros/humble/setup.bash
```

Go into the root of your workspace:

```bash
cd ~/ros2_ws
```

In the root, source your overlay:

```bash
source install/local_setup.bash
```

Now you can run the turtlesim package from the overlay:

```bash
ros2 run turtlesim turtlesim_node
```

### 7. Modify the overlay

You can modify `turtlesim` in your overlay by editing the title bar on the turtlesim window. To do this, locate the `turtle_frame.cpp` file in `~/ros2_ws/src/ros_tutorials/turtlesim/src`. Open `turtle_frame.cpp` with your preferred text editor.

Find the function `setWindowTitle("TurtleSim");`, change the value `"TurtleSim"` to `"MyTurtleSim"`, and save the file.

Return to the first terminal where you ran `colcon build` earlier and run it again.

Return to the second terminal (where the overlay is sourced) and run turtlesim again:

```bash
ros2 run turtlesim turtlesim_node
```

You will see the title bar on the turtlesim window now says “MyTurtleSim”.

![004](images/004.png)

Even though your main ROS 2 environment was sourced in this terminal earlier, the overlay of your `ros2_ws` environment takes precedence over the contents of the underlay.

To see that your underlay is still intact, open a brand new terminal and source only your ROS 2 installation. Run turtlesim again:

```bash
ros2 run turtlesim turtlesim_node
```

![005](images/005.png)

You can see that modifications in the overlay did not actually affect anything in the underlay.

## Summary

In this tutorial, you sourced your main ROS 2 distro install as your underlay, and created an overlay by cloning and building packages in a new workspace. The overlay gets prepended to the path, and takes precedence over the underlay, as you saw with your modified turtlesim.

Using overlays is recommended for working on a small number of packages, so you don’t have to put everything in the same workspace and rebuild a huge workspace on every iteration.
