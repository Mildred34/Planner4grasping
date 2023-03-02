# grasp_capture

# Prerequisite
    * [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
    * [Gazebo Ignition Garden](https://gazebosim.org/docs/garden/install_ubuntu)
    * [Moveit2](https://moveit.ros.org/install-moveit2/binary/)
    * rosdep
    * colcon tools
        ```bash
        sudo apt install python3-colcon-common-extensions
        ```

# Compile from source

If you want compile this from source, you should choose the Gazebo version. The default one is `garden`:

```bash
export GZ_VERSION=garden
# or
echo 'export GZ_VERSION=garden' > ~/.bashrc
source ~/.bashrc
```

Then create a workspace, clone the repo and compile it:

```bash
mkdir -p ~/ws/src
cd ~/ws/src
git clone git@github.com:Mildred34/Planner4grasping.git
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/ws
git submodule init
git submodule update # or do a git clone reccursively
colcon build
```

# Usage

## Video + Pictures


## Running


#### Executing the examples

