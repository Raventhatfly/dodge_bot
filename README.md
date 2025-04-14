# Dodgebot Respotory

### Clone the repo
```shell
git clone --recurse-submodules https://github.com/Raventhatfly/dodge_bot.git
```

### Install the required dependencies
First install the rosdep dependencies:
```shell
rosdep install --from-paths src --ignore-src -r -y
```
If using the realsense camera, additional setup must be executed.
Go to the [Realsense SDK Installation Guide](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide) and add udev rules for realsense cameras.
### Build the Repo
```shell
colcon build
```