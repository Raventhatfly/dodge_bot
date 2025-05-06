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
If using the realsense camera, additional setup (before using rosdep) must be executed.
Because rosdep cannot locate realsense2, first ignore realsense. 
Go to the [Realsense SDK Installation Guide](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide) and add udev rules for realsense cameras.
### Build the Repo
```shell
colcon build --symlink-install
```
Or the scripts under `script` folder can be used.