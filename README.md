RTT ORCA
========

This package contains orocos wrappers for the orca objects.

### Installation

* catkin
* orocos-rtt
* orocos-ocl
* gazebo


#### Catkin

Install pip https://pip.pypa.io/en/stable/installing/

```bash
pip install --user -U pip catkin-pkg catkin-tools vcstools vcstool wstool
```


#### Orocos toolchain 2.9

```bash
mkdir -p ~/isir/orocos-2.9_ws/src
cd ~/isir/orocos-2.9_ws/src
# Get all the packages
wstool init
wstool merge https://raw.githubusercontent.com/syroco/rtt_orca/master/config/orocos_toolchain-2.9.rosinstall
wstool update -j2
# Get the latest updates (OPTIONAL)
cd orocos_toolchain
git submodule foreach git checkout toolchain-2.9
git submodule foreach git pull
# Configure the workspace
cd ~/isir/orocos-2.9_ws/
# Install dependencies
source /opt/ros/kinetic/setup.bash
catkin config --init --install --extend /opt/ros/kinetic/ --cmake-args -DCMAKE_BUILD_TYPE=Release
# Build
catkin build
```

