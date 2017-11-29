FROM ubuntu:16.04

RUN apt-get update
RUN apt-get install -y git cmake g++
RUN apt-get install -y python-pip python-empy
RUN apt-get install -y libboost-all-dev
RUN pip install --user -U pip catkin-pkg catkin-tools vcstools vcstool wstool
RUN export PATH=$HOME/.local/bin:$PATH
RUN mkdir -p ~/isir/orocos-2.9_ws/src
RUN cd ~/isir/orocos-2.9_ws/src
RUN wstool init
RUN wstool merge https://raw.githubusercontent.com/syroco/rtt_orca/master/config/orocos_toolchain-2.9.rosinstall
RUN wstool update
RUN cd ~/isir/orocos-2.9_ws
RUN catkin config --init --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_LUA_RTT=OFF
RUN catkin build
