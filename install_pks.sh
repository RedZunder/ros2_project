#!/usr/bin/env bash

deps=(curl python3 colcon ros2-apt-source software-properties-common)

#Install and set up locale

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#Install necessary packages

sudo apt install -y "${deps[@]}"
sudo add-apt-repository universe
sudo apt update

snaps=(ros-humble-desktop ros-humble-ros-base)
sudo snap install "${snaps[@]}"
sudo apt install ros-dev-tools


