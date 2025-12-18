#!/usr/bin/env bash

deps=(curl python3 colcon software-properties-common)

#Install and set up locale

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#Install necessary packages

sudo apt install -y "${deps[@]}"
sudo add-apt-repository universe
sudo apt update
sudo apt install -y ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install -y ros-jazzy-desktop


