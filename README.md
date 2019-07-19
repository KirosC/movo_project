# Kinova Movo Simple Pick and Place
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

## Introduction
This is a simple project using [Kinova Movo](https://www.kinovarobotics.com/en/products/mobile-manipulators) to execute a simple pick and place (or stack) action.

[Demo Video](https://vimeo.com/348968986)

## Prerequisites
* Ubuntu 14.04
* ROS Indigo

Detailed installing procedure please visit the [setup instruction](https://github.com/Kinovarobotics/kinova-movo/wiki/1.-Setup-Instructions)

## Install
Simply copy to your workspace

Make sure you source the setup.bash or add the command to .bashrc

```
source ~/movo_ws/devel/setup.bash
```

## Usage

Start demo in Movo
```
roslaunch movo_project demo.launch
```
Start demo in stimulation (Gazebo)
```
roslaunch movo_project sim_demo.launch
```
Several utilitiy services can be used whiling running the demo
```
rosservice call util_goto_origin/util_tuck/util_grasp_pos
```

## Author

* **[KirosC](https://github.com/kirosc)**

## License

[MIT](LICENSE) © Kiros Choi
