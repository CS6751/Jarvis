Jarvis
============

A (hopefully) extensible project to create an intelligent collaborative robot arm 

Packages are standalone modules to execute different aspects of the task

File Organization (the standard ROS way) - this way, you can pull the entire git directory into your catkin workspace and each package will act like a different package 

Jarvis (main github folder)
|--package_1
    |--src
    |--package.xml
    |--readme
    |--CMakeLists.txt
    |--launch
    |--msg
    |--srv
    |etc
|--package_2
    |--src
    |--package.xml
    |--readme
    |--CMakeLists.txt
    |--launch
    |--msg
    |--srv
    |etc


