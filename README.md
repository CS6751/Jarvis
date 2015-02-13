Jarvis
============

A (hopefully) extensible project to create an intelligent collaborative robot arm 

Summary for Perception Module
------
System Dependencies

- Point Cloud Library (PCL) : [http://pointclouds.org/downloads/](http://pointclouds.org/downloads/)

- Openni : [https://github.com/OpenNI/OpenNI](https://github.com/OpenNI/OpenNI)

- Eigen : [eigen.tuxfamily.org](http://eigen.tuxfamily.org/index.php?title=Main_Page)

ROS Package Dependencies

- Openni_Launch: [http://wiki.ros.org/openni_launch](http://wiki.ros.org/openni_launch)

- Openni_Tracker: [http://wiki.ros.org/openni_tracker](http://wiki.ros.org/openni_tracker)

- ROS Indigo (it may work with Hydro)


How to Use
-------
First make sure all dependencies have been installed and a vicon (or other localization system) is broadcasting the pose of the kinect. 

Run

	roslaunch perception.launch


This will open an RVIZ window, ensure the kinect localization is running, and open a slew of nodes. Most of them are associated with Openni_launch and openni_tracker.

Once the nodes have started, stand in front of the kinect in the psi "surrender" position so that the tracker can lock on to you. Hold the position until a green cylinder apears around your left forearm. 

Take the PCB (or other flat object) in your left hand and hold it still or move it very slowly. Two things should appear on the screen: a red blob where the algorithm thinks the grabbable object is and a large vector indicating the grasp. 

Dependencies
-----------------
Oh lord. 
In a delicious bit of irony, Kinect is built to work on windows while ROS exists only on linux, so you need a third party kinect driver. There are at least three:
- Openni
- Libfreenect
- Openni 2

Different packages depend on different drivers. Make sure you know which ones you need because installations of multiple drivers can interfere with each other. 

Code Walkthrough
-------------------
Full code (along with other components - human interaction, path planning, compliant motor control etc.): *link*





Gotchas and Warnings
-----------
Openni (the foundation that maintains the kinect drivers behind many 3rd-party Kinect applications) is defunct at the time of this writing (December 2014.) It was recently acquired by Apple and its website has been shut down. *link* There are some mirrors *link* 

Point cloud segmentation is *slow.* With this in mind, on future projects I would:
- Do as much processing with OpenCV and 2d images as possible.
- Use PCL GPU acceleration.*link* It is currently in beta.
- Set up the processing algorithm so it does segmentation as little as possible. This could include segmenting initially using a particle filter (or other type of filter) to track the object, instead of trying to find it anew every time step. Here's a good example of this tactic: *link*

I think some kind of implicit or explicit model is essential for robotic grasping, and AI in general. There are many ways to do this (see the grasping review paper.) 
<!--
but even the machine learning grasp generation approaches assume an isolated rigid object on a surface. Think about how human's do it  
-->




Resources
-----------
Main PCL site: 

Best Point Cloud Library resource I could find: http://robotica.unileon.es/mediawiki/index.php/PhD-3D-Object-Tracking

Openni 2: http://structure.io/openni

[pcl]:http://pointclouds.org/downloads/
[openni_launch]:http://wiki.ros.org/openni_launch

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


