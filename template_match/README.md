# Pointcloud_Registration_ros
## Introduction
A ROS package for pointcloud filtering, segmentation(cluster extraction), coarse registration and ICP registration, used in bin picking question.
## Procedure
- First, read the model file, and subscirbe the /camera/depth/points topic, convert ros topic of pointclouds to pcl format.
- Second, filter the pointclouds, save the clouds of target objects.
- Third, doing the cluster extraction for all the targets.
- Then do the coarse registration for each object, using FPFH and SCA-IA.
- Based on the result of coarse registration, doing the icp registration for each object, then get the results of pose.
## Environment
- ubuntu 16.04
- ROS kinetic
- pcl
