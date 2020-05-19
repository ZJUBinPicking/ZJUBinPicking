/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "probot_grasping/grasping_demo.h"

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x,
                           float pregrasp_y, float pregrasp_z, float length,
                           float breadth)
    : it_(n_),
      armgroup("manipulator"),
      grippergroup("gripper"),
      vMng_(length, breadth) {
  this->nh_ = n_;

  try {
    this->tf_camera_to_robot.waitForTransform(
        "/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
  } catch (tf::TransformException &ex) {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  try {
    this->tf_camera_to_robot.lookupTransform(
        "/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false;

  this->pregrasp_x = pregrasp_x;
  this->pregrasp_y = pregrasp_y;
  this->pregrasp_z = pregrasp_z;
  arm_pub = n_.advertise<bpmsg::arm_state>("/arm_state", 30);
  trans_sub = n_.subscribe("/goal_translation", 30, &GraspingDemo::posCb, this);
  armgroup.allowReplanning(true);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  state.pick_state = state.READY_FOR_PICK;
  state.pick_index = -1;
  arm_pub.publish(state);
  ros::WallDuration(5.0).sleep();
  ROS_INFO_STREAM("Getting into the Grasping Position....");
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);

  // Subscribe to input video feed and publish object location
  // image_sub_ =
  //     it_.subscribe("/camera/color/image_raw", 1, &GraspingDemo::imageCb,
  //     this);

  // //设置位置(单位：米)和姿态（单位：弧度）的允许误差
  // armgroup.setGoalPositionTolerance(0.001);
  // armgroup.setGoalOrientationTolerance(0.01);
  armgroup.setMaxAccelerationScalingFactor(0.2);
  armgroup.setMaxVelocityScalingFactor(0.8);
  // //设置允许的最大速度和加速度
  // armgroup.setMaxAccelerationScalingFactor(0.2);
  // armgroup.setMaxVelocityScalingFactor(0.2);
}

void GraspingDemo::posCb(bpmsg::pose msg) {
  // if (!grasp_running) {
  this->target_pos.clear();
  this->target_angle.clear();
  // ROS_WARN("pos");
  if (msg.if_detect == msg.DETECTSUCCESS) {
    // ROS_WARN("start call");
    for (int i = 0; i < 3; i++) {
      this->target_angle.push_back(msg.target_angle[i]);
    }
    obj_camera_frame.setZ(-msg.target_pos[1]);
    obj_camera_frame.setY(-msg.target_pos[0]);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    // std::cout << " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX()
    //           << std::endl;
    // std::cout << " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY()
    //           << std::endl;
    // std::cout << " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ()
    //           << std::endl;
    this->target_pos.push_back(obj_robot_frame.getX());
    this->target_pos.push_back(obj_robot_frame.getY());
    this->target_pos.push_back(obj_robot_frame.getZ());
    this->pick_index = msg.target_index;
    this->target_num = msg.object_num;
    this->detect_state = msg.if_detect;
    // ROS_WARN("end call");

  } else {
    ROS_WARN("nothing");
    this->detect_state = msg.if_detect;
  }
  // }
}

void GraspingDemo::imageCb(const sensor_msgs::ImageConstPtr &msg) {
  if (!grasp_running) {
    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ROS_INFO("Image Message Received");
    float obj_x, obj_y;
    vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

    // Temporary Debugging
    std::cout << " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout << " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout << " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX()
              << std::endl;
    std::cout << " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY()
              << std::endl;
    std::cout << " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ()
              << std::endl;
  }
}

void GraspingDemo::attainPosition(float x, float y, float z) {
  // ROS_INFO("The attain position function called");

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link_2");
  visual_tools.deleteAllMarkers();

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;

  // Starting Postion before picking
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  armgroup.setPoseTarget(target_pose1);

  /* Uncomment Following section to visualize in rviz */
  // We can print the name of the reference frame for this robot.
  // ROS_INFO("Reference frame: %s", armgroup.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  // ROS_INFO("Reference frame: %s", armgroup.getEndEffectorLink().c_str());

  // ROS_INFO("Group names: %s",  armgroup.getName().c_str());

  /*ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? ""
  : "FAILED");

   const robot_state::JointModelGroup *joint_model_group =
  armgroup.getCurrentState()->getJointModelGroup("arm");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");*/

  armgroup.move();
}

void GraspingDemo::attainObject() {
  // ROS_INFO("The attain Object function called");
  cout << "pos" << target_pos[0] << " " << target_pos[1] << endl;
  attainPosition(target_pos[0], target_pos[1], target_pos[2] + grasp_y);

  // Open Gripper
  ros::WallDuration(0.5).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  // Slide down the Object
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Quaternion target_angle1;

  tf::Quaternion quat;
  tf::quaternionMsgToTF(armgroup.getCurrentPose().pose.orientation, quat);

  double roll, pitch, yaw;                       //定义存储r\p\y的容器
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //进行转换
  tf2::Quaternion orientation;
  // target_angle[2] = target_angle[2];
  // if (abs(target_angle[2]) > 1.57) {
  //   ROS_WARN("big!!!!! ");
  //   if (target_angle[2] > 0)
  //     target_angle[2] = target_angle[2] - 3.1415926;
  //   else if (target_angle[2] < 0)
  //     target_angle[2] = target_angle[2] + 3.1415926;
  // }
  orientation.setRPY(1.57, 1.57, -target_angle[0]);
  ROS_WARN("angle info : %f, %f,%f", 1.57, 1.57, -target_angle[0]);
  target_pose1.orientation.x = orientation.getX();
  target_pose1.orientation.y = orientation.getY();
  target_pose1.orientation.z = orientation.getZ();
  target_pose1.orientation.w = orientation.getW();

  // target_pose1.Quaternion = target_angle;
  target_pose1.position = currPose.pose.position;

  if ((fabs(target_angle[2]) - 1.5) / 1.5 < hor_ratio)
    target_pose1.position.z = obj_robot_frame.getZ() - hor_grasp_z;
  else if ((fabs(target_angle[2]) - 0.62) / 0.62 < ver_ratio)
    target_pose1.position.z = obj_robot_frame.getZ() - ver_grasp_z;
  else
    target_pose1.position.z =
        obj_robot_frame.getZ() - (ver_grasp_z + hor_grasp_z) / 2;

  // cout << "grasp_z" << grasp_z << endl;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::grasp() {
  // ROS_INFO("The Grasping function called");

  ros::WallDuration(0.5).sleep();
  grippergroup.setNamedTarget("close");
  grippergroup.move();
}

void GraspingDemo::lift() {
  // ROS_INFO("The lift function called");

  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  // Starting Postion after picking
  target_pose1.position.z = target_pose1.position.z + grasp_x;

  // if (rand() % 2) {
  // target_pose1.position.y = target_pose1.position.y + grasp_y;
  // } else {
  //   target_pose1.position.y = target_pose1.position.y - grasp_y;
  // }

  armgroup.setPoseTarget(target_pose1);
  armgroup.move();

  // Open Gripper
  ros::WallDuration(0.5).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();
  // target_pose1.position.z = target_pose1.position.z + 0.06;
  // armgroup.setPoseTarget(target_pose1);
  // armgroup.move();
}

void GraspingDemo::goHome() {
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  // Go to Home Position
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);
  attainPosition(homePose.pose.position.x, homePose.pose.position.y,
                 homePose.pose.position.z);
  // ros::WallDuration(1.0).sleep();
}

void GraspingDemo::initiateGrasping() {
  state.pick_state = state.READY_FOR_PICK;
  state.pick_index = -1;
  arm_pub.publish(state);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(0.5).sleep();
  cout << "init" << endl;
  homePose = armgroup.getCurrentPose();

  if (detect_state) {
    ROS_INFO_STREAM("Approaching the Object....");
    attainObject();
    state.pick_state = state.PICKING;
    state.pick_index = this->pick_index;
    arm_pub.publish(state);
    ROS_INFO_STREAM("Attempting to Grasp the Object now..");
    grasp();
    armgroup.setMaxAccelerationScalingFactor(0.02);
    armgroup.setMaxVelocityScalingFactor(0.1);
    ROS_INFO_STREAM("Lifting the Object....");
    lift();
    armgroup.setMaxAccelerationScalingFactor(0.2);
    armgroup.setMaxVelocityScalingFactor(0.8);
    // ROS_INFO_STREAM("Going back to home position....");
    // goHome();
  }
  grasp_running = false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_grasping");
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
  ros::NodeHandle n;

  if (!n.getParam("probot_grasping/table_length", length)) length = 0.3;
  if (!n.getParam("probot_grasping/table_breadth", breadth)) breadth = 0.3;
  if (!n.getParam("probot_grasping/pregrasp_x", pregrasp_x)) pregrasp_x = 0.20;
  if (!n.getParam("probot_grasping/pregrasp_y", pregrasp_y)) pregrasp_y = -0.17;
  if (!n.getParam("probot_grasping/pregrasp_z", pregrasp_z)) pregrasp_z = 0.28;

  GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);

  // if (!n.getParam("probot_grasping/grasp_x", simGrasp.grasp_x))
  //   simGrasp.grasp_x = 0.02;
  // if (!n.getParam("probot_grasping/grasp_y", simGrasp.grasp_x))
  //   simGrasp.grasp_y = 0.06;
  // if (!n.getParam("probot_grasping/grasp_z", simGrasp.grasp_x))
  //   simGrasp.grasp_z = 0.03;
  ros::param::get("~grasp_x", simGrasp.grasp_x);
  ros::param::get("~grasp_y", simGrasp.grasp_y);
  ros::param::get("~hor_grasp_z", simGrasp.hor_grasp_z);
  ros::param::get("~ver_grasp_z", simGrasp.ver_grasp_z);

  ros::param::get("~hor_ratio", simGrasp.hor_ratio);
  ros::param::get("~ver_ratio", simGrasp.ver_ratio);
  ROS_WARN("grasp info : %f, %f,%f", simGrasp.grasp_x, simGrasp.grasp_y,
           simGrasp.ver_grasp_z);
  ROS_INFO_STREAM("Waiting for five seconds..");

  // ros::WallDuration(5.0).sleep();
  while (ros::ok()) {
    // Process image callback
    ros::spinOnce();

    simGrasp.initiateGrasping();
  }
  return 0;
}
