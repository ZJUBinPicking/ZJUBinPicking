#define PI 3.1415926536
#include <ros/ros.h>

#include <vector>

#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "std_msgs/Float32MultiArray.h"

std_msgs::Float32MultiArray position_msg;
std::vector<std_msgs::Float32MultiArray> pos_msg;
// std_msgs::Float32MultiArray speed_msg;
// std::vector<std_msgs::Float32MultiArray> vec_msg;

void ArmCallback(const control_msgs::FollowJointTrajectoryActionGoal& msg) {
  /** for position control  **/
  pos_msg.clear();
  for (int i = 0; i < (msg.goal.trajectory.points.size()); ++i) {
    position_msg.data.at(0) =
        msg.goal.trajectory.points[i].positions[0] * 30 * 180 / PI;
    position_msg.data.at(1) =
        msg.goal.trajectory.points[i].positions[1] * 205 * 180 / (3 * PI);
    position_msg.data.at(2) =
        msg.goal.trajectory.points[i].positions[2] * 50 * 180 / PI;
    position_msg.data.at(3) =
        msg.goal.trajectory.points[i].positions[3] * 125 * 180 / (2 * PI);
    position_msg.data.at(4) =
        msg.goal.trajectory.points[i].positions[4] * 125 * 180 / (2 * PI);
    position_msg.data.at(5) =
        msg.goal.trajectory.points[i].positions[5] * 200 * 180 / (9 * PI);
    position_msg.data.at(6) = 600.0;
    pos_msg.push_back(position_msg);

    ROS_INFO("joint 1: %f", msg.goal.trajectory.points[i].positions[0]);
    ROS_INFO("joint 2: %f", msg.goal.trajectory.points[i].positions[1]);
    ROS_INFO("joint 3: %f", msg.goal.trajectory.points[i].positions[2]);
    ROS_INFO("joint 4: %f", msg.goal.trajectory.points[i].positions[3]);
    ROS_INFO("joint 5: %f", msg.goal.trajectory.points[i].positions[4]);
    ROS_INFO("joint 6: %f", msg.goal.trajectory.points[i].positions[5]);
    ROS_INFO("all positions are set: points %d", i + 1);

    // maybe need to delay 0.1s
  }

  //
  // vec_msg.clear();
  // for (int i=0;  i<(msg.goal.trajectory.points.size());++i){
  //     speed_msg.data.at(0) =
  //     msg.goal.trajectory.points[i].velocities[0]*30*180/PI;
  //     speed_msg.data.at(1) =
  //     msg.goal.trajectory.points[i].velocities[1]*205*180/(3*PI);
  //     speed_msg.data.at(2) =
  //     msg.goal.trajectory.points[i].velocities[2]*50*180/PI;
  //     speed_msg.data.at(3) =
  //     msg.goal.trajectory.points[i].velocities[3]*125*180/(2*PI);
  //     speed_msg.data.at(4) =
  //     msg.goal.trajectory.points[i].velocities[4]*125*180/(2*PI);
  //     speed_msg.data.at(5) =
  //     msg.goal.trajectory.points[i].velocities[5]*200*180/(9*PI);
  //     vec_msg.push_back(speed_msg);
  //
  //     ROS_INFO("joint 1: %f",msg.goal.trajectory.points[i].velocities[0]);
  //     ROS_INFO("joint 2: %f",msg.goal.trajectory.points[i].velocities[1]);
  //     ROS_INFO("joint 3: %f",msg.goal.trajectory.points[i].velocities[2]);
  //     ROS_INFO("joint 4: %f",msg.goal.trajectory.points[i].velocities[3]);
  //     ROS_INFO("joint 5: %f",msg.goal.trajectory.points[i].velocities[4]);
  //     ROS_INFO("joint 6: %f",msg.goal.trajectory.points[i].velocities[5]);
  //     ROS_INFO("all velocities are set: points %d",i+1);
  //
  //
  //   //maybe need to delay 0.1s
  //   }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_node_angle");
  ros::NodeHandle nh;

  ros::Publisher position_pub =
      nh.advertise<std_msgs::Float32MultiArray>("position_chatter", 1000);
  // ros::Publisher speed_pub =
  // nh.advertise<std_msgs::Float32MultiArray>("speed_chatter", 1000);
  ros::Subscriber sub = nh.subscribe(
      "/probot_anno/arm_joint_controller/follow_joint_trajectory/goal", 1,
      ArmCallback);

  position_msg.data.push_back(0.0);
  position_msg.data.push_back(0.0);
  position_msg.data.push_back(0.0);
  position_msg.data.push_back(0.0);
  position_msg.data.push_back(0.0);
  position_msg.data.push_back(0.0);
  position_msg.data.push_back(0.0);

  // speed_msg.data.push_back(0.0);
  // speed_msg.data.push_back(0.0);
  // speed_msg.data.push_back(0.0);
  // speed_msg.data.push_back(0.0);
  // speed_msg.data.push_back(0.0);
  // speed_msg.data.push_back(0.0);

  while (ros::ok()) {
    ros::spinOnce();
    // for (int i = 0; i < pos_msg.size(); ++i) {
    //   position_pub.publish(pos_msg[i]);
    //   ros::Rate(10).sleep();
    // }
    if (!pos_msg.empty()) {
      position_pub.publish(pos_msg.back());
    }
    pos_msg.clear();
    // ros::Rate(10).sleep();

    // loop_rate.sleep();
  }
}