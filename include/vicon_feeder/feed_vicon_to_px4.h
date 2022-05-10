#ifndef FEED_VICON_H
#define FEED_VICON_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

class FeedVicon
{

 public:

  FeedVicon(ros::NodeHandle& nh_input, ros::NodeHandle& nh_private_input);

  void viconDroneFrameCallback(const geometry_msgs::TransformStamped& vicon_msg);

  void viconGate1GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg);
  void viconGate2GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg);
  void viconGate3GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg);
  void viconGate4GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg);

  void loopCallback(const ros::TimerEvent& event);

 private:

  ros::NodeHandle nh_; // ROS node handle
  ros::NodeHandle nh_private_; // ROS node handle
  ros::Timer feedingloop_timer_;  //Timer
  
  ros::Publisher vicon_odometry_publisher_;
  ros::Publisher vicon_pose_publisher_;
  ros::Publisher gate_pose_array_publisher_;
  ros::Publisher vicon_visualize_pose_publisher_;

  ros::Subscriber vicon_subscriber_;
  ros::Subscriber vicon_gate1_groundtruth_subscriber_;
  ros::Subscriber vicon_gate2_groundtruth_subscriber_;
  ros::Subscriber vicon_gate3_groundtruth_subscriber_;
  ros::Subscriber vicon_gate4_groundtruth_subscriber_;

  std::string bodyframe_name_, mav_name_;

  std::vector<geometry_msgs::Twist> velocities_queue_;
  geometry_msgs::Point previous_position_;
  geometry_msgs::Point previous_orientation_;
  ros::Time time_old_;

  geometry_msgs::Twist filter();
  void push(double vx, double vy, double vz, double p, double q, double r);

  geometry_msgs::Pose pose_gate_1_;
  geometry_msgs::Pose pose_gate_2_;
  geometry_msgs::Pose pose_gate_3_;
  geometry_msgs::Pose pose_gate_4_;

  bool publish_groundtruth_pose_;

  void publishGateArray(geometry_msgs::Pose gatepose_1);
  void publishGateArray(geometry_msgs::Pose gatepose_1, geometry_msgs::Pose gatepose_2, geometry_msgs::Pose gatepose_3, geometry_msgs::Pose gatepose_4);





};  // End of Class

#endif /* FEED_VICON_H */