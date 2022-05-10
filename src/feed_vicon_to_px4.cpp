#include <vicon_feeder/feed_vicon_to_px4.h>

FeedVicon::FeedVicon(ros::NodeHandle& nh_input, ros::NodeHandle& nh_private_input) : // default constructor for
nh_(nh_input), nh_private_(nh_private_input)
{
  // State estimator: Vicon callback and feed vicon pose to PX4
  // TODO: use VIO insteads of Vicon
  // load params from launch file

  feedingloop_timer_ = nh_.createTimer(ros::Duration(1.0), &FeedVicon::loopCallback, this); // loop rate: 1 HZ


  // vicon_subscriber_ = nh_.subscribe("/vicon/ida_racer/ida_racer", 1, &FeedVicon::viconDroneFrameCallback, this);
  vicon_subscriber_ = nh_.subscribe("/drone_vicon_pose", 1, &FeedVicon::viconDroneFrameCallback, this);
  vicon_gate1_groundtruth_subscriber_ = nh_.subscribe("/vicon/gate_1/gate_1", 1, &FeedVicon::viconGate1GroundTruthCallback, this);
  vicon_gate2_groundtruth_subscriber_ = nh_.subscribe("/vicon/gate_2/gate_2", 1, &FeedVicon::viconGate2GroundTruthCallback, this);
  vicon_gate3_groundtruth_subscriber_ = nh_.subscribe("/vicon/gate_3/gate_3", 1, &FeedVicon::viconGate3GroundTruthCallback, this);
  vicon_gate4_groundtruth_subscriber_ = nh_.subscribe("/vicon/gate_4/gate_4", 1, &FeedVicon::viconGate4GroundTruthCallback, this);

  // vicon_odometry_publisher_ = nh_.advertise<nav_msgs::Odometry>("/ida/odometry", 1); 
  vicon_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);  
  vicon_visualize_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/groundtruth_for_visualization_pose", 1);  
  //note: EKF2 in pixhawk 4 only accept /mavros/vision_pose/pose. If using LPE, we can use /mavros/mocap/pose

  gate_pose_array_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("/ground_truth/gate_pose_array", 1);  

  nh_private_.param<bool>("publish_groundtruth_pose_to_drone", publish_groundtruth_pose_, false);
  
}

// Timer to publish gate array
void FeedVicon::loopCallback(const ros::TimerEvent& event){
  FeedVicon::publishGateArray(pose_gate_1_, pose_gate_2_, pose_gate_3_, pose_gate_4_);
  // FeedVicon::publishGateArray(pose_gate_3_);
}


void FeedVicon::viconDroneFrameCallback(const geometry_msgs::TransformStamped& vicon_msg) {
  // publish vicon to PX4
  geometry_msgs::PoseStamped pose_msg;
  // pose_msg.header = vicon_msg.header;
  // pose_msg.header.seq = vicon_msg.header.seq;
  pose_msg.header.frame_id = "map";
  pose_msg.header.stamp = ros::Time::now();

  pose_msg.pose.position.x = vicon_msg.transform.translation.x;
  pose_msg.pose.position.y = vicon_msg.transform.translation.y;
  pose_msg.pose.position.z = vicon_msg.transform.translation.z;
  pose_msg.pose.orientation = vicon_msg.transform.rotation;

  if ( publish_groundtruth_pose_ ){
    vicon_pose_publisher_.publish(pose_msg);
  }
  else{
    vicon_visualize_pose_publisher_.publish(pose_msg);
  }
}

void FeedVicon::viconGate1GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg) {
  // Retrieving pose of gate 1;
  pose_gate_1_.position.x = vicon_msg.transform.translation.x;
  pose_gate_1_.position.y = vicon_msg.transform.translation.y;
  pose_gate_1_.position.z = vicon_msg.transform.translation.z;
  pose_gate_1_.orientation = vicon_msg.transform.rotation;
}

void FeedVicon::viconGate2GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg) {
  // Retrieving pose of gate 1;
  pose_gate_2_.position.x = vicon_msg.transform.translation.x;
  pose_gate_2_.position.y = vicon_msg.transform.translation.y;
  pose_gate_2_.position.z = vicon_msg.transform.translation.z;
  pose_gate_2_.orientation = vicon_msg.transform.rotation;
}

void FeedVicon::viconGate3GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg) {
  // Retrieving pose of gate 1;
  pose_gate_3_.position.x = vicon_msg.transform.translation.x;
  pose_gate_3_.position.y = vicon_msg.transform.translation.y;
  pose_gate_3_.position.z = vicon_msg.transform.translation.z;
  pose_gate_3_.orientation = vicon_msg.transform.rotation;
}

void FeedVicon::viconGate4GroundTruthCallback(const geometry_msgs::TransformStamped& vicon_msg) {
  // Retrieving pose of gate 1;
  pose_gate_4_.position.x = vicon_msg.transform.translation.x;
  pose_gate_4_.position.y = vicon_msg.transform.translation.y;
  pose_gate_4_.position.z = vicon_msg.transform.translation.z;
  pose_gate_4_.orientation = vicon_msg.transform.rotation;
}

void FeedVicon::publishGateArray(geometry_msgs::Pose gatepose_1, geometry_msgs::Pose gatepose_2, geometry_msgs::Pose gatepose_3, geometry_msgs::Pose gatepose_4){
  // push gate1 into pose array. Feel free to extend it
  geometry_msgs::PoseArray gate_pose_array_msgs;
  gate_pose_array_msgs.header.stamp = ros::Time::now();
  gate_pose_array_msgs.header.frame_id = "map";
  gate_pose_array_msgs.poses.push_back(gatepose_1);
  gate_pose_array_msgs.poses.push_back(gatepose_2);
  gate_pose_array_msgs.poses.push_back(gatepose_3);
  gate_pose_array_msgs.poses.push_back(gatepose_4);
  gate_pose_array_publisher_.publish(gate_pose_array_msgs);

}

void FeedVicon::publishGateArray(geometry_msgs::Pose gatepose_1){
  // push gate1 into pose array. Feel free to extend it
  geometry_msgs::PoseArray gate_pose_array_msgs;
  gate_pose_array_msgs.header.stamp = ros::Time::now();
  gate_pose_array_msgs.header.frame_id = "map";
  gate_pose_array_msgs.poses.push_back(gatepose_1);
  gate_pose_array_publisher_.publish(gate_pose_array_msgs);

}

// Main node
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "vicon_estimator_node");

  ros::NodeHandle n("");
  ros::NodeHandle n_private("~");

  // Implement with a defined waypoints: start waypoint, land waypoint and a list of gate waypoints
  FeedVicon feed_vicon(n, n_private);

  // Spin
  ros::spin ();
  return 0;
}