// MIT License
//
// Copyright (c) 2022 Alvin Sun
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "vrpn_mocap/tracker.hpp"
#include "interfaces/msg/pose_rpy.hpp"


#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <string>

#include <Eigen/Geometry>
#include <Eigen/StdVector>


namespace vrpn_mocap
{

using geometry_msgs::msg::AccelStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using interfaces::msg::PoseRPY;
using namespace std::chrono_literals;

std::string Tracker::ValidNodeName(const std::string & tracker_name)
{
  // replace non alphanum characters with _
  const std::string alnum_name = std::regex_replace(tracker_name, std::regex("[^a-zA-Z0-9_]"), "_");
  // strip consecutive underscores
  const std::string node_name = std::regex_replace(alnum_name, std::regex("_+"), "_");

  return node_name;
}

Tracker::Tracker(const std::string & tracker_name)
: Node(ValidNodeName(tracker_name)),
  name_(tracker_name),
  multi_sensor_(declare_parameter("multi_sensor", false)),
  frame_id_(declare_parameter("frame_id", "world")),
  sensor_data_qos_(declare_parameter("sensor_data_qos", true)),
  use_vrpn_timestamps_(declare_parameter("use_vrpn_timestamps", false)),
  vrpn_tracker_(name_.c_str())
{
  Init();

  // start main loop when instantiated as a standalone node
  const double update_freq = this->declare_parameter("update_freq", 100.);
  timer_ = this->create_wall_timer(1s / update_freq, std::bind(&Tracker::MainLoop, this));

}

Tracker::Tracker(
  const rclcpp::Node & base_node, const std::string & tracker_name,
  const std::shared_ptr<vrpn_Connection> & connection)
: Node(base_node, ValidNodeName(tracker_name)),
  name_(tracker_name),
  multi_sensor_(base_node.get_parameter("multi_sensor").as_bool()),
  frame_id_(base_node.get_parameter("frame_id").as_string()),
  sensor_data_qos_(base_node.get_parameter("sensor_data_qos").as_bool()),
  use_vrpn_timestamps_(base_node.get_parameter("use_vrpn_timestamps").as_bool()),
  vrpn_tracker_(name_.c_str(), connection.get())
{
  Init();
}

Tracker::~Tracker()
{
  vrpn_tracker_.unregister_change_handler(this, &Tracker::HandlePose);
  vrpn_tracker_.unregister_change_handler(this, &Tracker::HandleTwist);
  vrpn_tracker_.unregister_change_handler(this, &Tracker::HandleAccel);

  RCLCPP_INFO_STREAM(this->get_logger(), "Destroyed new tracker " << name_);
}

void Tracker::Init()
{
  vrpn_tracker_.register_change_handler(this, &Tracker::HandlePose);
  vrpn_tracker_.register_change_handler(this, &Tracker::HandleTwist);
  vrpn_tracker_.register_change_handler(this, &Tracker::HandleAccel);
  vrpn_tracker_.shutup = true;

  RCLCPP_INFO_STREAM(this->get_logger(), "Created new tracker " << name_);
}

void Tracker::MainLoop() {vrpn_tracker_.mainloop();}

builtin_interfaces::msg::Time Tracker::GetTimestamp(struct timeval vrpn_timestamp)
{
  if (this->use_vrpn_timestamps_) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = vrpn_timestamp.tv_sec;
    stamp.nanosec = vrpn_timestamp.tv_usec * 1000;

    return stamp;
  }
  return this->get_clock()->now();
}

// Add a shared structure to store velocity in the Tracker class
struct Velocity {
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
};

Velocity latest_velocity_;

// Update HandlePose to include velocity from the shared structure
void VRPN_CALLBACK Tracker::HandlePose(void * data, const vrpn_TRACKERCB tracker_pose)
{
  Tracker * tracker = static_cast<Tracker *>(data);

  // Lazy initialization of PoseStamped publisher
  auto pub = tracker->GetOrCreatePublisher<PoseStamped>(
    static_cast<size_t>(tracker_pose.sensor), "pose", &tracker->pose_pubs_);

  // Populate PoseStamped message
  PoseStamped msg;
  msg.header.frame_id = tracker->frame_id_;
  msg.header.stamp = tracker->GetTimestamp(tracker_pose.msg_time);

  msg.pose.position.x = tracker_pose.pos[0];
  msg.pose.position.y = tracker_pose.pos[1];
  msg.pose.position.z = tracker_pose.pos[2];

  msg.pose.orientation.x = tracker_pose.quat[0];
  msg.pose.orientation.y = tracker_pose.quat[1];
  msg.pose.orientation.z = tracker_pose.quat[2];
  msg.pose.orientation.w = tracker_pose.quat[3];

  pub->publish(msg);

  // Lazy initialization of PoseStamped publisher
  auto pose_rpy_pub = tracker->GetOrCreatePublisher<PoseRPY>(
    static_cast<size_t>(tracker_pose.sensor), "pose_rpy", &tracker->pose_rpy_pubs_);

  // Populate and publish PoseRPY message
  PoseRPY pose_rpy_msg;
  pose_rpy_msg.x = tracker_pose.pos[0];
  pose_rpy_msg.y = tracker_pose.pos[1];
  pose_rpy_msg.z = tracker_pose.pos[2];

  // Add velocity fields from the shared structure
  pose_rpy_msg.x_vel = tracker->latest_velocity_.vx;
  pose_rpy_msg.y_vel = tracker->latest_velocity_.vy;
  pose_rpy_msg.z_vel = tracker->latest_velocity_.vz;

  // Convert quaternion to roll, pitch, yaw
  Eigen::Quaterniond quat(
    tracker_pose.quat[3], tracker_pose.quat[0], tracker_pose.quat[1], tracker_pose.quat[2]);
  Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

  pose_rpy_msg.roll = (euler[0]*3.14)/180;
  pose_rpy_msg.pitch = (euler[1]*3.14)/180;
  pose_rpy_msg.yaw = (euler[2]*3.14)/180;

  pose_rpy_pub->publish(pose_rpy_msg);
}

// Update HandleTwist to store velocity in the shared structure
void VRPN_CALLBACK Tracker::HandleTwist(void * data, const vrpn_TRACKERVELCB tracker_twist)
{
  Tracker * tracker = static_cast<Tracker *>(data);

  // Store the latest velocity
  tracker->latest_velocity_.vx = tracker_twist.vel[0];
  tracker->latest_velocity_.vy = tracker_twist.vel[1];
  tracker->latest_velocity_.vz = tracker_twist.vel[2];

  // Lazy initialization of publisher
  auto pub = tracker->GetOrCreatePublisher<TwistStamped>(
    static_cast<size_t>(tracker_twist.sensor), "twist", &tracker->twist_pubs_);

  // Populate message
  TwistStamped msg;
  msg.header.frame_id = tracker->frame_id_;
  msg.header.stamp = tracker->GetTimestamp(tracker_twist.msg_time);

  msg.twist.linear.x = tracker_twist.vel[0];
  msg.twist.linear.y = tracker_twist.vel[1];
  msg.twist.linear.z = tracker_twist.vel[2];

  const Eigen::Quaterniond quat(
    tracker_twist.vel_quat[3], tracker_twist.vel_quat[0], tracker_twist.vel_quat[1],
    tracker_twist.vel_quat[2]);
  const Eigen::AngleAxisd axis_ang(quat);
  const Eigen::Vector3d rot_vel = axis_ang.axis() * axis_ang.angle() / tracker_twist.vel_quat_dt;
  msg.twist.angular.x = rot_vel.x();
  msg.twist.angular.y = rot_vel.y();
  msg.twist.angular.z = rot_vel.z();

  pub->publish(msg);
}

void VRPN_CALLBACK Tracker::HandleAccel(void * data, const vrpn_TRACKERACCCB tracker_accel)
{
  Tracker * tracker = static_cast<Tracker *>(data);

  // lazy initialization of publisher
  auto pub = tracker->GetOrCreatePublisher<AccelStamped>(
    static_cast<size_t>(tracker_accel.sensor), "accel", &tracker->accel_pubs_);

  // populate message
  AccelStamped msg;
  msg.header.frame_id = tracker->frame_id_;
  msg.header.stamp = tracker->GetTimestamp(tracker_accel.msg_time);

  msg.accel.linear.x = tracker_accel.acc[0];
  msg.accel.linear.y = tracker_accel.acc[1];
  msg.accel.linear.z = tracker_accel.acc[2];

  const Eigen::Quaterniond quat(
    tracker_accel.acc_quat[3], tracker_accel.acc_quat[0], tracker_accel.acc_quat[1],
    tracker_accel.acc_quat[2]);
  const Eigen::AngleAxisd axis_ang(quat);
  const Eigen::Vector3d rot_acc = axis_ang.axis() * axis_ang.angle() / tracker_accel.acc_quat_dt;
  msg.accel.angular.x = rot_acc.x();
  msg.accel.angular.y = rot_acc.y();
  msg.accel.angular.z = rot_acc.z();

  pub->publish(msg);
}

// Add a publisher for PoseRPY in the Tracker class
rclcpp::Publisher<PoseRPY>::SharedPtr pose_rpy_pub_;

}  // namespace vrpn_mocap
