#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <memory>
#include <string>

class PoseTFBroadcaster : public rclcpp::Node
{
public:
  PoseTFBroadcaster()
  : Node("pose_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    Subscribe("/world/default/pose/info");
    Subscribe("/model/x500_gimbal_0/pose_static");
    Subscribe("/model/x500_gimbal_0/pose");
    Subscribe("/model/X1_asp/pose_static");
    Subscribe("/model/X1_asp/pose");
  }

private:
  void Subscribe(const std::string & topic)
  {
    if (!gz_node_.Subscribe(topic, &PoseTFBroadcaster::PoseVectorCallback, this)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to Gazebo topic: %s", topic.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo topic: %s", topic.c_str());
  }

  static std::string NormalizeFrame(std::string frame)
  {
    if (frame == "default") {
      return "map";
    }

    std::string::size_type pos = 0;
    while ((pos = frame.find("::", pos)) != std::string::npos) {
      frame.replace(pos, 2, "/");
      ++pos;
    }

    return frame;
  }

  static std::string HeaderValue(const gz::msgs::Pose & pose, const std::string & key)
  {
    if (!pose.has_header()) {
      return "";
    }

    const auto & header = pose.header();
    for (const auto & data : header.data()) {
      if (data.key() == key && data.value_size() > 0) {
        return data.value(0);
      }
    }

    return "";
  }

  static bool ShouldPublish(const gz::msgs::Pose & pose)
  {
    const std::string name = pose.name();
    if (name == "x500_gimbal_0" || name == "X1_asp") {
      return true;
    }

    return name.rfind("x500_gimbal_0::", 0) == 0 ||
           name.rfind("X1_asp::", 0) == 0;
  }

  template<typename StampT>
  static bool CopyStamp(const StampT & stamp, geometry_msgs::msg::TransformStamped & transform)
  {
    if (stamp.sec() == 0 && stamp.nsec() == 0) {
      return false;
    }

    transform.header.stamp.sec = static_cast<int32_t>(stamp.sec());
    transform.header.stamp.nanosec = static_cast<uint32_t>(stamp.nsec());
    return true;
  }

  void PoseVectorCallback(const gz::msgs::Pose_V & msg)
  {
    for (const auto & pose : msg.pose()) {
      if (!ShouldPublish(pose)) {
        continue;
      }

      geometry_msgs::msg::TransformStamped transform;

      bool has_stamp = false;
      if (pose.has_header() && pose.header().has_stamp()) {
        has_stamp = CopyStamp(pose.header().stamp(), transform);
      }
      if (!has_stamp && msg.has_header() && msg.header().has_stamp()) {
        has_stamp = CopyStamp(msg.header().stamp(), transform);
      }
      if (!has_stamp) {
        transform.header.stamp = this->get_clock()->now();
      }

      std::string parent = HeaderValue(pose, "frame_id");
      std::string child = HeaderValue(pose, "child_frame_id");

      if (parent.empty()) {
        parent = "map";
      }
      if (child.empty()) {
        child = pose.name();
      }

      transform.header.frame_id = NormalizeFrame(parent);
      transform.child_frame_id = NormalizeFrame(child);

      transform.transform.translation.x = pose.position().x();
      transform.transform.translation.y = pose.position().y();
      transform.transform.translation.z = pose.position().z();
      transform.transform.rotation.x = pose.orientation().x();
      transform.transform.rotation.y = pose.orientation().y();
      transform.transform.rotation.z = pose.orientation().z();
      transform.transform.rotation.w = pose.orientation().w();

      tf_broadcaster_->sendTransform(transform);
    }
  }

  gz::transport::Node gz_node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
