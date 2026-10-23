#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "DataStreamClient.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using ViconDataStreamSDK::CPP::Client;
namespace Direction = ViconDataStreamSDK::CPP::Direction;
namespace Result = ViconDataStreamSDK::CPP::Result;

class ViconBridge : public rclcpp::Node {
 public:
  ViconBridge() : Node("vicon_datastream_bridge") {
    host_ = declare_parameter<std::string>("host", "10.86.153.14:801");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    publishers_["cf1"] = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/vicon/cf1/pose", 10);
    publishers_["cf2"] = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/wand/pose", 10);
  }

  bool connect() {
    RCLCPP_INFO(get_logger(), "Connecting to Vicon DataStream at %s", host_.c_str());
    if (client_.Connect(host_).Result != Result::Success) {
      RCLCPP_ERROR(get_logger(), "Could not connect to Vicon DataStream");
      return false;
    }
    client_.EnableSegmentData();
    client_.SetAxisMapping(Direction::Forward, Direction::Left, Direction::Up);
    RCLCPP_INFO(get_logger(),
                "Connected: cf1 -> /vicon/cf1/pose, cf2 -> /wand/pose");
    return true;
  }

  void run() {
    while (rclcpp::ok()) {
      if (client_.GetFrame().Result != Result::Success) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      const auto count = client_.GetSubjectCount();
      if (count.Result != Result::Success) continue;
      for (unsigned int index = 0; index < count.SubjectCount; ++index) {
        const auto subject = client_.GetSubjectName(index);
        const std::string name = subject.SubjectName;
        const auto publisher = publishers_.find(name);
        if (subject.Result != Result::Success || publisher == publishers_.end()) continue;
        const auto root = client_.GetSubjectRootSegmentName(name);
        const auto position = client_.GetSegmentGlobalTranslation(name, root.SegmentName);
        const auto rotation = client_.GetSegmentGlobalRotationQuaternion(name, root.SegmentName);
        if (root.Result != Result::Success || position.Result != Result::Success ||
            rotation.Result != Result::Success || position.Occluded || rotation.Occluded) continue;

        geometry_msgs::msg::PoseStamped message;
        message.header.stamp = now();
        message.header.frame_id = frame_id_;
        message.pose.position.x = position.Translation[0] / 1000.0;
        message.pose.position.y = position.Translation[1] / 1000.0;
        message.pose.position.z = position.Translation[2] / 1000.0;
        message.pose.orientation.x = rotation.Rotation[0];
        message.pose.orientation.y = rotation.Rotation[1];
        message.pose.orientation.z = rotation.Rotation[2];
        message.pose.orientation.w = rotation.Rotation[3];
        publisher->second->publish(message);
        if (published_subjects_.insert(name).second) {
          RCLCPP_INFO(get_logger(),
                      "Publishing %s pose: (%.3f, %.3f, %.3f) m",
                      name.c_str(), message.pose.position.x,
                      message.pose.position.y, message.pose.position.z);
        }
      }
      if (rclcpp::ok()) rclcpp::spin_some(shared_from_this());
    }
  }

 private:
  Client client_;
  std::string host_;
  std::string frame_id_;
  std::unordered_map<std::string,
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_;
  std::unordered_set<std::string> published_subjects_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ViconBridge>();
  if (node->connect()) node->run();
  rclcpp::shutdown();
  return 0;
}
