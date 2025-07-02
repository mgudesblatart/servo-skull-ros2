// person_tracking_node.hpp
// Header for PersonTrackingNode class

#ifndef PERSON_TRACKING_NODE_HPP
#define PERSON_TRACKING_NODE_HPP

#include <memory>
#include <unordered_map>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "servo_skull_msgs/msg/person_detection.hpp"
#include "servo_skull_msgs/msg/tracked_person.hpp"
#include "servo_skull_msgs/msg/tracked_persons.hpp"

enum class TrackState { ACTIVE, LOST, DELETED };

struct Track {
  int internal_id; // unique internal tracking ID
  int sensor_id;   // id from the sensor
  float box_left;
  float box_top;
  float box_right;
  float box_bottom;
  float box_confidence;
  float id_confidence;
  bool is_facing;
  std::chrono::steady_clock::time_point last_update;
  int age = 0; // number of frames tracked
  int visible_count = 0; // consecutive frames seen
  TrackState state = TrackState::ACTIVE;
  // Optionally: std::string name, role, etc.
};

class PersonTrackingNode : public rclcpp::Node {
public:
  PersonTrackingNode();

private:
  void detection_callback(const servo_skull_msgs::msg::PersonDetection::SharedPtr msg);
  void cleanup_tracks();
  void publish_tracks();
  int next_internal_id_ = 1;
  rclcpp::Subscription<servo_skull_msgs::msg::PersonDetection>::SharedPtr subscription_;
  rclcpp::Publisher<servo_skull_msgs::msg::TrackedPersons>::SharedPtr tracked_persons_pub_;
  std::unordered_map<int, Track> tracks_;
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  const std::chrono::seconds track_timeout_{5}; // configurable timeout for occlusion handling
};

#endif // PERSON_TRACKING_NODE_HPP
