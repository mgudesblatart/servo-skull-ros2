// person_tracking_node.cpp
// ROS2 Jazzy C++ node: subscribes to /person_sensor/detections and preprocesses PersonDetection messages

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "servo_skull_msgs/msg/person_detection.hpp"
#include "servo_skull_msgs/msg/tracked_person.hpp"
#include "servo_skull_msgs/msg/tracked_persons.hpp"
#include "person_tracking_node.hpp"
#include <algorithm>

using std::placeholders::_1;

PersonTrackingNode::PersonTrackingNode() : Node("person_tracking_node") {
  subscription_ = this->create_subscription<servo_skull_msgs::msg::PersonDetection>(
    "/person_sensor/detections", 10,
    std::bind(&PersonTrackingNode::detection_callback, this, _1)
  );
  tracked_persons_pub_ = this->create_publisher<servo_skull_msgs::msg::TrackedPersons>(
    "/person_tracking/tracked_persons", 10);
  cleanup_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PersonTrackingNode::cleanup_tracks, this)
  );
  RCLCPP_INFO(this->get_logger(), "Person Tracking Node started, listening for detections...");
}

void PersonTrackingNode::publish_tracks() {
  servo_skull_msgs::msg::TrackedPersons msg;
  for (const auto& pair : tracks_) {
    const Track& track = pair.second;
    servo_skull_msgs::msg::TrackedPerson tp;
    tp.person_id = track.internal_id;
    tp.name = "unknown"; // TODO: Add name association in future
    // tp.position -- TODO: Add 3D position estimation in future
    // tp.velocity -- TODO: Add 3D velocity estimation in future
    tp.box_left = track.box_left;
    tp.box_top = track.box_top;
    tp.box_right = track.box_right;
    tp.box_bottom = track.box_bottom;
    tp.box_confidence = track.box_confidence;
    tp.sensor_id = track.sensor_id;
    tp.age = track.age;
    tp.visible_count = track.visible_count;
    switch (track.state) {
      case TrackState::ACTIVE: tp.state = "active"; break;
      case TrackState::LOST: tp.state = "lost"; break;
      case TrackState::DELETED: tp.state = "deleted"; break;
    }
    msg.tracks.push_back(tp);
  }
  tracked_persons_pub_->publish(msg);
}

void PersonTrackingNode::detection_callback(const servo_skull_msgs::msg::PersonDetection::SharedPtr msg) {
  if (msg->box_confidence < 0.5f) {
    RCLCPP_INFO(this->get_logger(), "Detection below confidence threshold: %.2f", msg->box_confidence);
    return;
  }
  auto now = std::chrono::steady_clock::now();
  auto it = tracks_.find(msg->id);
  if (it == tracks_.end()) {
    // New track
    Track track;
    track.internal_id = next_internal_id_++;
    track.sensor_id = msg->id;
    track.box_left = msg->box_left;
    track.box_top = msg->box_top;
    track.box_right = msg->box_right;
    track.box_bottom = msg->box_bottom;
    track.box_confidence = msg->box_confidence;
    track.id_confidence = msg->id_confidence;
    track.is_facing = msg->is_facing;
    track.last_update = now;
    track.age = 1;
    track.visible_count = 1;
    track.state = TrackState::ACTIVE;
    tracks_[msg->id] = track;
    RCLCPP_INFO(this->get_logger(), "New track created: internal_id=%d, sensor_id=%d", track.internal_id, track.sensor_id);
  } else {
    // Update existing track
    Track& track = it->second;
    track.box_left = msg->box_left;
    track.box_top = msg->box_top;
    track.box_right = msg->box_right;
    track.box_bottom = msg->box_bottom;
    track.box_confidence = msg->box_confidence;
    track.id_confidence = msg->id_confidence;
    track.is_facing = msg->is_facing;
    track.last_update = now;
    track.age++;
    track.visible_count++;
    track.state = TrackState::ACTIVE;
    RCLCPP_INFO(this->get_logger(), "Track updated: internal_id=%d, sensor_id=%d, age=%d, visible=%d", track.internal_id, track.sensor_id, track.age, track.visible_count);
  }
  publish_tracks();
}

void PersonTrackingNode::cleanup_tracks() {
  auto now = std::chrono::steady_clock::now();
  for (auto it = tracks_.begin(); it != tracks_.end(); ) {
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - it->second.last_update);
    if (elapsed > track_timeout_) {
      if (it->second.state != TrackState::DELETED) {
        it->second.state = TrackState::LOST;
        RCLCPP_INFO(this->get_logger(), "Track lost: internal_id=%d, sensor_id=%d", it->second.internal_id, it->second.sensor_id);
      }
      // Optionally, keep lost tracks for a while before deleting
      // For now, delete immediately
      it = tracks_.erase(it);
    } else {
      ++it;
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PersonTrackingNode>());
  rclcpp::shutdown();
  return 0;
}
