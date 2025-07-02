#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "person_tracking_node.hpp"
#include "servo_skull_msgs/msg/person_detection.hpp"

class PersonTrackingNodeTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<PersonTrackingNode>();
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
  std::shared_ptr<PersonTrackingNode> node_;
};

TEST_F(PersonTrackingNodeTest, TrackIsCreatedOnDetection) {
  servo_skull_msgs::msg::PersonDetection msg;
  msg.id = 42;
  msg.box_confidence = 0.9f;
  msg.box_left = 0.1f;
  msg.box_top = 0.2f;
  msg.box_right = 0.3f;
  msg.box_bottom = 0.4f;
  msg.id_confidence = 0.8f;
  msg.is_facing = true;
  node_->detection_callback(std::make_shared<servo_skull_msgs::msg::PersonDetection>(msg));
  // Check that the track was created
  auto it = node_->tracks_.find(42);
  ASSERT_NE(it, node_->tracks_.end());
  EXPECT_EQ(it->second.sensor_id, 42);
  EXPECT_EQ(it->second.box_confidence, 0.9f);
  EXPECT_EQ(it->second.state, TrackState::ACTIVE);
}

TEST_F(PersonTrackingNodeTest, TrackIsNotCreatedForLowConfidence) {
  servo_skull_msgs::msg::PersonDetection msg;
  msg.id = 99;
  msg.box_confidence = 0.1f;
  node_->detection_callback(std::make_shared<servo_skull_msgs::msg::PersonDetection>(msg));
  // Should not create a track for low confidence
  auto it = node_->tracks_.find(99);
  EXPECT_EQ(it, node_->tracks_.end());
}

// Add more tests for update, cleanup, and publishing as needed
