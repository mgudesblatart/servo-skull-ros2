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
  EXPECT_EQ(it->second.id_confidence, 0.8f);
  EXPECT_EQ(it->second.is_facing, true);
  EXPECT_EQ(it->second.age, 1);
  EXPECT_EQ(it->second.visible_count, 1);
  EXPECT_EQ(it->second.state, TrackState::ACTIVE);
  EXPECT_GT(it->second.internal_id, 0);
}

TEST_F(PersonTrackingNodeTest, TrackIsNotCreatedForLowConfidence) {
  servo_skull_msgs::msg::PersonDetection msg;
  msg.id = 99;
  msg.box_confidence = 0.4f; // Below threshold of 0.5
  node_->detection_callback(std::make_shared<servo_skull_msgs::msg::PersonDetection>(msg));
  // Should not create a track for low confidence
  auto it = node_->tracks_.find(99);
  EXPECT_EQ(it, node_->tracks_.end());
}

TEST_F(PersonTrackingNodeTest, TrackIsUpdatedOnRepeatedDetection) {
  servo_skull_msgs::msg::PersonDetection msg;
  msg.id = 7;
  msg.box_confidence = 0.95f;
  msg.box_left = 0.2f;
  msg.box_top = 0.3f;
  msg.box_right = 0.4f;
  msg.box_bottom = 0.5f;
  msg.id_confidence = 0.7f;
  msg.is_facing = false;
  node_->detection_callback(std::make_shared<servo_skull_msgs::msg::PersonDetection>(msg));
  // Update with new values
  msg.box_confidence = 0.99f;
  msg.box_left = 0.25f;
  msg.is_facing = true;
  node_->detection_callback(std::make_shared<servo_skull_msgs::msg::PersonDetection>(msg));
  auto it = node_->tracks_.find(7);
  ASSERT_NE(it, node_->tracks_.end());
  EXPECT_EQ(it->second.box_confidence, 0.99f);
  EXPECT_EQ(it->second.box_left, 0.25f);
  EXPECT_EQ(it->second.is_facing, true);
  EXPECT_EQ(it->second.age, 2);
  EXPECT_EQ(it->second.visible_count, 2);
  EXPECT_EQ(it->second.state, TrackState::ACTIVE);
}

// Add more tests for cleanup and publishing as needed
