// Copyright 2024 Austrian Institute of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>
#include <stdexcept>
#include <limits>

#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "joint_trajectory_controller/tolerances.hpp"
#include "test_trajectory_controller_utils.hpp"

using joint_trajectory_controller::SegmentTolerances;
using trajectory_msgs::msg::JointTrajectoryPoint;

std::vector<std::string> joint_names_ = {"joint1", "joint2", "joint3"};

control_msgs::action::FollowJointTrajectory_Goal prepareGoalMsg(
  const std::vector<JointTrajectoryPoint> & points, double goal_time_tolerance,
  const std::vector<control_msgs::msg::JointTolerance> path_tolerance =
    std::vector<control_msgs::msg::JointTolerance>(),
  const std::vector<control_msgs::msg::JointTolerance> goal_tolerance =
    std::vector<control_msgs::msg::JointTolerance>())
{
  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(goal_time_tolerance);
  goal_msg.goal_tolerance = goal_tolerance;
  goal_msg.path_tolerance = path_tolerance;
  goal_msg.trajectory.joint_names = joint_names_;
  goal_msg.trajectory.points = points;

  return goal_msg;
}

class TestTolerancesFixture : public ::testing::Test
{
protected:
  SegmentTolerances default_tolerances;
  joint_trajectory_controller::Params params;
  std::vector<std::string> joint_names_;
  rclcpp::Logger logger = rclcpp::get_logger("TestTolerancesFixture");

  void SetUp() override
  {
    // Initialize joint_names_ with some test data
    joint_names_ = {"joint1", "joint2", "joint3"};

    // Initialize default_tolerances and params with common setup for all tests
    default_tolerances.goal_time_tolerance = default_goal_time;
    default_tolerances.state_tolerance.resize(joint_names_.size());
    default_tolerances.goal_state_tolerance.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      default_tolerances.state_tolerance.at(i).position = 0.1;
      default_tolerances.state_tolerance.at(i).velocity = 0.0;
      default_tolerances.state_tolerance.at(i).acceleration = 0.0;

      default_tolerances.goal_state_tolerance.at(i).position = 0.1;
      default_tolerances.goal_state_tolerance.at(i).velocity = stopped_velocity_tolerance;
      default_tolerances.goal_state_tolerance.at(i).acceleration = 0.0;
    }
    params.joints = joint_names_;
  }

  void TearDown() override
  {
    // Cleanup code if necessary
  }
};

TEST_F(TestTolerancesFixture, test_resolve_tolerance_source)
{
  const double DEFAULT = 0.5;

  // Case 1: Positive value (Explicit Override)
  EXPECT_DOUBLE_EQ(joint_trajectory_controller::resolve_tolerance_source(DEFAULT, 1.2), 1.2);

  // Case 2: Zero value (Use Default)
  EXPECT_DOUBLE_EQ(joint_trajectory_controller::resolve_tolerance_source(DEFAULT, 0.0), DEFAULT);

  // Case 3: ERASE_VALUE (-1.0) -> Should return 0.0 (disable check)
  EXPECT_DOUBLE_EQ(joint_trajectory_controller::resolve_tolerance_source(DEFAULT, -1.0), 0.0);

  // Case 4: Illegal Negative Value -> Should throw a runtime error
  EXPECT_THROW(
    joint_trajectory_controller::resolve_tolerance_source(DEFAULT, -0.5), std::runtime_error);
}

TEST_F(TestTolerancesFixture, test_get_segment_tolerances)
{
  // send goal with nonzero tolerances, are they accepted?
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> path_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  // add the same tolerance for every joint, give it in correct order
  tolerance.name = "joint1";
  tolerance.position = 0.2;
  tolerance.velocity = 0.3;
  tolerance.acceleration = 0.4;
  path_tolerance.push_back(tolerance);
  tolerance.name = "joint2";
  tolerance.position = 0.2;
  tolerance.velocity = 0.3;
  tolerance.acceleration = 0.4;
  path_tolerance.push_back(tolerance);
  tolerance.name = "joint3";
  tolerance.position = 0.2;
  tolerance.velocity = 0.3;
  tolerance.acceleration = 0.4;
  path_tolerance.push_back(tolerance);

  std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
  // add different tolerances in jumbled order
  tolerance.name = "joint2";
  tolerance.position = 1.2;
  tolerance.velocity = 2.2;
  tolerance.acceleration = 3.2;
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint3";
  tolerance.position = 1.3;
  tolerance.velocity = 2.3;
  tolerance.acceleration = 3.3;
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint1";
  tolerance.position = 1.1;
  tolerance.velocity = 2.1;
  tolerance.acceleration = 3.1;
  goal_tolerance.push_back(tolerance);

  auto goal_msg = prepareGoalMsg(points, 2.0, path_tolerance, goal_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);

  EXPECT_DOUBLE_EQ(active_tolerances.goal_time_tolerance, 2.0);

  // Path tolerances check (state_tolerance)
  ASSERT_EQ(active_tolerances.state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).position, 0.2);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).velocity, 0.3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).acceleration, 0.4);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).position, 0.2);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).velocity, 0.3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).acceleration, 0.4);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).position, 0.2);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).velocity, 0.3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).acceleration, 0.4);

  // Goal tolerances check (goal_state_tolerance)
  ASSERT_EQ(active_tolerances.goal_state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).position, 1.1);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).velocity, 2.1);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).acceleration, 3.1);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).position, 1.2);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).velocity, 2.2);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).acceleration, 3.2);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).position, 1.3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).velocity, 2.3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).acceleration, 3.3);
}

// send goal with deactivated tolerances (-1)
TEST_F(TestTolerancesFixture, test_deactivate_tolerances)
{
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> path_tolerance;
  std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  // Use -1.0 to deactivate/erase the default tolerance for all interfaces
  tolerance.name = "joint1";
  tolerance.position = -1.0;
  tolerance.velocity = -1.0;
  tolerance.acceleration = -1.0;
  path_tolerance.push_back(tolerance);
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint2";
  path_tolerance.push_back(tolerance);
  goal_tolerance.push_back(tolerance);
  tolerance.name = "joint3";
  path_tolerance.push_back(tolerance);
  goal_tolerance.push_back(tolerance);

  auto goal_msg = prepareGoalMsg(points, -1.0, path_tolerance, goal_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);

  // goal_time_tolerance of -1.0 should result in 0.0 (max(0.0, -1.0))
  EXPECT_DOUBLE_EQ(active_tolerances.goal_time_tolerance, 0.0);

  // Check state tolerances (all should be 0.0)
  ASSERT_EQ(active_tolerances.state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(0).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(1).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.state_tolerance.at(2).acceleration, 0.0);

  // Check goal state tolerances (all should be 0.0)
  ASSERT_EQ(active_tolerances.goal_state_tolerance.size(), 3);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(0).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(1).acceleration, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).position, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).velocity, 0.0);
  EXPECT_DOUBLE_EQ(active_tolerances.goal_state_tolerance.at(2).acceleration, 0.0);
}

// send goal with invalid tolerances, are the default ones used?
TEST_F(TestTolerancesFixture, test_invalid_tolerances)
{
  {
    SCOPED_TRACE("negative goal_time_tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, -123.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    // Goal time is handled with std::max(0.0, value), so it should return 0.0,
    // but the tolerance check itself should return the defaults if any joint tolerance is illegal.
    // Since only goal_time is illegal here, we check for default tolerances (except for goal_time).
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative path position tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = -123.0;  // ILLEGAL VALUE
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative path velocity tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = -123.0;  // ILLEGAL VALUE
    tolerance.acceleration = 0.0;
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative path acceleration tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> path_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = -123.0;  // ILLEGAL VALUE
    path_tolerance.push_back(tolerance);

    auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative goal position tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = -123.0;  // ILLEGAL VALUE
    tolerance.velocity = 0.0;
    tolerance.acceleration = 0.0;
    goal_tolerance.push_back(tolerance);

    auto goal_msg =
      prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative goal velocity tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = -123.0;  // ILLEGAL VALUE
    tolerance.acceleration = 0.0;
    goal_tolerance.push_back(tolerance);

    auto goal_msg =
      prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
  {
    SCOPED_TRACE("negative goal acceleration tolerance");
    std::vector<JointTrajectoryPoint> points;
    JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(0.5);
    point.positions.resize(joint_names_.size());

    point.positions[0] = 1.0;
    point.positions[1] = 2.0;
    point.positions[2] = 3.0;
    points.push_back(point);

    std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
    control_msgs::msg::JointTolerance tolerance;
    tolerance.name = "joint1";
    tolerance.position = 0.0;
    tolerance.velocity = 0.0;
    tolerance.acceleration = -123.0;  // ILLEGAL VALUE
    goal_tolerance.push_back(tolerance);

    auto goal_msg =
      prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
    auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances, goal_msg, params.joints);
    expectDefaultTolerances(active_tolerances);
  }
}
TEST_F(TestTolerancesFixture, test_invalid_joints_path_tolerance)
{
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> path_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  tolerance.name = "joint123";  // UNKNOWN JOINT
  path_tolerance.push_back(tolerance);

  auto goal_msg = prepareGoalMsg(points, 3.0, path_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);
  expectDefaultTolerances(active_tolerances);
}
TEST_F(TestTolerancesFixture, test_invalid_joints_goal_tolerance)
{
  std::vector<JointTrajectoryPoint> points;
  JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  point.positions.resize(joint_names_.size());

  point.positions[0] = 1.0;
  point.positions[1] = 2.0;
  point.positions[2] = 3.0;
  points.push_back(point);

  std::vector<control_msgs::msg::JointTolerance> goal_tolerance;
  control_msgs::msg::JointTolerance tolerance;
  tolerance.name = "joint123";  // UNKNOWN JOINT
  goal_tolerance.push_back(tolerance);

  auto goal_msg =
    prepareGoalMsg(points, 3.0, std::vector<control_msgs::msg::JointTolerance>(), goal_tolerance);
  auto active_tolerances = joint_trajectory_controller::get_segment_tolerances(
    logger, default_tolerances, goal_msg, params.joints);
  expectDefaultTolerances(active_tolerances);
}

TEST_F(TestTolerancesFixture, test_create_error_trajectory_point_pv_only)
{
  JointTrajectoryPoint desired;
  desired.positions = {1.0, 2.0};
  desired.velocities = {0.1, 0.2};

  JointTrajectoryPoint actual;
  actual.positions = {1.05, 2.1};
  actual.velocities = {0.15, 0.3};

  // Execute: Error = Desired - Actual
  auto error = joint_trajectory_controller::create_error_trajectory_point(desired, actual);

  // Assertions for P and V interfaces
  ASSERT_EQ(error.positions.size(), 2);
  ASSERT_EQ(error.velocities.size(), 2);
  ASSERT_EQ(error.accelerations.size(), 0); // Not available

  // Joint 0 checks
  EXPECT_DOUBLE_EQ(error.positions[0], -0.05);  // 1.0 - 1.05
  EXPECT_DOUBLE_EQ(error.velocities[0], -0.05); // 0.1 - 0.15

  // Joint 1 checks
  EXPECT_DOUBLE_EQ(error.positions[1], -0.1);   // 2.0 - 2.1
  EXPECT_DOUBLE_EQ(error.velocities[1], -0.1);  // 0.2 - 0.3
}

TEST_F(TestTolerancesFixture, test_create_error_trajectory_point_pva)
{
  JointTrajectoryPoint desired;
  desired.positions = {1.0, 2.0};
  desired.velocities = {0.1, 0.2};
  desired.accelerations = {0.01, 0.02};

  JointTrajectoryPoint actual;
  actual.positions = {1.05, 2.1};
  actual.velocities = {0.15, 0.3};
  actual.accelerations = {0.03, 0.01};

  // Execute: Error = Desired - Actual
  auto error = joint_trajectory_controller::create_error_trajectory_point(desired, actual);

  // Assertions for P, V, and A interfaces
  ASSERT_EQ(error.positions.size(), 2);
  ASSERT_EQ(error.velocities.size(), 2);
  ASSERT_EQ(error.accelerations.size(), 2);

  // Joint 0 checks
  EXPECT_DOUBLE_EQ(error.positions[0], -0.05);    // 1.0 - 1.05
  EXPECT_DOUBLE_EQ(error.velocities[0], -0.05);   // 0.1 - 0.15
  EXPECT_DOUBLE_EQ(error.accelerations[0], -0.02);  // 0.01 - 0.03

  // Joint 1 checks
  EXPECT_DOUBLE_EQ(error.positions[1], -0.1);     // 2.0 - 2.1
  EXPECT_DOUBLE_EQ(error.velocities[1], -0.1);    // 0.2 - 0.3
  EXPECT_DOUBLE_EQ(error.accelerations[1], 0.01);  // 0.02 - 0.01
}


TEST_F(TestTolerancesFixture, test_check_state_tolerance_per_joint)
{
  // Error Point
  JointTrajectoryPoint error;
  error.positions = {0.02, 0.005};
  error.velocities = {0.15, 0.0};
  error.accelerations = {0.05, 0.0};

  // Tolerance 0 (Joint 0: Fails on P)
  joint_trajectory_controller::StateTolerances tol0;
  tol0.position = 0.01;      // FAIL (0.02 > 0.01)
  tol0.velocity = 0.20;      // PASS (0.15 < 0.20)
  tol0.acceleration = 0.10;  // PASS (0.05 < 0.10)

  // Tolerance 1 (Joint 1: All checks disabled or pass)
  joint_trajectory_controller::StateTolerances tol1;
  tol1.position = 0.01;  // PASS (0.005 < 0.01)

  // Test 1: Joint 0 - FAIL (due to Position)
  EXPECT_FALSE(
    joint_trajectory_controller::check_state_tolerance_per_joint(error, 0, tol0, false))
    << "Joint 0 should fail due to position exceeding tolerance.";

  // Test 2: Joint 1 - PASS
  EXPECT_TRUE(
    joint_trajectory_controller::check_state_tolerance_per_joint(error, 1, tol1, false))
    << "Joint 1 should pass.";

  // Test 3: Zero Tolerance (Disabled Check) - PASS
  joint_trajectory_controller::StateTolerances tol_zero;  // All 0.0 (disabled)
  EXPECT_TRUE(
    joint_trajectory_controller::check_state_tolerance_per_joint(error, 0, tol_zero, false))
    << "Zero tolerance should always pass.";
}

TEST_F(TestTolerancesFixture, test_check_trajectory_point_tolerance_aggregate)
{
  JointTrajectoryPoint error;
  error.positions = {0.01, 0.03, 0.005};  // 3 joints
  error.velocities = {0.01, 0.01, 0.01};
  error.accelerations = {0.01, 0.01, 0.01};

  std::vector<joint_trajectory_controller::StateTolerances> tolerances(3);
  // Joint 0: Pass (error == tolerance)
  tolerances[0].position = 0.01;
  tolerances[0].velocity = 0.01;
  // Joint 1: Fail (error 0.03 > 0.02)
  tolerances[1].position = 0.02;
  tolerances[1].velocity = 0.01;
  // Joint 2: Pass (error 0.005 < 0.01)
  tolerances[2].position = 0.01;
  tolerances[2].velocity = 0.01;

  // Test 1: Fail due to Joint 1
  EXPECT_FALSE(
    joint_trajectory_controller::check_trajectory_point_tolerance(error, tolerances, false));

  // Test 2: Fix Joint 1 position tolerance, should pass
  tolerances[1].position = 0.04;
  EXPECT_TRUE(
    joint_trajectory_controller::check_trajectory_point_tolerance(error, tolerances, false));

  // Test 3: Size mismatch failure
  std::vector<joint_trajectory_controller::StateTolerances> short_tolerances(2);
  EXPECT_FALSE(
    joint_trajectory_controller::check_trajectory_point_tolerance(error, short_tolerances, false))
    << "Size mismatch must fail.";
}

}  // namespace joint_trajectory_controller
