/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sebastian Jahr
   Description: Integration tests for the move_group interface
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "parameter_name_list.hpp"

class MoveGroupFixture : public testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    test_node_ = std::make_shared<rclcpp::Node>("move_group_param_test_node", node_options);
  }

  void TearDown() override
  {
  }

  std::shared_ptr<rclcpp::Node> test_node_;
};

TEST_F(MoveGroupFixture, testParamAPI)
{
  // Create parameter client
  auto params_client = std::make_shared<rclcpp::SyncParametersClient>(test_node_, "move_group");
  bool reach_param_client = params_client->wait_for_service(std::chrono::seconds(5));
  // This pattern helps with debugging if service is not available.
  if (!reach_param_client)
  {
    RCLCPP_ERROR(test_node_->get_logger(), "Couldn't reach parameter server. Is the move_group up and running?");
  }
  ASSERT_TRUE(reach_param_client);

  // GIVEN a running move_group
  // WHEN a parameter name from parameter API checked
  // THEN that parameter exists
  for (const auto& param_name : move_group_test::PARAMETER_NAME_LIST)
  {
    bool param_exists = false;
    if (params_client->wait_for_service(std::chrono::milliseconds(1)))
    {
      param_exists = params_client->has_parameter(param_name);
      if (!param_exists)
      {
        RCLCPP_ERROR(test_node_->get_logger(), "Parameter %s doesn't exists", param_name.c_str());
      }
    }
    else
    {
      RCLCPP_ERROR(test_node_->get_logger(), "Cannot read parameter %s because service couldn't be reached",
                   param_name.c_str());
    }
    EXPECT_TRUE(param_exists);
  }
}

TEST_F(MoveGroupFixture, testPlanningTimeInMoveAction)
{
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  auto client = rclcpp_action::create_client<MoveGroup>(test_node_, "/move_action");
  auto controller = rclcpp_action::create_client<FollowJointTrajectory>(
      test_node_, "/panda_arm_controller/follow_joint_trajectory");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(10)));
  ASSERT_TRUE(controller->wait_for_action_server(std::chrono::seconds(10)));

  MoveGroup::Goal goal;
  goal.request.group_name = "panda_arm";
  goal.request.pipeline_id = "ompl";
  goal.request.allowed_planning_time = 5.0;
  goal.request.num_planning_attempts = 1;
  goal.request.max_velocity_scaling_factor = 0.5;
  goal.request.max_acceleration_scaling_factor = 0.5;
  goal.request.start_state.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  moveit_msgs::msg::JointConstraint joint;
  joint.joint_name = "panda_joint7";
  joint.position = 0.905;
  joint.tolerance_above = 0.001;
  joint.tolerance_below = 0.001;
  joint.weight = 1.0;
  moveit_msgs::msg::Constraints constraints;
  constraints.joint_constraints.push_back(joint);
  goal.request.goal_constraints.push_back(constraints);

  // Planning alone leaves the robot at its starting pose.
  for (const bool plan_only : { true, false })
  {
    SCOPED_TRACE(plan_only);
    goal.planning_options.plan_only = plan_only;
    auto goal_future = client->async_send_goal(goal);
    ASSERT_EQ(rclcpp::spin_until_future_complete(test_node_, goal_future, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    auto goal_handle = goal_future.get();
    ASSERT_TRUE(goal_handle);
    auto result_future = client->async_get_result(goal_handle);
    ASSERT_EQ(rclcpp::spin_until_future_complete(test_node_, result_future, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto result = result_future.get();
    ASSERT_TRUE(result.result);
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_EQ(result.result->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    ASSERT_FALSE(result.result->planned_trajectory.joint_trajectory.points.empty());
    EXPECT_GT(result.result->planning_time, 0.0);
  }
}

// TODO(sjahr): Add more API tests e.g. from move_group_interface_cpp_test.cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
