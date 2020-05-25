// Copyright (c) 2020 Aitor Miguel Blanco
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

#ifndef NAV2_BEHAVIOR_TREE__CONDITION_CHECK_NODE_HPP_
#define NAV2_BEHAVIOR_TREE__CONDITION_CHECK_NODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{

class ConditionCheckNode : public BT::ControlNode
{
public:
  explicit ConditionCheckNode(const std::string & name)
  : BT::ControlNode::ControlNode(name, {})
  {
    setRegistrationID("RoundRobin");
  }

  BT::NodeStatus tick() override
  {
    const size_t children_count = children_nodes_.size();

    setStatus(BT::NodeStatus::RUNNING);

    while (current_child_idx_ < children_count)
    {
      TreeNode* current_child_node = children_nodes_[current_child_idx_];
      const BT::NodeStatus child_status = current_child_node->executeTick();

      switch (child_status)
      {
        case BT::NodeStatus::RUNNING:
        {
          return child_status;
        }
        case BT::NodeStatus::SUCCESS:
        {
          // If any child returns success, halt all and return success
          haltChildren();
          current_child_idx_ = 0;
          return child_status;
        }
        case BT::NodeStatus::FAILURE:
        {
          current_child_idx_++;
        }
        break;

        case BT::NodeStatus::IDLE:
        {
          throw BT::LogicError("A child node must never return IDLE");
        }
      }
    }

    // The entire loop was completed and should be restarted.
    if (current_child_idx_ == children_count)
    {
        current_child_idx_ = 0;
    }

    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    ControlNode::halt();
    current_child_idx_ = 0;
  }

private:
  unsigned int current_child_idx_{0};
};

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ConditionCheckNode>("ConditionCheck");
}

#endif  // NAV2_BEHAVIOR_TREE__CONDITION_CHECK_NODE_HPP_
