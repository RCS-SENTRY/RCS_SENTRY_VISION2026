#pragma once

#if __has_include(<behaviortree_cpp/bt_factory.h>)
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#elif __has_include(<behaviortree_cpp_v3/bt_factory.h>)
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/xml_parsing.h>
#else
#error "BehaviorTree.CPP headers not found. Install behaviortree_cpp or behaviortree_cpp_v3."
#endif
