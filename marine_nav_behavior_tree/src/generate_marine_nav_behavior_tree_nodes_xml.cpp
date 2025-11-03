// Copyright (c) 2024 Davide Faconti
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
// limitations under the License. Reserved.

// Adapted from nav2_behavior_tree/src/generate_nav2_tree_nodes_xml.cpp

#include <vector>
#include <string>
#include <fstream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/xml_parsing.h"

#include "plugins_list.hpp"
#include "nav2_util/string_utils.hpp"

//BT_REGISTER_NODES(factory);
#include "bt_register_nodes.cpp"

int main()
{
  BT::BehaviorTreeFactory factory;

  std::vector<std::string> plugins_list = nav2_util::split(marine_nav_behavior_tree::details::BT_BUILTIN_PLUGINS, ';');

  // for (const auto & plugin : plugins_list) {
  //   std::cout << "Loading: " << plugin << "\n";
  //   factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  // }
  
  BT_RegisterNodesFromPlugin(factory);

  std::cout << "\nGenerating file: marine_nav_behavior_tree_nodes.xml\n"
            << "\nCompare it with the one in the git repo and update the latter if necessary.\n";

  std::ofstream xml_file;
  xml_file.open("marine_nav_behavior_tree_nodes.xml");
  xml_file << BT::writeTreeNodesModelXML(factory) << std::endl;
  xml_file.close();

  return 0;
}
