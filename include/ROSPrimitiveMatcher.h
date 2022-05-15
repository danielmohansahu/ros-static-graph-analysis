/* Class containing all ROS Primitive specific matching heuristics.
 *
 * This structure aims to decouple the "CLANG" code from the "ROS"
 * code. To that end, this class provides all the necessary information
 * to check if a given AST element is a ROS call that we care about.
 * For convenience we also accumulate the metadata here.
 *
 * Author: Daniel M. Sahu
 */

#pragma once

// STL
#include <fstream>
#include <string>
#include <unordered_map>
#include <filesystem>
#include <optional>

// YAML
#include <yaml-cpp/yaml.h>

// Custom
#include "Types.h"

// file name to write metadata
#define METADATA_FILE "_ros_primitives.yaml"

namespace find_ros_primitives
{

/* Core ROS matching class.
 */
class ROSPrimitiveMatcher
{
  // collection of all the methods we're looking for
  const std::unordered_map<std::string,std::string> ROSMethods {
    // Vetted types:
    {"ros::NodeHandle::advertise", "ros::Publisher"},
    {"ros::NodeHandle::subscribe", "ros::Subscriber"},
    {"ros::NodeHandle::advertiseService", "ros::ServiceServer"},
    {"ros::NodeHandle::serviceClient", "ros::ServiceClient"},
    {"ros::NodeHandle::param", "ros::param"},
    {"ros::NodeHandle::deleteParam", "ros::param"},
    {"ros::NodeHandle::getParam", "ros::param"},
    {"ros::NodeHandle::getParamCached", "ros::param"},
    {"ros::NodeHandle::hasParam", "ros::param"},
    {"ros::NodeHandle::searchParam", "ros::param"},
    {"ros::NodeHandle::setParam", "ros::param"},
    {"ros::init", "ros::Node"},
    {"nodelet::Nodelet::getNodeHandle", "ros::NodeHandle"},
    {"nodelet::Nodelet::getPrivateNodeHandle", "ros::NodeHandle"},
  };
  const std::unordered_map<std::string,std::string> ROSConstructors {
    // Under evaluation:
    {"nodelet::Nodelet", "nodelet::Nodelet"},
    {"actionlib::SimpleActionClient::SimpleActionClient", "actionlib::SimpleActionClient"},
    {"actionlib::SimpleActionServer::SimpleActionServer", "actionlib::SimpleActionServer"},
    {"actionlib::ActionClient::ActionClient", "actionlib::ActionClient"},
    {"actionlib::ActionServer::ActionServer", "actionlib::ActionServer"},
    {"ros::NodeHandle::NodeHandle", "ros::NodeHandle"},
  };

  // YAML representation of collected metadata
  YAML::Node metadata;

 public:
  
  /* Evaluate whether or not the given resolved function call and filename are matches.
   */
  bool is_method(const std::string& function, const std::string& filepath) const;

  /* Evaluate whether or not the given resolved constructor call is a match.
   */
  bool is_constructor(const std::string& constructor, const std::string& filepath) const;

  /* Add newly matched metadata to current set.
   */
  void add_method(const std::string& function, const LocType& location, const std::vector<ArgType>& args);

  /* Add newly matched metadata for the given class constructor.
   */
  void add_constructor(const std::string& constructor, const LocType& location, const std::vector<ArgType>& args);

  /* Attempt to dump collected data.
   */
  void dump(const std::string& base_filepath) const;

  /* Summarize results
   */
  std::optional<std::string> summarize(const std::string& base_filepath) const;

 private:

  void add(const std::string& method, const LocType& location, const std::vector<ArgType>& args);

}; // class ROSPrimitiveMatcher


} // namespace find_ros_primitives
