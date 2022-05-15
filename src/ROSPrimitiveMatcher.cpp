/* Class implementation for ROSPrimitiveMatcher
 *
 * Author: Daniel M. Sahu
 */

// STL
#include <assert.h>

// Custom
#include "ROSPrimitiveMatcher.h"

namespace find_ros_primitives
{

bool ROSPrimitiveMatcher::is_match(const std::string& function, const std::string& filepath) const
{
  // check if function name is a match
  if (ROSMethods.find(function) == ROSMethods.end())
    return false;

  // check if this is defined in a header we want to ignore
  // @TODO make this less hardcoded and more extensible
  //  or, alternatively, figure out how to use ASTMatchers to avoid the need entirely...
  if (filepath.rfind("/opt/ros/") == 0 && filepath.rfind("/include/ros/") != std::string::npos)
    return false;

  // otherwise, it's a match
  return true;
}

void ROSPrimitiveMatcher::add(const std::string& function, const LocType& location, const std::vector<ArgType>& args)
{
  // sanity check
  assert(ROSMethods.find(function) != ROSMethods.end());

  // get the primitive name
  const std::string primitive = ROSMethods.find(function)->second;

  // store this information in YAML format
  YAML::Node data;
  data["loc"] = location.as_yaml();
  for (const auto arg : args)
    data["args"].push_back(arg.as_yaml());
  metadata[primitive].push_back(data);
}

void ROSPrimitiveMatcher::dump(const std::string& base_filepath) const
{
  // check if we've got any data at all
  if (metadata.size() == 0)
    return;

  // write metadata to file (uniquely named based on current top level file)
  std::filesystem::path filepath(base_filepath);
  std::ofstream file(filepath.stem().string() + std::string(METADATA_FILE));
  file << metadata;
  file.close();
}

std::optional<std::string> ROSPrimitiveMatcher::summarize(const std::string& base_filepath) const
{
  // return nothing if we have no data
  if (metadata.size() == 0)
    return std::nullopt;

  // generate summary
  std::stringstream ss;
  ss << "Found ";
  for (auto it = metadata.begin(); it != metadata.end(); ++it)
  {
    ss << it->second.size() << " " << it->first.as<std::string>();
    if (std::distance(it, metadata.end()) > 1)
      ss << ", ";
  }
  ss << " in " << std::filesystem::path(base_filepath).filename().string();

  // return resulting summary
  return std::make_optional(ss.str());
}

} // namespace find_ros_primitives

