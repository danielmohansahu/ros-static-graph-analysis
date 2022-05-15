/* Class implementation for ROSPrimitiveMatcher
 *
 * Author: Daniel M. Sahu
 */

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
  // @TODO
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

std::optional<std::string> ROSPrimitiveMatcher::summarize() const
{
  // @TODO
  return std::nullopt;
}

} // namespace find_ros_primitives

