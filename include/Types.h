/* Definitions of shared types across codebase.
 *
 * Author: Daniel M. Sahu
 */

#pragma once

// STL
#include <string>

// YAML
#include <yaml-cpp/yaml.h>

namespace find_ros_primitives
{

/* Structure containing general argument information.
 */
struct ArgType
{
  unsigned int index; // argument order information
  std::string name;   // argument name
  std::string value;  // argument value
  std::string type;   // argument type
  bool default_value; // whether or not this is the default value (i.e. unset)

  // constructor
  ArgType()=delete;
  ArgType(unsigned int index, const std::string& name, const std::string& value, const std::string& type, bool default_value)
    : index(index), name(name), value(value), type(type), default_value(default_value) {}

  // convert to a YAML node
  YAML::Node as_yaml() const
  {
    YAML::Node res;
    res["index"] = index;
    res["name"] = name;
    res["value"] = value;
    res["type"] = type;
    res["default_value"] = default_value;
    return res;
  }

}; // struct ArgType

/* Structure containing location specific information.
 */
struct LocType
{
  std::string filename; // full path to source file
  unsigned int line;    // line number
  unsigned int column;  // column number

  // constructor
  LocType()=delete;
  LocType(const std::string& filename, unsigned int line, unsigned int column)
    : filename(filename), line(line), column(column) {}

  // convert to a YAML node
  YAML::Node as_yaml() const
  {
    YAML::Node res;
    res["filename"] = filename;
    res["line"] = line;
    res["column"] = column;
    return res;
  }
}; // struct LocType

} // namespace find_ros_primitives
