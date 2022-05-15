/* CLANG Compiler Plugin for ROS Primitive Extraction.
 *
 * Class declarations.
 *
 * Author: Daniel M. Sahu
 */

#pragma once

// STL
#include <fstream>
#include <string>
#include <unordered_map>
#include <filesystem>

// YAML
#include <yaml-cpp/yaml.h>

// CLANG
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"

// maximum number of characters per output line
#define MAXCHARS 512

// file name to write metadata
#define METADATA_FILE "_ros_primitives.yaml"

namespace find_ros_primitives
{

/* Display console output.
 */
inline void console_print(clang::CompilerInstance* CI,
                          const std::string& message,
                          const clang::DiagnosticsEngine::Level& level = clang::DiagnosticsEngine::Remark);

/* Custom Recursive AST Visitor to analyze each declaration in the AS Tree.
 *
 * This class does the core evaluation of whether or not a given declaration
 * is a ROS primitive.
 */
class FindROSPrimitivesVisitor : public clang::RecursiveASTVisitor<FindROSPrimitivesVisitor>
{
 public:
  explicit FindROSPrimitivesVisitor(clang::CompilerInstance *CI)
    : Context(&CI->getASTContext()), CI(CI), Policy(clang::PrintingPolicy(clang::LangOptions())),
      ROSMethods( {
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
        {"ros::NodeHandle::setParam", "ros::param"}
      } )
  {
    // format for C++
    Policy.adjustForCPlusPlus();
  }

  /* Accessor for the collected metadata.
   */
  YAML::Node GetMetadata()
  {
    return Metadata;
  }

  /* Evaluate a single member function call.
   *
   * I.e.:
   *  - ros::NodeHandle::advertise
   *  - ros::NodeHandle::subscribe
   *  - ros::NodeHandle::advertiseService
   *  - ros::NodeHandle::serviceClient
   *  - ros::NodeHandle::param
   *  - ros::NodeHandle::deleteParam
   *  - ros::NodeHandle::getParam
   *  - ros::NodeHandle::getParamCached
   *  - ros::NodeHandle::hasParam
   *  - ros::NodeHandle::searchParam
   *  - ros::NodeHandle::setParam
   *
   * TODO:
   *  - Does this also cover class constructors?
   */
  bool VisitCXXMemberCallExpr(clang::CXXMemberCallExpr *Call);

  /* Evaluate a single function call (any)
   *
   * I.e.:
   *  - ros::init
   *
   * TODO:
   *  - Does this also cover member functions?
   */
  bool VisitCallExpr(clang::CallExpr *Call);
 
 private:
  // Context in the AST
  clang::ASTContext *Context;
  // pointer to the compiler instance, for logging
  clang::CompilerInstance *CI;
  // Language formatting
  clang::PrintingPolicy Policy;
  // collection of all the methods we're looking for
  std::unordered_map<std::string,std::string> ROSMethods;
  // YAML representation of collected metadata
  YAML::Node Metadata;
};

/* Custom AST Consumer to configure calls to the Visitor.
 */
class FindROSPrimitivesConsumer : public clang::ASTConsumer
{
 public:
  FindROSPrimitivesConsumer(clang::CompilerInstance* CI,
                            llvm::StringRef InFile)
      : Visitor(CI), CI(CI), Filepath(std::filesystem::path(InFile))
  {}

  /* Core method to analyze a given element in the AST.
   */
  virtual void HandleTranslationUnit(clang::ASTContext &Context)
  {
    // process the given translation unit
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());

    // get collected metadata
    YAML::Node metadata = Visitor.GetMetadata();

    // write metadata to file (uniquely named based on current top level file)
    std::string filename = Filepath.stem().string() + std::string(METADATA_FILE);
    std::ofstream file(filename);
    file << metadata;
    file.close();
  }

 private:
  // Instance of the compiler, for general state information
  clang::CompilerInstance* CI;
  // Visitor instance doing the processing
  FindROSPrimitivesVisitor Visitor;
  // Top level file we're currently investigating
  std::filesystem::path Filepath;
};

/* AST Plugin Action - this is our top level class that interfaces with the Compiler.
 */
class FindROSPrimitivesAction : public clang::PluginASTAction
{
 protected:

  /* Factory method to return our custom consumer.
   */
  std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(clang::CompilerInstance &CI,
                                                        llvm::StringRef InFile) override
  {
    return std::make_unique<FindROSPrimitivesConsumer>(&CI, InFile);
  }

  /* Parse command line input arguments.
   */
  bool ParseArgs(const clang::CompilerInstance &CI, const std::vector<std::string> &args) override;

  /* Human readable help usage.
   */
  void PrintHelp(llvm::raw_ostream& oss);

 private:
  std::set<std::string> ParsedTemplates;
};

} // namespace find_ros_primitives
