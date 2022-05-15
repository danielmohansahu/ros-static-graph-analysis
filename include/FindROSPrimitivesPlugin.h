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

// YAML
#include <yaml-cpp/yaml.h>

// CLANG
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"

// maximum number of characters per output line
#define MAXCHARS 512

// file name to write metadata
#define METADATA_FILE "ros_primitives.yaml"

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
  explicit FindROSPrimitivesVisitor(clang::ASTContext *Context, clang::CompilerInstance *CI)
    : Context(Context), CI(CI), Policy(clang::PrintingPolicy(clang::LangOptions())),
      ROSMethods( {
        {"ros::NodeHandle::advertise", " ros::Publisher creation"},
        {"ros::NodeHandle::subscribe", " ros::Subscriber creation"},
        {"ros::NodeHandle::advertiseService", " ros::ServiceServer creation"},
        {"ros::NodeHandle::serviceClient", " ros::ServiceClient creation"},
        {"ros::NodeHandle::param", " ros::param access"},
        {"ros::NodeHandle::deleteParam", " ros::param deletion"},
        {"ros::NodeHandle::getParam", " ros::param access"},
        {"ros::NodeHandle::getParamCached", " ros::param access"},
        {"ros::NodeHandle::hasParam", " ros::param access"},
        {"ros::NodeHandle::searchParam", " ros::param access"},
        {"ros::NodeHandle::setParam", " ros::param modification"}
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
  FindROSPrimitivesConsumer(clang::ASTContext *Context,
                            clang::CompilerInstance *Instance,
                            std::set<std::string> ParsedTemplates)
      : Visitor(Context, Instance), CI(CI), ParsedTemplates(ParsedTemplates)
  {}

  /* Core method to analyze a given element in the AST.
   */
  virtual void HandleTranslationUnit(clang::ASTContext &Context)
  {
    // process the given translation unit
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());

    // get collected metadata
    YAML::Node metadata = Visitor.GetMetadata();

    // write metadata to file
    std::ofstream file(METADATA_FILE);
    file << metadata;
    file.close();
  }

 private:
  // Some of these are currently unused, but may be required at later stages of development.
  std::set<std::string> ParsedTemplates;
  clang::CompilerInstance* CI;
  FindROSPrimitivesVisitor Visitor;
};

/* AST Plugin Action - this is our top level class that interfaces with the Compiler.
 */
class FindROSPrimitivesAction : public clang::PluginASTAction
{
 protected:

  /* Factory method to return our custom consumer.
   */
  std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(clang::CompilerInstance &CI,
                                                        llvm::StringRef) override
  {
    return std::make_unique<FindROSPrimitivesConsumer>(&CI.getASTContext(), &CI, ParsedTemplates);
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
