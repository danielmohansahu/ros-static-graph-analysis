/* CLANG Compiler Plugin for ROS Primitive Extraction.
 *
 * Class declarations.
 *
 * Author: Daniel M. Sahu
 */

#pragma once

// STL
#include <string>

// CLANG
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"

// maximum number of characters per output line
#define MAXCHARS 120

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
    : Context(Context), CI(CI) {}

  /* Evaluate a single member function call.
   *
   * E.g.:
   *  - ros::NodeHandle::advertise
   *  - ros::NodeHandle::param
   *
   * TODO:
   *  - Does this also cover class constructors?
   */
  bool VisitCXXMemberCallExpr(clang::CXXMemberCallExpr *Call);

  /* Evaluate a single function call (any)
   *
   * E.g.:
   *  - ros::init
   *
   * TODO:
   *  - Does this also cover member functions?
   */
  bool VisitCallExpr(clang::CallExpr *Call);
 
 private:
  clang::ASTContext *Context;
  clang::CompilerInstance *CI;
};

/* Custom AST Consumer to configure calls to the Visitor.
 */
class FindROSPrimitivesConsumer : public clang::ASTConsumer
{
 public:
  FindROSPrimitivesConsumer(clang::ASTContext *Context,
                            clang::CompilerInstance *Instance,
                            std::set<std::string> ParsedTemplates)
      : Visitor(Context, Instance), ParsedTemplates(ParsedTemplates)
  {}

  /* Core method to analyze a given element in the AST.
   */
  virtual void HandleTranslationUnit(clang::ASTContext &Context)
  {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
  }

 private:
  // Some of these are currently unused, but may be required at later stages of development.
  std::set<std::string> ParsedTemplates;
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
