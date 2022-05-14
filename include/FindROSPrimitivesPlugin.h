/* CLANG Compiler Plugin for ROS Primitive Extraction.
 *
 * Class declarations.
 *
 * Author: Daniel M. Sahu
 */

#pragma once

// STL

// CLANG
#include "clang/AST/ASTConsumer.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/CompilerInstance.h"

namespace find_ros_primitives
{

/* Custom Recursive AST Visitor to analyze each declaration in the AS Tree.
 *
 * This class does the core evaluation of whether or not a given declaration
 * is a ROS primitive.
 */
class FindROSPrimitivesVisitor : public clang::RecursiveASTVisitor<FindROSPrimitivesVisitor>
{
 public:
  explicit FindROSPrimitivesVisitor(clang::ASTContext *Context)
    : Context(Context) {}

  /* Evaluate a single declaration.
   */
  bool VisitCXXRecordDecl(clang::CXXRecordDecl *Declaration);
 
 private:
  clang::ASTContext *Context;
};

/* Custom AST Consumer to configure calls to the Visitor.
 */
class FindROSPrimitivesConsumer : public clang::ASTConsumer
{
 public:
  FindROSPrimitivesConsumer(clang::ASTContext *Context,
                            clang::CompilerInstance &Instance,
                            std::set<std::string> ParsedTemplates)
      : Visitor(Context), Instance(Instance), ParsedTemplates(ParsedTemplates)
  {}

  /* Core method to analyze a given element in the AST.
   */
  virtual void HandleTranslationUnit(clang::ASTContext &Context)
  {
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());
  }

 private:
  // Some of these are currently unused, but may be required at later stages of development.
  clang::CompilerInstance &Instance;
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
    return std::make_unique<FindROSPrimitivesConsumer>(&CI.getASTContext(), CI, ParsedTemplates);
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
