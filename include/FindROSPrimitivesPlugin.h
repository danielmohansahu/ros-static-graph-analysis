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

// Custom
#include "Types.h"
#include "ROSPrimitiveMatcher.h"

// maximum number of characters per output line
#define MAXCHARS 512

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
    : Context(&CI->getASTContext()), CI(CI), Policy(clang::PrintingPolicy(clang::LangOptions()))
  {
    // format for C++
    Policy.adjustForCPlusPlus();
  }

  /* Save the collated metadata to file.
   */
  void DumpMetadata(const std::string& InFile)
  {
    ROSMatcher.dump(InFile);

    // also print a summary to console
    if (const auto summary = ROSMatcher.summarize(InFile); summary)
      console_print(CI, *summary);
  }

  /* Evaluate a single function call (any)
   *
   * E.g.:
   *  - std::blah();
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
  // ROS matching class
  ROSPrimitiveMatcher ROSMatcher;
};

/* Custom AST Consumer to configure calls to the Visitor.
 */
class FindROSPrimitivesConsumer : public clang::ASTConsumer
{
 public:
  FindROSPrimitivesConsumer(clang::CompilerInstance* CI,
                            llvm::StringRef InFile)
      : Visitor(CI), CI(CI), InFile(InFile)
  {}

  /* Core method to analyze a given element in the AST.
   */
  virtual void HandleTranslationUnit(clang::ASTContext &Context)
  {
    // process the given translation unit
    Visitor.TraverseDecl(Context.getTranslationUnitDecl());

    // dump metadata to file
    Visitor.DumpMetadata(InFile);
  }

 private:
  // Instance of the compiler, for general state information
  clang::CompilerInstance* CI;
  // Visitor instance doing the processing
  FindROSPrimitivesVisitor Visitor;
  // Top level file we're currently investigating
  std::string InFile;
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
