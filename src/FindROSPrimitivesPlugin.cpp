/* CLANG Compiler Plugin for ROS Primitive Extraction.
 *
 */

// STL
#include <sstream>

// CLANG
#include "clang/Frontend/FrontendPluginRegistry.h"

// CUSTOM
#include "FindROSPrimitivesPlugin.h"

using namespace clang;

namespace find_ros_primitives
{

inline void console_print(clang::CompilerInstance* CI,
                          const std::string& message,
                          const clang::DiagnosticsEngine::Level& level)
{
  // display the given string message as compiler output.
  char msg[MAXCHARS];
  memset(msg, ' ', MAXCHARS);
  size_t length = message.copy(msg, MAXCHARS);
  msg[length] = '\0';

  // print message via compiler instance
  clang::DiagnosticsEngine &D = CI->getDiagnostics();
  D.Report(D.getCustomDiagID(level, msg));
}

bool FindROSPrimitivesVisitor::VisitCXXMemberCallExpr(clang::CXXMemberCallExpr *Call)
{
  if (Call->getMethodDecl()->getQualifiedNameAsString() == "ros::NodeHandle::advertise")
  {
    // check if this location definition is valid; return early otherwise
    FullSourceLoc FullLocation = Context->getFullLoc(Call->getBeginLoc());
    if (!FullLocation.isValid())
      return true;

    // check if this is defined in a header we want to ignore
    // @TODO make this less hardcoded and more extensible
    //  or, alternatively, figure out how to use ASTMatchers to avoid the need entirely.
    const std::string filename = Context->getSourceManager().getFilename(FullLocation).str();
    if (filename.rfind("/opt/ros/") == 0 && filename.rfind("/include/ros/") != std::string::npos)
      return true;

    // if we made it this far this is a good match - construct message
    std::stringstream ss;
    ss << "Found ros::Publisher at " << FullLocation.getSpellingLineNumber()
       << ":" << FullLocation.getSpellingColumnNumber() << " in " << filename;

    // display message
    console_print(CI, ss.str());

    // extract argument information
    std::stringstream ss2;
    ss2 << "  parameters: (";
    for (const auto arg : Call->getMethodDecl()->parameters())
    {
      ss2 << arg->getNameAsString() << ", ";
    }
    ss2 << ")";
    console_print(CI, ss2.str());

    std::stringstream ss3;
    ss3 << "  args:       (";
    for (const auto arg : Call->arguments())
    {
      if (arg->isEvaluatable(*Context))
      {
      //    ss3 << Call->getArg(arg) << ", ";
      }
      else
      {
        ss3 << "Unevaluated(" << arg->getStmtClassName() << "), ";
      }
    }
    ss3 << ")";
    console_print(CI, ss3.str());
  }
  return true;
}

bool FindROSPrimitivesVisitor::VisitCallExpr(clang::CallExpr *Call)
{
  return true;
}

bool FindROSPrimitivesAction::ParseArgs(const clang::CompilerInstance &CI,
                                        const std::vector<std::string> &args)
{
  // handle input command line arguments one at a time
  for (unsigned i = 0, e = args.size(); i != e; ++i)
  {
    llvm::errs() << "FindROSPrimitives arg = " << args[i] << "\n";

    // Example error handling.
    clang::DiagnosticsEngine &D = CI.getDiagnostics();
    if (args[i] == "-an-error")
    {
      unsigned DiagID = D.getCustomDiagID(DiagnosticsEngine::Error,
                                          "invalid argument '%0'");
      D.Report(DiagID) << args[i];
      return false;
    }
    else if (args[i] == "-parse-template")
    {
      if (i + 1 >= e)
      {
        D.Report(D.getCustomDiagID(DiagnosticsEngine::Error,
                                   "missing -parse-template argument"));
        return false;
      }
      ++i;
      ParsedTemplates.insert(args[i]);
    }
  }
  if (!args.empty() && args[0] == "help")
    PrintHelp(llvm::errs());

  return true;
}

void FindROSPrimitivesAction::PrintHelp(llvm::raw_ostream& oss)
{
  oss << "Help for FindROSPrimitives plugin goes here\n";
}

} // namespace find_ros_primitives

// plugin registration
static FrontendPluginRegistry::Add<find_ros_primitives::FindROSPrimitivesAction>
X("find-ros-primitives", "find location of ROS primitives");

