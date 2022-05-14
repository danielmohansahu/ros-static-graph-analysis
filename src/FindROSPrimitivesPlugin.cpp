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

bool FindROSPrimitivesVisitor::VisitCXXRecordDecl(clang::CXXRecordDecl *Declaration)
{
  if (Declaration->getQualifiedNameAsString() == "ros::Publisher")
  {
    FullSourceLoc FullLocation = Context->getFullLoc(Declaration->getBeginLoc());
    if (FullLocation.isValid())
    {
      // get source manager to decode file name
      SourceManager& SrcMgr = Context->getSourceManager();

      // construct message
      std::stringstream ss;
      ss << "Found ros::Publisher at " << FullLocation.getSpellingLineNumber()
         << ":" << FullLocation.getSpellingColumnNumber() << " in "
         << SrcMgr.getFilename(FullLocation).str();

      // display message
      console_print(CI, ss.str());
    }
  }
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

