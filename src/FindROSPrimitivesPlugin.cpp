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
  memset(msg, '\0', MAXCHARS);
  message.copy(msg, MAXCHARS);

  // print message via compiler instance
  clang::DiagnosticsEngine &D = CI->getDiagnostics();
  D.Report(D.getCustomDiagID(level, msg));
}

bool FindROSPrimitivesVisitor::VisitCXXMemberCallExpr(clang::CXXMemberCallExpr *Call)
{

  // get fully qualified function name and file location for match
  const std::string Function = Call->getMethodDecl()->getQualifiedNameAsString();
  const FullSourceLoc FullLocation = Context->getFullLoc(Call->getBeginLoc());

  // reject invalid file locations
  if (!FullLocation.isValid())
    return true;

  // check if this function is a match
  const std::string Filename = Context->getSourceManager().getFilename(FullLocation).str();
  if (!ROSMatcher.is_match(Function, Filename))
    return true;

  // this is a match; extract location informaiton
  LocType Location {Filename, FullLocation.getSpellingLineNumber(), FullLocation.getSpellingColumnNumber()};

  // extract argument information
  // @TODO extract argument type and try to resolve values further
  std::vector<ArgType> Args;
  for (unsigned i = 0; i != Call->getNumArgs(); ++i)
  {
    // get argument value (or default)
    std::string TypeS;
    llvm::raw_string_ostream s(TypeS);
    if (Call->getArg(i)->isDefaultArgument())
      Call->getMethodDecl()->getParamDecl(i)->getDefaultArg()->printPretty(s, 0, Policy);
    else
      Call->getArg(i)->printPretty(s, 0, Policy); 

    // push this back to our list of args
    Args.push_back({
        i,
        Call->getMethodDecl()->getParamDecl(i)->getNameAsString(),
        s.str(),
        "",
        Call->getArg(i)->isDefaultArgument()
    });
  }
  
  // pass collected data back to our Matcher
  ROSMatcher.add(Function, Location, Args);
  return true;
}

bool FindROSPrimitivesVisitor::VisitCallExpr(clang::CallExpr *Call)
{
  // @TODO!
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

