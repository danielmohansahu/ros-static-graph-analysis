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
  if (const auto match = ROSMethods.find(Call->getMethodDecl()->getQualifiedNameAsString())
      ; match != ROSMethods.end())
  {
    // check if this location definition is valid; return early otherwise
    FullSourceLoc FullLocation = Context->getFullLoc(Call->getBeginLoc());
    if (!FullLocation.isValid())
      return true;

    // check if this is defined in a header we want to ignore
    // @TODO make this less hardcoded and more extensible
    //  or, alternatively, figure out how to use ASTMatchers to avoid the need entirely...
    const std::string filename = Context->getSourceManager().getFilename(FullLocation).str();
    if (filename.rfind("/opt/ros/") == 0 && filename.rfind("/include/ros/") != std::string::npos)
      return true;

    // if we made it this far this is a good match - construct message
    const auto [class_function_call, class_type] = *match;
    std::stringstream ss;
    // display function name and location message
    ss << "Found " << class_type << " via " << class_function_call;
    ss << "\n\t\t loc:  " << FullLocation.getSpellingLineNumber() << ":"
       << FullLocation.getSpellingColumnNumber() << " in " << filename;

    // store metadata
    YAML::Node instance;
    instance["loc"]["filename"] = filename;
    instance["loc"]["line"] = FullLocation.getSpellingLineNumber();
    instance["loc"]["column"] = FullLocation.getSpellingColumnNumber();

    // extract argument information
    ss << "\n\t\t args: (";
    for (unsigned i = 0; i != Call->getNumArgs(); ++i)
    {
      // get argument value (or default)
      std::string TypeS;
      llvm::raw_string_ostream s(TypeS);
      if (Call->getArg(i)->isDefaultArgument())
        Call->getMethodDecl()->getParamDecl(i)->getDefaultArg()->printPretty(s, 0, Policy);
      else
        Call->getArg(i)->printPretty(s, 0, Policy); 

      // display argument name and value
      ss << Call->getMethodDecl()->getParamDecl(i)->getNameAsString() << "=" << s.str();
      if (i < Call->getNumArgs() - 1)
        ss << ", ";

      // save to metadata structure
      YAML::Node arg_repr;
      arg_repr["name"] = Call->getMethodDecl()->getParamDecl(i)->getNameAsString();
      arg_repr["value"] = s.str();
      instance["args"].push_back(arg_repr);
    }
    ss << ")";

    // display function argument values
    console_print(CI, ss.str());

    // store as metadata
    Metadata[class_type].push_back(instance);
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

