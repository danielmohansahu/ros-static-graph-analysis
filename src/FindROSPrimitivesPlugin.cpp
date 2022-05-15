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

bool FindROSPrimitivesVisitor::VisitCallExpr(clang::CallExpr *Call)
{
  // get base method declaration (if extant)
  FunctionDecl* Declaration = nullptr;
  if (Declaration = Call->getDirectCallee(); !Declaration)
    return true;

  // get fully qualified function name and file location for match
  const FullSourceLoc FullLocation = Context->getFullLoc(Call->getBeginLoc());

  // reject invalid file locations
  if (!FullLocation.isValid())
    return true;

  // check if this function is a match
  std::string MethodName = Declaration->getQualifiedNameAsString();
  clean_string(MethodName);
  const std::string Filename = Context->getSourceManager().getFilename(FullLocation).str();
  if (!ROSMatcher.is_method(MethodName, Filename))
    return true;

  // this is a match; extract location informaiton
  LocType Location {Filename, FullLocation.getSpellingLineNumber(), FullLocation.getSpellingColumnNumber()};

  // extract argument information
  // @TODO extract argument type and try to resolve values further
  std::vector<ArgType> Args;
  for (unsigned i = 0; i != Call->getNumArgs(); ++i)
  {
    // initialize pretty print ostream object
    std::string TypeS;
    llvm::raw_string_ostream s(TypeS);

    // get argument value (or default)
    clang::Expr* Arg;
    if (Call->getArg(i)->isDefaultArgument())
      Arg = Declaration->getParamDecl(i)->getDefaultArg();
    else
      Arg = Call->getArg(i);

    // get the raw value as printed
    Arg->printPretty(s, 0, Policy);
    std::string raw_value = s.str();

    // try to evaluate
    std::string value = raw_value;
    if (clang::Expr::EvalResult res;
        Arg->EvaluateAsRValue(res, *Context, /*don't_break_things=*/true))
      value = res.Val.getAsString(*Context, Arg->getType());

    // push this back to our list of args
    Args.push_back({
        Declaration->getParamDecl(i)->getNameAsString(),
        raw_value,
        value,
        QualType::getAsString(Arg->getType().split(), Policy),
        Call->getArg(i)->isDefaultArgument(),
    });
  }

  // get unique identifier for the class we're calling this from
  int ObjectID = -1;
  {
    // taken from:
    //  https://clang.llvm.org/doxygen/ExprCXX_8cpp_source.html#l00650
    const Expr *Callee = Call->getCallee()->IgnoreParens();
    if (const auto *MemExpr = dyn_cast<MemberExpr>(Callee))
      ObjectID = MemExpr->getBase()->getID(*Context);
    if (const auto *BO = dyn_cast<BinaryOperator>(Callee))
      if (BO->getOpcode() == BO_PtrMemD || BO->getOpcode() == BO_PtrMemI)
        ObjectID = BO->getLHS()->getID(*Context);
  }

  // pass collected data back to our Matcher
  ROSMatcher.add_method(MethodName, Location, Args, ObjectID);
  return true;
}

bool FindROSPrimitivesVisitor::VisitCXXConstructExpr(clang::CXXConstructExpr *Call)
{
  // @TODO try to reduce the amount of duplicate code between this and the CallExpr method.

  // Get declaration for this constructor expression
  CXXConstructorDecl* Declaration = Call->getConstructor();

  // get fully qualified function name and file location for match
  const FullSourceLoc FullLocation = Context->getFullLoc(Call->getBeginLoc());

  // reject invalid file locations
  if (!FullLocation.isValid())
    return true;

  // check if this function is a match
  std::string ConstructorName = Declaration->getQualifiedNameAsString();
  clean_string(ConstructorName);
  const std::string Filename = Context->getSourceManager().getFilename(FullLocation).str();
  if (!ROSMatcher.is_constructor(ConstructorName, Filename))
    return true;

  // this is a match; extract location informaiton
  LocType Location {Filename, FullLocation.getSpellingLineNumber(), FullLocation.getSpellingColumnNumber()};

  // extract argument information
  // @TODO extract argument type and try to resolve values further
  std::vector<ArgType> Args;
  for (unsigned i = 0; i != Call->getNumArgs(); ++i)
  {
    // initialize pretty print ostream object
    std::string TypeS;
    llvm::raw_string_ostream s(TypeS);

    // get argument value (or default)
    clang::Expr* Arg;
    if (Call->getArg(i)->isDefaultArgument())
      Arg = Declaration->getParamDecl(i)->getDefaultArg();
    else
      Arg = Call->getArg(i);

    // get the raw value as printed
    Arg->printPretty(s, 0, Policy);
    std::string raw_value = s.str();

    // try to evaluate
    std::string value = raw_value;
    if (clang::Expr::EvalResult res;
        Arg->EvaluateAsRValue(res, *Context, /*don't_break_things=*/true))
      value = res.Val.getAsString(*Context, Arg->getType());

    // push this back to our list of args
    Args.push_back({
        Declaration->getParamDecl(i)->getNameAsString(),
        raw_value,
        value,
        QualType::getAsString(Arg->getType().split(), Policy),
        Call->getArg(i)->isDefaultArgument(),
    });
  }

  // get the unique ID of the resulting object
  // @TODO this doesn't seem right...
  const int ObjectID = Call->getExprStmt()->getID(*Context);
  
  // pass collected data back to our Matcher
  ROSMatcher.add_constructor(ConstructorName, Location, Args, ObjectID);
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

