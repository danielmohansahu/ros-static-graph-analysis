cmake_minimum_required(VERSION 3.5.1)
project(ros-static-graph-analysis)

find_package(Clang REQUIRED)
set(LLVM_LINK_COMPONENTS support)

# Add AST parser executable example
add_executable(find-class-decls FindClassDecls.cpp)
target_include_directories(find-class-decls PRIVATE ${CLANG_INCLUDE_DIRS})

target_link_libraries(find-class-decls
  PRIVATE
  clangAST
  clangASTMatchers
  clangBasic
  clangFrontend
  clangSerialization
  clangTooling
  )

# add dynamic plugin example
# https://github.com/llvm/llvm-project/tree/main/clang/examples/PrintFunctionNames

# If we don't need RTTI or EH, there's no reason to export anything
# from the plugin.
if( NOT MSVC ) # MSVC mangles symbols differently, and
               # PrintFunctionNames.export contains C++ symbols.
  if( NOT LLVM_REQUIRES_RTTI )
    if( NOT LLVM_REQUIRES_EH )
      set(LLVM_EXPORTED_SYMBOL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/PrintFunctionNames.exports)
    endif()
  endif()
endif()

add_library(PrintFunctionNames MODULE PrintFunctionNames.cpp)
target_include_directories(PrintFunctionNames PRIVATE ${CLANG_INCLUDE_DIRS})

# add find named class plugin example

add_library(FindNamedClassPlugin MODULE FindClassDeclsPlugin.cpp)
target_include_directories(FindNamedClassPlugin PRIVATE ${CLANG_INCLUDE_DIRS})
