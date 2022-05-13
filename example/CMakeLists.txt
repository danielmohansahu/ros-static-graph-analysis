cmake_minimum_required(VERSION 3.5.1)
project(ros-static-graph-analysis)

find_package(Clang REQUIRED)
set(LLVM_LINK_COMPONENTS support)

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
