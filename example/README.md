This example is taken from the LLVM docs: https://clang.llvm.org/docs/RAVFrontendAction.html

# Usage:

Build instructions: 

```bash
mkdir example/build
cd example/build
cmake ..
make
cd ..
```

Usage instructions (example):

```bash
./build/find-class-decls "namespace n { namespace m { class C {}; } }"
# Found declaration at 1:29
```

```bash
# examine a C++ file and print out top level elements
clang -cc1 -load build/libPrintFunctionNames.so -plugin print-fns foo.cc
```
