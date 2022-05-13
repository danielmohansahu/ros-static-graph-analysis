This example is taken from the LLVM docs: https://clang.llvm.org/docs/RAVFrontendAction.html

# Usage:

Build instructions: 

```bash
mkdir example/build
cd example/build
cmake ..
make
```

Usage instructions (example):

```bash
./find-class-decls "namespace n { namespace m { class C {}; } }"
# Found declaration at 1:29
```
