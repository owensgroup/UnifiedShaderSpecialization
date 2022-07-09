# thusc
Translator for Heterogeneous Unified Shaders in C++

This repository contains source code supporting the High Performance Graphics 2022 paper:

**Supporting Unified Shader Specialization by Co-opting C++ Features**\
[Kerry A. Seitz, Jr.](https://seitz.tech/), Theresa Foley, [Serban D. Porumbescu](http://graphics.cs.ucdavis.edu/~porumbes), and [John D. Owens](https://www.ece.ucdavis.edu/~jowens/)\
Proceedings of the ACM on Computer Graphics and Interactive Techniques (PACMCGIT)\
Volume 5 Issue 3, July 2022\
Article No. 25

DOI: [https://doi.org/10.1145/3543866](https://doi.org/10.1145/3543866)\
Code: [https://github.com/owensgroup/UnifiedShaderSpecialization](https://github.com/owensgroup/UnifiedShaderSpecialization)


Getting Started
===============

**Note:** This code is currently tested only on 64-bit Windows.

Required Software
=================

1) Microsoft Visual Studio 2019: [https://visualstudio.microsoft.com/vs/older-downloads/](https://visualstudio.microsoft.com/vs/older-downloads/)

   **Note:** Also known as Microsoft Visual Studio 16.0

2) LLVM and Clang version 11.0.0: [https://llvm.org/](https://llvm.org/)

   **Note:** We recommend building LLVM 11 from source with the following CMake arguments:

   * `-DLLVM_ENABLE_PROJECTS=clang`
   * `-DLLVM_TARGETS_TO_BUILD=X86`
   * `-DLLVM_USE_CRT_RELWITHDEBINFO=MDd`
   * `-DMSVC_DIA_SDK_DIR=C:/Program Files (x86)/Microsoft Visual Studio/2019/Community/DIA SDK`
    * (Modify path appropriately for your install of Visual Studio 2019)

  **Note 2:** We recommend building the `Release` and `RelWithDebInfo` builds of LLVM and Clang, which are used with thusc's `Release` and `Debug` builds, respectively.  However, you can change this by modifying `$THSUC/source/thusc/LLVMSpecificProperties-Release.props` and `$THUSC/source/thusc/LLVMSpecificProperties-Debug.props` to build against different LLVM build types.


Building
========

1) Clone the thusc repository. We'll call the directory into which you cloned the repository: `$THUSC`

   ```Shell
   # Make sure to clone with --recursive
   git clone --recursive https://github.com/owensgroup/UnifiedShaderSpecialization.git
   ```

   **Note:** If you didn't clone with the `--recursive` flag, then you need to manually clone the submodules:

   ```Shell
   cd $THUSC
   git submodule update --init --recursive
   ```

2) Modify `$THUSC/source/thusc/ThuscCommon.h` as follows:

   * Modify line 38 `#define THUSC_H_PATH "G:\\workspace\\thusc\\include\\thusc.h"` to point to your cloned copy of the `$THUSC/include/thusc.h` file.

3) Modify `$THUSC/source/thusc/LLVMSpecificProperties.props` as follows:

   * (Optional) Modify line 5 `<LLVMDir>$(SolutionDir)\..\llvm\</LLVMDir>` to point to the location of the parent directory to your LLVM source and build directories.
   * Modify line 6 `<LLVMSrcDir>$(LLVMDir)\llvm-project</LLVMSrcDir>` to point to the location of your LLVM top-level source directory. This directory should contain subdirectories for `llvm` and `clang`.
   * Modify line 7 `<LLVMBuildDir>$(LLVMDir)\build-11.0.0</LLVMBuildDir>` to point to the location of your LLVM build directory.

4) Build thusc by opening `thusc.sln` and building via Visual Studio 2019.  The solution provides both `Debug` and `Release` builds.

5) You should now be able to run the translator tool.  Run with `-h` to see the command-line arguments.


Tips for use with Unreal Engine 4 (UE4)
=======================================

In the paper, the C++ attribute `[[ShaderClass]]` is used to denote a ShaderClass. However, UE4 uses this identifier already.  Instead, use `[[ThuscShader]]` to denote a ShaderClass.


If you are writing code that `#includes` UE4 header files, we recommend generating a [Clang Compilation Database](https://clang.llvm.org/docs/HowToSetupToolingForLLVM.html) from UE4.  The Unreal Build Tool can generate this database for you:

1) Unreal Build Tool expects LLVM and Clang to be installed at `C:\Program Files\LLVM` (it searches for `bin\clang-cl.exe` at that location).  If you did not install LLVM and Clang to this location, you can symbolically link `C:\Program Files\LLVM` to your build.

2) You can then generate the compilation database by running this command:

   ```Shell
   UnrealEngine\Engine\Build\BatchFiles\Build.bat -mode=GenerateClangDatabase -Target="UE4Editor Win64 Development" -WaitMutex
   ```

3) This generated file is very large, and the translator tool takes a long time to parse it.  We recommend making a copy and editing it to contain just the portions you need.
