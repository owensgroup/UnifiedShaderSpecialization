// Copyright (c) 2022, The Regents of the University of California,
// Davis campus. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither any name of The Regents of the University of California nor
//     the names of its contributors may be used to endorse or promote
//     products derived from this software without specific prior written
//     permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// thusc.cpp

#include "ThuscBackendUE4.h"
#include "ThuscCommon.h"
#include "ThuscFrontendAction.h"

#include <string>
#include <vector>

DISABLE_WARNINGS_LLVM
#include "clang/Tooling/CommonOptionsParser.h"
#include "clang/Tooling/CompilationDatabase.h"
#include "clang/Tooling/Tooling.h"

#include "llvm/Support/CommandLine.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ToolOutputFile.h"
ENABLE_WARNINGS_LLVM

using namespace clang::tooling;
using namespace llvm;

static cl::OptionCategory thuscOptionsCategory("thusc options");

static cl::opt<std::string> hostOutputFilename("o", cl::desc("Output filename for Host code"), cl::value_desc("filename"),
  cl::cat(thuscOptionsCategory), cl::Required);

static cl::opt<std::string> gpuOutputDirectory("gpuOutDir", cl::desc("Output directory for GPU code"), cl::value_desc("directory"),
  cl::cat(thuscOptionsCategory), cl::Required);

static cl::opt<std::string> gpuOutputDirectoryVirtual("gpuOutDirVirtual", cl::desc("Corresponding virtual path for directory specified in gpuOutDir (UE4 backend only)"), cl::value_desc("directory"),
  cl::cat(thuscOptionsCategory), cl::Required);

static cl::opt<bool> disableNameManglingOpt("disableNameMangling", cl::desc("Disables name mangling of GPU functions"), cl::cat(thuscOptionsCategory));
bool thusc::disableNameMangling = false;

static cl::opt<bool> outputAllGPUFuncsOpt("outputAllGPUFuncs", cl::desc("Output all GPU functions to the HLSL file, whether or not they are used"), cl::cat(thuscOptionsCategory));
bool thusc::outputAllGPUFuncs = false;

static cl::extrahelp CommonHelp(CommonOptionsParser::HelpMessage);

#define ADD_ANNOTATION_OPTION(macroName, spelling, extraArgs) \
  "--extra-arg=-D" macroName "=clang::annotate(\"" spelling "\"" extraArgs ")"

void addExtraClangArgs(std::vector<const char*>* args) {
  // TODO These extra arguments probably be specified alongside the files to compile and parsed in,
  //      rather than be hardcoded here.
  static const char* extraArgs[] = {
    "--extra-arg=-DUNREAL_CODE_ANALYZER=1",
    "--extra-arg=-Wno-nonportable-include-path",
    "--extra-arg=-Wno-inconsistent-missing-override",
    "--extra-arg=-Wno-microsoft-unqualified-friend",
    "--extra-arg=-Wno-gnu-string-literal-operator-template",
    "--extra-arg=-Wno-ignored-attributes",
    "--extra-arg=-Wno-switch",
    "--extra-arg=-Wno-tautological-undefined-compare",
    "--extra-arg=-Wno-invalid-offsetof",

    // TODO This path is hardcoded. Figure out how to get this path in a more portable way.
    "--extra-arg=-include" THUSC_H_PATH,

    // Thusc-specific #defines
    ADD_ANNOTATION_OPTION("ThuscShader", THUSC_SHADER_SPELLING, ""),

    // Uniforms
    ADD_ANNOTATION_OPTION("uniform", THUSC_UNIFORM_SPELLING, ""),

    // Permutations / Specializations
    ADD_ANNOTATION_OPTION("permutation_enum", THUSC_PERMUTATION_ENUM_SPELLING, ""),
    ADD_ANNOTATION_OPTION("permutation_bool", THUSC_PERMUTATION_BOOL_SPELLING, ""),
    ADD_ANNOTATION_OPTION("permutation_int(x)", THUSC_PERMUTATION_INT_SPELLING, " #x"),
    ADD_ANNOTATION_OPTION("permutation_sparseInt(...)", THUSC_PERMUTATION_SPARSE_INT_SPELLING, " #__VA_ARGS__"),
    ADD_ANNOTATION_OPTION("permutation_ShaderClass", THUSC_PERMUTATION_SHADERCLASS_SPELLING, ""),

    ADD_ANNOTATION_OPTION("specialization_Enum", THUSC_PERMUTATION_ENUM_SPELLING, ""),
    ADD_ANNOTATION_OPTION("specialization_Bool", THUSC_PERMUTATION_BOOL_SPELLING, ""),
    ADD_ANNOTATION_OPTION("specialization_Int(x)", THUSC_PERMUTATION_INT_SPELLING, " #x"),
    ADD_ANNOTATION_OPTION("specialization_SparseInt(...)", THUSC_PERMUTATION_SPARSE_INT_SPELLING, " #__VA_ARGS__"),
    ADD_ANNOTATION_OPTION("specialization_ShaderClass", THUSC_PERMUTATION_SHADERCLASS_SPELLING, ""),

    // Function parameters
    ADD_ANNOTATION_OPTION("SV_DispatchThreadID", THUSC_SV_DISPATCH_THREAD_ID_SPELLING, ""),
    ADD_ANNOTATION_OPTION("SV_GroupThreadID", THUSC_SV_GROUP_THREAD_ID_SPELLING, ""),
    ADD_ANNOTATION_OPTION("SV_GroupID", THUSC_SV_GROUP_ID_SPELLING, ""),
    ADD_ANNOTATION_OPTION("SV_GroupIndex", THUSC_SV_GROUP_INDEX_SPELLING, ""),
    ADD_ANNOTATION_OPTION("inout", THUSC_HLSL_INOUT_SPELLING, ""),

    // Shader entry functions
    ADD_ANNOTATION_OPTION("entry_ComputeShader(x, y, z)", THUSC_ENTRY_COMPUTE_SPELLING, " #x \",\" #y \",\" #z"),
    ADD_ANNOTATION_OPTION("entry_PixelShader", THUSC_ENTRY_PIXEL_SPELLING, ""),

    // GPU functions
    ADD_ANNOTATION_OPTION("gpu", THUSC_GPU_FN_SPELLING, ""),
    ADD_ANNOTATION_OPTION("PrecodeHLSL", THUSC_HLSL_PRECODE_SPELLING, ""),
    ADD_ANNOTATION_OPTION("PostcodeHLSL", THUSC_HLSL_POSTCODE_SPELLING, ""),

    // Backend-specific #defines
    // TODO move these to a backend-specific file
    "--extra-arg=-DTHUSC_SHADER(TY, SUPERTYPE)="
      "void ThuscShaderStubFunc() {};"
      "void AddComputePass(FRDGBuilder& GraphBuilder, FRDGEventName&& PassName, const FGlobalShaderMap* ShaderMap, FIntVector GroupCount) { }"
      THUSC_TYPE_ADAPTERS
  };

  args->insert(args->end(), std::begin(extraArgs), std::end(extraArgs));
}

// Returns false if the arguments did not produce a valid source path and corresponding compile command
// Returns true otherwise
bool validateArguments(CommonOptionsParser& optionsParser) {
  if (optionsParser.getSourcePathList().size() == 0) {
    // If no input soruce file is found, report an error
    llvm::errs() << "Error: No source file found for the specified arguments.\n";
    return false;
  }
  else if (optionsParser.getSourcePathList().size() != 1) {
    // thusc should be run on one input file at a time, to produce one source-to-source translation of that file.
    // The translation will be written to the output file specified by -o (or stdout by default).
    // If multiple source paths are found, report an error.
    llvm::errs() << "Error: Multiple source files found for the specified arguments.\n";
    return false;
  }

  const std::string sourcePath = optionsParser.getSourcePathList()[0];
  const std::vector<CompileCommand> commands = optionsParser.getCompilations().getCompileCommands(sourcePath);

  if (commands.size() == 0) {
    // If there are no valid compile commands for the input file, report an error.
    llvm::errs() << "Error: No compile command found for input file <" << sourcePath << ">.\n";
    return false;
  }
  else if (commands.size() != 1) {
    // Since thusc produces one source-to-source translation per input file, it cannot handle the case where
    // there are mutliple entries found for that file in the compilation database. So report an error.
    // (This may change later.)
    llvm::errs() << "Error: Multiple compile commands found for input file <" << sourcePath << ">. "
      "(The compilation database may contain multiple entries for this file, which is currently unsupported.)\n";
    return false;
  }
  else {
    CompileCommand cc = commands[0];

    if (cc.Heuristic != std::string("")) {
      // The heuristic used to determine compile commands when an exact match isn't found in the compilation database
      // does not seem to do a great job. For now, report an error if a heuristic was used to find the compile command.
      llvm::errs() << "Error: The compile command was determined by a heuristic, "
        "which might mean that the exact input file could not be found in the compilation database. "
        "The reported heuristic is: " << cc.Heuristic << "\n";
      return false;
    }
  }

  // If all of the checks have passed, then the arguments have produced a valid source path and compile command
  return true;
}

int main(int argc, const char** argv) {
  std::vector<const char*> clangArgs(argv, argv + argc);
  addExtraClangArgs(&clangArgs);

  int clangArgsCount = (int)clangArgs.size();
  CommonOptionsParser optionsParser(clangArgsCount, clangArgs.data(), thuscOptionsCategory, cl::NumOccurrencesFlag::Required);

  if (!validateArguments(optionsParser)) {
    return EXIT_FAILURE;
  }

  if (clangArgsCount != clangArgs.size()) {
    // TODO I'm not sure of the implications if the argument count is changed, so report an error for now.
    llvm::errs() << "Error: CommonOptionsParser changed the argument count.\n";
    return EXIT_FAILURE;
  }

  std::error_code ec;
  llvm::ToolOutputFile hostOutputFile(hostOutputFilename, ec, llvm::sys::fs::OpenFlags::OF_None);
  if (ec) {
    llvm::errs() << "Error: Could not open output file '" << hostOutputFilename << "': " << ec.message() << "\n";
    return EXIT_FAILURE;
  }

  bool isDirectory = false;
  ec = llvm::sys::fs::is_directory(gpuOutputDirectory, isDirectory);
  if (ec) {
    llvm::errs() << "Error: Could not access GPU output directory '" << gpuOutputDirectory << "': " << ec.message() << "\n";
    return EXIT_FAILURE;
  }
  else if (!isDirectory) {
    llvm::errs() << "Error: Specified GPU output directory '" << gpuOutputDirectory << "' is not a directory\n";
    return EXIT_FAILURE;
  }


  // TODO Other backends?
  ThuscFrontendActionFactory<ThuscBackendUE4> factory(hostOutputFile.os(), gpuOutputDirectory, gpuOutputDirectoryVirtual);
  thusc::disableNameMangling = disableNameManglingOpt;
  thusc::outputAllGPUFuncs = outputAllGPUFuncsOpt;

  ClangTool tool(optionsParser.getCompilations(), optionsParser.getSourcePathList());
  int toolExitCode = tool.run(&factory);

  if (toolExitCode) {
    return toolExitCode;
  }

  hostOutputFile.keep();

  return toolExitCode;
}

