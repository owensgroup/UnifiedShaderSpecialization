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

#pragma once

#include "ThuscBackend.h"

#include "ThuscCppToHLSLTranslator.h"

DISABLE_WARNINGS_LLVM
#include "clang/AST/RecursiveASTVisitor.h"
ENABLE_WARNINGS_LLVM

namespace thusc {

// TODO Fix this hack. It's a quick-and-dirty way to get the corresponding command line option to this file
extern bool outputAllGPUFuncs;

} // namespace thusc

class ThuscBackendHLSL : public ThuscBackendGPU {
public:
  ThuscBackendHLSL(const thusc::ShaderIR& shaderIR, clang::Rewriter& rewriter, llvm::StringRef gpuOutputDirectory)
    : ThuscBackendGPU(shaderIR, rewriter, gpuOutputDirectory)
  {
    for (const auto& gpuFunc : shaderIR.gpuFunctions) {
      clangFuncToThuscFunc.insert(std::make_pair(gpuFunc->getFunctionDecl(), gpuFunc.get()));
    }

    for (const auto& shaderClass : shaderIR.getShaderClasses()) {
      for (const auto& gpuMethod : shaderClass->getGPUMethods()) {
        clangFuncToThuscFunc.insert(std::make_pair(gpuMethod->getFunctionDecl(), gpuMethod.get()));
      }
    }
  }

  virtual bool Output(const clang::FileID currentFileID) override;

private:
  bool OutputShaderClass(const thusc::ShaderClass* shaderClass);

  void OutputPermutationShaderClassFields(const thusc::ShaderClass* shaderClass, llvm::raw_ostream& stream) const;
  void OutputPermutationFields(const thusc::ShaderClass* shaderClass, llvm::raw_ostream& stream) const;
  void OutputUniformFields(const thusc::ShaderClass*, llvm::raw_ostream& stream, llvm::StringRef prefix = "") const;

  void OutputSelection(const thusc::PermutationShaderClassSelection& selection, llvm::raw_ostream& stream, llvm::StringRef prevPrefix = "" ) const;

  void OutputUniformFieldSimple(const thusc::UniformField* uniform, llvm::raw_ostream& stream, const llvm::Twine& prefix) const;
  void OutputFlattenedUniformStruct(const clang::CXXRecordDecl* classDecl, llvm::raw_ostream& stream, const llvm::Twine& prefix) const;
  void OutputUniformFieldStruct(const thusc::UniformField* uniform, llvm::raw_ostream& stream, const llvm::Twine& prefix) const;
  void OutputUniformField(const thusc::UniformField* uniform, llvm::raw_ostream& stream, const llvm::Twine& prefix) const;

  void OutputFunctionDeclaration(const thusc::GPUFunctionBase* gpuFunc, llvm::raw_ostream& stream, bool shouldOutputSemantics, llvm::StringRef prefix = "") const;
  void OutputFunctionBody(const clang::FunctionDecl* funcDecl, llvm::raw_ostream& stream) const;

  void OutputGPUFunction(const thusc::GPUFunction* gpuFunc, llvm::raw_ostream& stream, llvm::StringRef prefix = "") const;

  void OutputShaderEntryCommon(const thusc::EntryPoint* entryPoint, llvm::raw_ostream& stream) const;
  void OutputComputeShaderEntry(const thusc::ComputeEntryPoint* entryPoint, llvm::raw_ostream& stream) const;

  llvm::DenseMap<const clang::FunctionDecl*, const thusc::GPUFunction*> clangFuncToThuscFunc;
  std::unique_ptr<thusc::CppToHLSLVisitor> translator = nullptr;
};

