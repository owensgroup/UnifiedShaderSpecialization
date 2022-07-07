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
#include "ThuscBackendHLSL.h"
#include "ThuscShaderIR.h"

class ThuscBackendUE4Host : public ThuscBackendHost {
public:
  ThuscBackendUE4Host(const thusc::ShaderIR& shaderIR, clang::Rewriter& rewriter, llvm::StringRef gpuOutputDirectory)
    : ThuscBackendHost(shaderIR, rewriter, gpuOutputDirectory)
  {}

  virtual bool Output(const clang::FileID currentFileID) override;

private:
  bool OutputShaderClass(const thusc::ShaderClass* shaderClss);
  bool OutputGPUFunction(const thusc::GPUFunction* gpuFunc);

  void OutputUniformFieldStruct(const thusc::UniformField* uniform, llvm::raw_ostream& stream) const;
  void OutputUniformField(const thusc::UniformField* uniform, llvm::raw_ostream& stream) const;
  void OutputUniformFields(const thusc::ShaderClass* shaderClass, const thusc::ImplClass* implClass, llvm::raw_ostream& stream) const;

  void OutputPermutationErrorChecking(const thusc::PermutationField& perm, llvm::raw_ostream& stream) const;
  void OutputPermutationFields(const thusc::ShaderClass* shaderClass, const thusc::ImplClass* implClass, llvm::raw_ostream& stream) const;

  void OutputComputeShaderEntry(const thusc::ComputeEntryPoint* entryPoint, const thusc::ShaderClass* shaderClass, llvm::raw_ostream& stream) const;

  bool OutputImplClass(const thusc::ShaderClass* shaderClass, const thusc::ImplClass* implClass, llvm::raw_ostream& stream) const;


  const char* permutationClassPrefix = "ThuscPermutationClass_";
  const char* thuscTypeIDPrefix = "Thusc_getTypeID_";
  const char* thuscgetImplClassIDFunctionName = "Thusc_getImplClassID";
};

class ThuscBackendUE4 : public ThuscBackend<ThuscBackendUE4Host, ThuscBackendHLSL> {
public:
  ThuscBackendUE4(const thusc::ShaderIR& shaderIR, clang::Rewriter& rewriter,
    llvm::StringRef gpuOutputDirectory, llvm::StringRef gpuOutputDirectoryVirtual)
    : ThuscBackend(shaderIR, rewriter, gpuOutputDirectory, gpuOutputDirectoryVirtual)
  {}
};

