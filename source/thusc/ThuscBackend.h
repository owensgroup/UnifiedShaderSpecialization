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

#include "ThuscCommon.h"
#include "ThuscShaderIR.h"

DISABLE_WARNINGS_LLVM
#include "clang/AST/DeclCXX.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/LangOptions.h"
#include "clang/Basic/SourceManager.h"
#include "clang/Rewrite/Core/Rewriter.h"

#include "llvm/Support/MemoryBuffer.h"
ENABLE_WARNINGS_LLVM

#define WRITELN(X) X << ((X.empty()) ? "" : "\n")

class ThuscBackendBase {
public:
  ThuscBackendBase(const thusc::ShaderIR& shaderIR, clang::Rewriter& rewriter)
    : shaderIR(shaderIR)
    , sourceMgr(rewriter.getSourceMgr())
    , langOpts(rewriter.getLangOpts())
    , diagEngine(rewriter.getSourceMgr().getDiagnostics())
    , rewriter(rewriter)
  {}

  virtual bool Output(const clang::FileID currentFileID) = 0;

protected:
  const thusc::ShaderIR& shaderIR;
  const clang::SourceManager& sourceMgr;
  const clang::LangOptions& langOpts;
  clang::DiagnosticsEngine& diagEngine;
  clang::Rewriter& rewriter;
};

class ThuscBackendHost : public ThuscBackendBase {
public:
  ThuscBackendHost(const thusc::ShaderIR& shaderIR, clang::Rewriter& rewriter, llvm::StringRef gpuOutputDirectory)
    : ThuscBackendBase(shaderIR, rewriter)
    , gpuOutputDir(gpuOutputDirectory)
  {}

protected:
  std::string gpuOutputDir;
};

class ThuscBackendGPU : public ThuscBackendBase {
public:
  ThuscBackendGPU(const thusc::ShaderIR& shaderIR, clang::Rewriter& rewriter, llvm::StringRef gpuOutputDirectory)
    : ThuscBackendBase(shaderIR, rewriter)
    , gpuOutputDir(gpuOutputDirectory)
  {}

protected:
  std::string gpuOutputDir;
};

template<typename HostBackendType, typename GPUBackendType>
class ThuscBackend: public ThuscBackendBase {
public:
  ThuscBackend(const thusc::ShaderIR& shaderIR, clang::Rewriter& rewriter,
    llvm::StringRef gpuOutputDirectory, llvm::StringRef gpuOutputDirectoryVirtual)
    : ThuscBackendBase(shaderIR, rewriter)
    , hostBackend(shaderIR, rewriter, gpuOutputDirectoryVirtual)
    , gpuBackend(shaderIR, rewriter, gpuOutputDirectory)
  {}

  virtual bool Output(const clang::FileID currentFileID) override {
    // GPU backend function must be called first
    // The host backend rewrites the source code, including deleting functions,
    // so the GPU backend must be called first so it has the chance to use the unaltered code
    return gpuBackend.Output(currentFileID) && hostBackend.Output(currentFileID);
  }

protected:
  HostBackendType hostBackend;
  GPUBackendType gpuBackend;
};

