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
#include "ThuscCommon.h"
#include "ThuscShaderIR.h"

DISABLE_WARNINGS_LLVM
#include "clang/AST/ASTContext.h"
#include "clang/AST/DeclCXX.h"
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Frontend/ASTConsumers.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/FrontendAction.h"
#include "clang/Rewrite/Core/Rewriter.h"
#include "clang/Tooling/Tooling.h"

#include "llvm/ADT/StringRef.h"
ENABLE_WARNINGS_LLVM

class ThuscASTVisitor : public clang::RecursiveASTVisitor<ThuscASTVisitor> {
public:
  ThuscASTVisitor(clang::Rewriter& r)
    : rewriter(r)
    , sourceMgr(r.getSourceMgr())
    , langOpts(r.getLangOpts())
    , diagEngine(r.getSourceMgr().getDiagnostics())
    , printingPolicy(langOpts)
  {
    printingPolicy.SuppressUnwrittenScope = 1;
  }

  void setShaderIR(thusc::ShaderIR* IR) {
    shaderIR = IR;
  }

  bool VisitCXXRecordDecl(clang::CXXRecordDecl* classDecl);
  bool VisitFunctionDecl(clang::FunctionDecl* funcDecl);
  bool VisitDecl(clang::Decl* Decl);

private:
  bool HandleUniformFields(clang::CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass);
  bool HandlePermutationFields(clang::CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass);
  bool HandleShaderEntryFunctions(const clang::CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass);

  bool HandleOtherGPUDecls(const clang::CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass);
  bool HandleGPUMethods(const clang::CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass);
  bool HandleHLSLPrecodePostcode(const clang::CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass);

  // Helper functions
  std::unique_ptr<thusc::GPUFunction> CreateGPUFunctionIR(const clang::FunctionDecl* funcDecl);
  thusc::ShaderParameterSemanticKind getComputeEntryParameterSemantic(const clang::ParmVarDecl* param,
                                                                      const clang::CXXRecordDecl* shaderClassDecl) const;

  clang::Rewriter& rewriter;
  thusc::ShaderIR* shaderIR = nullptr;

  const clang::SourceManager& sourceMgr;
  const clang::LangOptions& langOpts;
  clang::DiagnosticsEngine& diagEngine;
  clang::PrintingPolicy printingPolicy;
};

template <typename BackendType>
class ThuscASTConsumer : public clang::ASTConsumer {
public:
  ThuscASTConsumer(clang::Rewriter& r, llvm::StringRef gpuOutputDirectory, llvm::StringRef gpuOutputDirectoryVirtual)
    : visitor(r)
    , rewriter(r)
    , gpuOutputDir(gpuOutputDirectory)
    , gpuOutputDirVirtual(gpuOutputDirectoryVirtual)
  {}

  virtual void HandleTranslationUnit(clang::ASTContext& context) override {
    thusc::ShaderIR shaderIR;
    visitor.setShaderIR(&shaderIR);

    visitor.TraverseDecl(context.getTranslationUnitDecl());

    BackendType backend(shaderIR, rewriter, gpuOutputDir, gpuOutputDirVirtual);
    backend.Output(rewriter.getSourceMgr().getMainFileID());
  }

private:
  ThuscASTVisitor visitor;
  clang::Rewriter& rewriter;
  llvm::StringRef gpuOutputDir;
  llvm::StringRef gpuOutputDirVirtual;
};

template <typename BackendType>
class ThuscFrontendAction : public clang::ASTFrontendAction {
public:
  ThuscFrontendAction(llvm::raw_ostream& outputStream, llvm::StringRef gpuOutputDirectory, llvm::StringRef gpuOutputDirectoryVirtual)
    : outstream(outputStream), gpuOutputDir(gpuOutputDirectory), gpuOutputDirVirtual(gpuOutputDirectoryVirtual) {}

  void EndSourceFileAction() override {
    clang::SourceManager& sourceMgr = rewriter.getSourceMgr();
    const clang::FileID fileID = sourceMgr.getMainFileID();
    const clang::FileEntry* fileEntry = sourceMgr.getFileEntryForID(sourceMgr.getMainFileID());

    outstream << "// Generated from " << fileEntry->getName() << "\n\n";
    outstream << "#define THUSC_HOST\n";
    outstream << "#include \"" THUSC_H_PATH "\"\n";
    outstream << "#undef THUSC_HOST\n\n";
    rewriter.getEditBuffer(fileID).write(outstream);
  }

  std::unique_ptr<clang::ASTConsumer> CreateASTConsumer(
      clang::CompilerInstance& compilerInstance, llvm::StringRef inFile) override {

    rewriter.setSourceMgr(compilerInstance.getSourceManager(), compilerInstance.getLangOpts());

    return std::make_unique<ThuscASTConsumer<BackendType>>(rewriter, gpuOutputDir, gpuOutputDirVirtual);
  }

private:
  llvm::raw_ostream& outstream;
  llvm::StringRef gpuOutputDir;
  llvm::StringRef gpuOutputDirVirtual;

  clang::Rewriter rewriter;
};

template <typename BackendType>
class ThuscFrontendActionFactory : public clang::tooling::FrontendActionFactory {
public:
  ThuscFrontendActionFactory(llvm::raw_ostream& outputStream, llvm::StringRef gpuOutputDirectory, llvm::StringRef gpuOutputDirectoryVirtual)
    : outstream(outputStream), gpuOutputDir(gpuOutputDirectory), gpuOutputDirVirtual(gpuOutputDirectoryVirtual) {}

  std::unique_ptr<clang::FrontendAction> create() override {
    // TODO Clang tools are often meant to be run on multiple files simultaneously,
    //      so using the same output stream for each instance of the FrontendAction created
    //      is probably a bad idea. However, it's fine for now, since thusc.cpp's main()
    //      guarantees that only one source file will be processed per invocation of main().
    return std::make_unique< ThuscFrontendAction<BackendType> >(outstream, gpuOutputDir, gpuOutputDirVirtual);
  }

private:
  llvm::raw_ostream& outstream;
  llvm::StringRef gpuOutputDir;
  llvm::StringRef gpuOutputDirVirtual;
};

