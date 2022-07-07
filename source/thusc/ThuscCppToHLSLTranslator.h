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
#include "clang/AST/RecursiveASTVisitor.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/SourceManager.h"
#include "clang/Rewrite/Core/Rewriter.h"
ENABLE_WARNINGS_LLVM

#include <stack>

namespace thusc {

class CppToHLSLVisitor : public clang::RecursiveASTVisitor<CppToHLSLVisitor> {
public:
  CppToHLSLVisitor(const llvm::DenseMap<const clang::FunctionDecl*, const thusc::GPUFunction*>& clangFuncToThuscFunc,
                   const ShaderClass* shaderClass,
                   clang::Rewriter& rewriter)
    : clangFuncToThuscFunc(clangFuncToThuscFunc)
    , shaderClass(shaderClass)
    , context(shaderClass->classDecl->getASTContext())
    , rewriter(rewriter)
    , diagEngine(rewriter.getSourceMgr().getDiagnostics())
  {}

  bool VisitStmt(clang::Stmt* stmt);
  bool VisitDeclRefExpr(clang::DeclRefExpr* declRefExpr);
  bool VisitCallExpr(clang::CallExpr* callExpr);
  bool VisitMemberExpr(clang::MemberExpr* memberExpr);

  void pushNestedShaderClass(const ShaderClass* shaderClassParam, llvm::StringRef prefix) {
    shaderClassStack.push_back(std::make_pair(shaderClass, implicitThisOverride));
    shaderClass = shaderClassParam;
    implicitThisOverride = prefix.str();
  }

  void popNestedShaderClass() {
    shaderClass = shaderClassStack.back().first;
    implicitThisOverride = shaderClassStack.back().second;
    shaderClassStack.pop_back();
  }

  void undoRewrites();

private:
  // Helper methods
  void replaceTextWithUndo(llvm::StringRef newText, std::function<clang::SourceRange()> f);
  bool rewriteUniformMemberAccess(const clang::MemberExpr* memberExpr);

  const llvm::DenseMap<const clang::FunctionDecl*, const thusc::GPUFunction*>& clangFuncToThuscFunc;
  const ShaderClass* shaderClass;
  const clang::ASTContext& context;
  clang::Rewriter& rewriter;
  clang::DiagnosticsEngine& diagEngine;

  std::vector<std::pair<const ShaderClass*, std::string>> shaderClassStack;
  std::string implicitThisOverride = "";

  std::stack<std::tuple<std::function<clang::SourceRange()>, unsigned, std::string>> undoStack;
};

}; // namespace thusc
