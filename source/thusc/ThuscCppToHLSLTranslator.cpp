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

#include "ThuscCppToHLSLTranslator.h"

#include "ClangHelperFunctions.h"

using namespace clang;
using namespace llvm;
using namespace thusc;

// CppToHLSLVisitor methods

void CppToHLSLVisitor::replaceTextWithUndo(StringRef newText, std::function<SourceRange()> f) {
  std::string origText = rewriter.getRewrittenText(f());
  undoStack.push(std::make_tuple(f, unsigned(newText.size()), origText));

  rewriter.ReplaceText(f(), newText);
}

void CppToHLSLVisitor::undoRewrites() {
  while (!undoStack.empty()) {
    auto elem = undoStack.top();
    undoStack.pop();

    rewriter.ReplaceText(std::get<0>(elem)().getBegin(), std::get<1>(elem), std::get<2>(elem));
  }
}

bool CppToHLSLVisitor::VisitStmt(Stmt* stmt) {
  //llvm::errs() << "============================================\n";
  //stmt->dump();
  //stmt->dumpPretty(context);

  return true;
}

bool CppToHLSLVisitor::VisitDeclRefExpr(DeclRefExpr* declRefExpr) {
  // Replace references to enum values (Foo::Bar) with the underscore version used in the output HLSL (Foo_Bar)
  //   Currently only operates on enums that were used as permutation fields
  // TODO Make this work for non-permutation enums
  if (const auto* enumConstantDecl = dyn_cast<EnumConstantDecl>(declRefExpr->getDecl())) {
    const auto* enumDecl = cast<EnumDecl>(enumConstantDecl->getDeclContext());

    for (const auto& perm : shaderClass->permutationFields) {
      if (const auto* enumType = dyn_cast<EnumType>(perm.fieldDecl->getType())) {
        if (enumType->getDecl() == enumDecl) {
          replaceTextWithUndo(perm.type + "_" + declRefExpr->getDecl()->getNameAsString(), [declRefExpr]() { return declRefExpr->getSourceRange(); });
          return true;
        }
      }
    }

    diagError(diagEngine, declRefExpr->getLocation(), "References to enum values must correspond to an enum used as a permutation parameter");
    return true;
  }

  return true;
}

bool CppToHLSLVisitor::VisitCallExpr(CallExpr* callExpr) {
  const FunctionDecl* funcDecl = callExpr->getDirectCallee();
  if (funcDecl == nullptr)
    return true;

  // TODO Rather than just searching for calls to known GPU functions,
  //      we should also search for calls to builtin functions.
  //      Then, if we find a CallExpr that calls a non-GPU and non-builtin
  //      function, we can report an error.

  const auto funcMapEntry = clangFuncToThuscFunc.find(funcDecl);
  if (funcMapEntry != clangFuncToThuscFunc.end()) {
    const GPUFunction* gpuFunc = funcMapEntry->second;

    std::string argString = "(";
    if (callExpr->getNumArgs() == 0) {
      argString.append(")");
    }
    else {
      const SourceRange argRange(callExpr->getArg(0)->getBeginLoc(), callExpr->getRParenLoc());
      argString.append(rewriter.getRewrittenText(argRange));
    }

    std::string newText = (gpuFunc->getMangledName() + argString).str();

    // Replace function call with a call to the name-mangled version
    if (const auto* cxxMemberCallExpr = dyn_cast<CXXMemberCallExpr>(callExpr)) {
      replaceTextWithUndo(newText, [cxxMemberCallExpr]() { return SourceRange(cxxMemberCallExpr->getExprLoc(), cxxMemberCallExpr->getRParenLoc()); });
    }
    else {
      replaceTextWithUndo(newText, [callExpr]() { return callExpr->getSourceRange(); });
    }

  }

  return true;
}

bool CppToHLSLVisitor::VisitMemberExpr(MemberExpr* memberExpr) {
  bool shouldContinue = rewriteUniformMemberAccess(memberExpr);

  const auto* base = memberExpr->getBase()->IgnoreImpCasts();

  // For implicit this expressions, replace with the implicitThisOverride prior to the access
  if (const auto* thisExpr = dyn_cast<CXXThisExpr>(base)) {
    bool isPermutationFieldOrOtherGPUDecl =
      shaderClass->isPermutationField(memberExpr->getMemberDecl()) || shaderClass->isOtherGPUDecl(memberExpr->getMemberDecl());

    if (!(isPermutationFieldOrOtherGPUDecl || implicitThisOverride.empty())) {
      std::string memberExprString = rewriter.getRewrittenText(memberExpr->getSourceRange());
      replaceTextWithUndo(implicitThisOverride + memberExprString, [memberExpr]() { return memberExpr->getSourceRange(); });
    }
  }
  // For accesses to member of ShaderClass permutation fields, change -> to _
  else if (const auto* baseMemberExpr = dyn_cast<MemberExpr>(base)) {
    if (shaderClass->isPermutationShaderClassField(baseMemberExpr->getMemberDecl())) {
      replaceTextWithUndo("_", [memberExpr]() { return SourceRange(memberExpr->getOperatorLoc(), memberExpr->getOperatorLoc()); });
    }
  }

  return shouldContinue;
}


// CppToHLSLVisitor helper methods

// In UE4, struct uniform variables are flattened into individual uniforms.
// So, if the MemberExpr has a base (somewhere up the chain) that is a
// uniform, replace the MemberExpr with an access to the flattened uniform.
bool CppToHLSLVisitor::rewriteUniformMemberAccess(const MemberExpr* memberExpr) {
  const Expr* base = memberExpr->getBase()->IgnoreCasts();

  bool foundUniformStruct = false;
  while (const auto* baseMemberExpr = dyn_cast<MemberExpr>(base)) {
    if (baseMemberExpr->getBase()->isImplicitCXXThis()) {
      if (const auto* fieldDecl = dyn_cast<FieldDecl>(baseMemberExpr->getMemberDecl())) {
        foundUniformStruct = shaderClass->isUniformField(fieldDecl, UniformField::Kind::Struct);
      }
      break;
    }

    base = baseMemberExpr->getBase()->IgnoreCasts();
  }

  if (!foundUniformStruct) {
    return true;
  }

  assert(!memberExpr->isArrow() && "Access to uniform parameter using arrow syntax (->) is not supported");

  // Replace the member access dot operator with an underscore
  replaceTextWithUndo("_", [memberExpr]() { return SourceRange(memberExpr->getOperatorLoc(), memberExpr->getOperatorLoc()); });

  return true;
}

