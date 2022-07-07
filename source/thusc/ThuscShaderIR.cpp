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

#include "ThuscShaderIR.h"

using namespace clang;
using namespace llvm;
using namespace thusc;

const char* thusc::shaderImplClassPrefix = "ThuscImpl_";

// PermutationShaderClass
PermutationShaderClass::PermutationShaderClass(const clang::FieldDecl* fieldDecl, llvm::StringRef name,
  const ShaderClass* shaderClassType, const ShaderIR* shaderIR)
  : fieldDecl(fieldDecl)
  , name(name)
  , shaderClassType(shaderClassType)
{
  validShaderClasses.push_back(shaderClassType);

  for (const auto& shaderClass : shaderIR->getShaderClasses()) {
    const ShaderClass* baseShaderClass = shaderClass->baseShaderClass;
    while (baseShaderClass) {
      if (baseShaderClass == shaderClassType) {
        validShaderClasses.push_back(shaderClass.get());
        break;
      }
      baseShaderClass = baseShaderClass->baseShaderClass;
    }
  }
}

// ImplClass
std::string ImplClass::generateImplClassName(const ShaderClass* parent, const std::vector<PermutationShaderClassSelection>& selections) {
  std::string name = (shaderImplClassPrefix + parent->getName()).str();
  for (const auto& s : selections) {
    name += "_" + s.implClass->getName().str();
  }
  return name;
}


// ShaderClass
void ShaderClass::addGPUMethod(std::unique_ptr<GPUFunction> gpuFunc) {
  // If this function overrides a function that's already in the ShaderClass,
  // then remove the overridden functions
  if (const auto* methodDecl = dyn_cast<CXXMethodDecl>(gpuFunc->getFunctionDecl())) {
    for (const auto* overriddenMethod : methodDecl->overridden_methods()) {
      gpuMethods.erase(std::remove_if(gpuMethods.begin(), gpuMethods.end(),
        [overriddenMethod](std::unique_ptr<GPUFunction>& f) {
          return (f->getFunctionDecl() == overriddenMethod);
        }), gpuMethods.end());
    }
  }

  gpuMethods.push_back(std::move(gpuFunc));
}

