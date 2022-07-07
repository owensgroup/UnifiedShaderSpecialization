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

DISABLE_WARNINGS_LLVM
#include "clang/AST/ASTContext.h"
#include "clang/AST/Type.h"
#include "llvm/ADT/StringRef.h"
ENABLE_WARNINGS_LLVM

#include <string>
#include <utility>

inline
bool isPrimitiveType(const llvm::StringRef typeName) {
  const std::string typeNames[]{
    "float", "float2", "float3", "float4",
    "int", "int2", "int3", "int4",
    // TODO add others
  };

  for (const auto& name : typeNames) {
    if (name == typeName)
      return true;
  }

  return false;
}

// Map from UE4 types to HLSL types
inline
std::string getGpuTypeName(const clang::QualType& type, const clang::ASTContext& context) {
  std::pair<std::string, std::string> typeNames[] = {
    std::make_pair("FVector2D", "float2"),
    std::make_pair("FVector4",  "float4"),
    std::make_pair("FIntPoint", "int2"),
    std::make_pair("thusc::Texture2DBase<hlslpp::uint2, FRDGTextureSRV *>", "Texture2D<uint2>"),
    // TODO add others
  };

  std::string typeName = type.getDesugaredType(context).getAsString(context.getLangOpts());

  for (const auto& namePair : typeNames) {
    if (typeName == namePair.first)
      return namePair.second;
  }

  return typeName;
}

