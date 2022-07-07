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

#define DISABLE_WARNINGS_LLVM \
  __pragma(warning(push, 0))        /* Disable all warnings before including LLVM/Clang headers */ \
  __pragma(warning(disable : 4146)) /* Disable this specific warning too (which is reported as an error) */

#define ENABLE_WARNINGS_LLVM \
  __pragma(warning(pop))            /* Reenable warnings */

#define THUSC_H_PATH "G:\\workspace\\thusc\\include\\thusc.h"

#define THUSC_FOWRARDING_FUNCTION(functionName) "template<typename... Args> static auto " functionName "(Args&&... args) -> decltype(thusc::" functionName "(Forward<Args>(args)...)) { return " functionName "(Forward<Args>(args)...); }"

#define HLSLPP_FOWRARDING_FUNCTION(functionName) "template<typename... Args> static auto " functionName "(Args&&... args) -> decltype(hlslpp::" functionName "(Forward<Args>(args)...)) { return " functionName "(Forward<Args>(args)...); }"

#define THUSC_TYPE_ADAPTERS \
  THUSC_FOWRARDING_FUNCTION("sincos") \
  THUSC_FOWRARDING_FUNCTION("max") \
  THUSC_FOWRARDING_FUNCTION("min") \
  HLSLPP_FOWRARDING_FUNCTION("all") \
  HLSLPP_FOWRARDING_FUNCTION("any") \
  ""

#define THUSC_SHADER_SPELLING "ThuscShader"

#define THUSC_UNIFORM_PREFIX "ThuscUniform-"
#define THUSC_UNIFORM_SPELLING THUSC_UNIFORM_PREFIX

#define THUSC_PERMUTATION_PREFIX "ThuscPermutation-"
#define THUSC_PERMUTATION_ENUM_SPELLING THUSC_PERMUTATION_PREFIX "Enum"
#define THUSC_PERMUTATION_BOOL_SPELLING THUSC_PERMUTATION_PREFIX "Bool"
#define THUSC_PERMUTATION_INT_SPELLING THUSC_PERMUTATION_PREFIX "Int"
#define THUSC_PERMUTATION_SPARSE_INT_SPELLING THUSC_PERMUTATION_PREFIX "SparseInt"
#define THUSC_PERMUTATION_SHADERCLASS_SPELLING THUSC_PERMUTATION_PREFIX "ShaderClass"

#define THUSC_HLSL_SEMANTIC_PREFIX "ThuscHLSLSemantic-"

#define THUSC_HLSL_SEMANTIC_COMPUTESHADER_PREFIX THUSC_HLSL_SEMANTIC_PREFIX "ComputeShader-"
#define THUSC_SV_DISPATCH_THREAD_ID_SPELLING THUSC_HLSL_SEMANTIC_COMPUTESHADER_PREFIX "SV_DispatchThreadID"
#define THUSC_SV_GROUP_THREAD_ID_SPELLING THUSC_HLSL_SEMANTIC_COMPUTESHADER_PREFIX "SV_GroupThreadID"
#define THUSC_SV_GROUP_ID_SPELLING THUSC_HLSL_SEMANTIC_COMPUTESHADER_PREFIX "SV_GroupID"
#define THUSC_SV_GROUP_INDEX_SPELLING THUSC_HLSL_SEMANTIC_COMPUTESHADER_PREFIX "SV_GroupIndex"

#define THUSC_SHADER_ENTRY_PREFIX "ThuscShaderEntry-"
#define THUSC_ENTRY_COMPUTE_SPELLING THUSC_SHADER_ENTRY_PREFIX "Compute"
#define THUSC_ENTRY_PIXEL_SPELLING THUSC_SHADER_ENTRY_PREFIX "Pixel"

#define THUSC_GPU_FN_SPELLING "Thusc-GPU"
#define THUSC_HLSL_PRECODE_SPELLING "Thusc-PrecodeHLSL"
#define THUSC_HLSL_POSTCODE_SPELLING "Thusc-PostcodeHLSL"
#define THUSC_HLSL_INOUT_SPELLING "Thusc-inout"

