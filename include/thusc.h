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

#include <cstdint>

#include "RenderGraphResources.h"

__pragma(warning(push, 0))
__pragma(warning(disable : 4582))
__pragma(warning(disable : 4668))
#define HLSLPP_SCALAR
#include "../external/hlslpp/include/hlsl++.h"
__pragma(warning(pop))

#define BRANCH
#define UNROLL


namespace thusc {
  template<typename T1, typename T2>
  T1 max(T1 f1, T2 f2) { return (f2 > f1) ? f2 : f1; }

  template<typename T1, typename T2>
  T1 min(T1 f1, T2 f2) { return (f2 < f1) ? f2 : f1; }

  template<typename T>
  T rcp(T f) { return hlslpp::rcp(f); }

  template<int X, int Y>
  void sincos(const hlslpp::float1& f, hlslpp::swizzle1<X>& s, hlslpp::swizzle1<Y>& c) {
    hlslpp::float1 sinFloat = s;
    hlslpp::float1 cosFloat = c;
    hlslpp::sincos(f, sinFloat, cosFloat);
    s = sinFloat;
    c = cosFloat;
  }

  template<typename T>
  void sincos(const T& f, T& s, T& c) { hlslpp::sincos(f, s, c); }

  class SamplerState {
  public:
    SamplerState operator=(FRHISamplerState* rhs) {
      data = rhs;
      return *this;
    }

    operator FRHISamplerState*() const {
      return data;
    }

  private:
    FRHISamplerState* data;
  };

  template<typename T, typename UE4Type>
  class Texture2DBase {
  public:
    Texture2DBase<T, UE4Type>& operator=(const UE4Type& rhs) {
      data = rhs;
      return *this;
    }

    operator UE4Type() const {
      return data;
    }

    // GPU-only functions
#ifndef THUSC_HOST
    T SampleLevel(SamplerState samplerState, hlslpp::float2 location, float lod, hlslpp::int2 offset = hlslpp::int2(0)) const {
      return T(0);
    }

    T Load(hlslpp::int3 in) const {
      return T(0);
    }
#endif // ndef THUSC_HOST

  private:
    UE4Type data;
  };

  template<typename T>
  class RWTexture2D {
  public:
    RWTexture2D<T>& operator=(const FRDGTextureUAVRef& rhs) {
      data = rhs;
      return *this;
    }

    operator FRDGTextureUAVRef() const {
      return data;
    }

    // GPU-only functions
#ifndef THUSC_HOST
    T operator[](const hlslpp::uint2 pos) const {
      return T();
    }
#endif // ndef THUSC_HOST


  private:
    FRDGTextureUAVRef data;
  };

  template<typename T>
#ifdef THUSC_HOST
  class GlobalUniformBuffer {
#else
  class GlobalUniformBuffer : public T {
#endif // THUSC_HOST
  public:
    GlobalUniformBuffer<T>& operator=(const TUniformBufferRef<T>& rhs) {
      data = rhs;
      return *this;
    }

    operator TUniformBufferRef<T>() const {
      return data;
    }

  private:
    TUniformBufferRef<T> data;
  };
} // namespace thusc

using float2 = hlslpp::float2;
using float3 = hlslpp::float3;
using float4 = hlslpp::float4;
using int1 = hlslpp::int1;
using int2 = hlslpp::int2;
using int3 = hlslpp::int3;
using int4 = hlslpp::int4;
using uint = unsigned int;
using uint2 = hlslpp::uint2;
using uint3 = hlslpp::uint3;
using uint4 = hlslpp::uint4;
using bool2 = hlslpp::uint2;

#define THUSC_FOWRARDING_FUNCTION(functionName) template<typename... Args> static auto  functionName (Args&&... args) -> decltype(thusc:: functionName (Forward<Args>(args)...)) { return  functionName (Forward<Args>(args)...); }

#define HLSLPP_FOWRARDING_FUNCTION(functionName) template<typename... Args> static auto functionName (Args&&... args) -> decltype(hlslpp:: functionName (Forward<Args>(args)...)) { return functionName (Forward<Args>(args)...); }

THUSC_FOWRARDING_FUNCTION(sincos)
THUSC_FOWRARDING_FUNCTION(max)
THUSC_FOWRARDING_FUNCTION(min)
THUSC_FOWRARDING_FUNCTION(rcp)
//HLSLPP_FOWRARDING_FUNCTION(all)
//HLSLPP_FOWRARDING_FUNCTION(any)

//template<typename T1, typename T2>
//using min = thusc::min<T1,T2>;
//
//template<typename T1, typename T2>
//using max = thusc::max<T1,T2>;

using SamplerState = thusc::SamplerState;

using Texture2D = thusc::Texture2DBase<float4, FRDGTextureRef>;

template<typename T>
using Texture2DSRV = thusc::Texture2DBase<T, FRDGTextureSRVRef>;

template<typename T>
using RWTexture2D = thusc::RWTexture2D<T>;

template<typename T>
using GlobalUniformBuffer = thusc::GlobalUniformBuffer<T>;

