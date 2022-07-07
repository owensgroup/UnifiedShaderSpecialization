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

#include <vector>

DISABLE_WARNINGS_LLVM
#include "clang/AST/Decl.h"
#include "clang/AST/DeclCXX.h"
#include "llvm/Support/Casting.h"
ENABLE_WARNINGS_LLVM

namespace thusc {

// TODO Fix this hack. It's a quick-and-dirty way to get the corresponding command line option to this file
extern bool disableNameMangling;

// Constants
extern const char* shaderImplClassPrefix;

// Forward declaration
class ImplClass;
struct ShaderClass;
struct ShaderIR;


inline
std::string generateMangledName(const clang::NamedDecl* decl) {
  std::string ret = decl->getName().str();

  if (disableNameMangling)
    return ret;

  const clang::DeclContext* context = decl->getDeclContext();

  while (const auto* namedDecl = llvm::dyn_cast<clang::NamedDecl>(context)) {
    ret = (namedDecl->getName() + "_" + ret).str();
    context = namedDecl->getDeclContext();
  }

  ret = "thusc_" + ret;

  return ret;
}

struct UniformField {
  enum class Kind {
    Sampler,
    Texture,
    TextureSRV,
    TextureUAV,
    Primitive,
    Array,
    Struct,
    GlobalUniformBuffer,
    Unknown,
  };

  UniformField(const clang::FieldDecl* decl, llvm::StringRef name, llvm::StringRef type, Kind kind)
    : fieldDecl(decl)
    , name(name)
    , type(type)
    , kind(kind)
  {
    assert(kind != Kind::Unknown && "Creating a UniformField with Unknown kind");
  }

  Kind getKind() const { return kind; }

  const clang::FieldDecl* fieldDecl;
  const std::string name;
  const std::string type;

private:
  Kind kind;
};

class PermutationShaderClass {
public:
  PermutationShaderClass() = delete;

  PermutationShaderClass(const clang::FieldDecl* fieldDecl, llvm::StringRef name,
    const ShaderClass* shaderClassType, const ShaderIR* shaderIR);

  const llvm::StringRef getFieldName() const { return name; }

  const clang::FieldDecl* getFieldDecl() const { return fieldDecl; }

  const ShaderClass* getShaderClassType() const { return shaderClassType; }

  const std::vector<const ShaderClass*>& getValdShaderClasses() const { return validShaderClasses; }

private:
  const clang::FieldDecl* const fieldDecl;
  const std::string name;
  const ShaderClass* shaderClassType;
  std::vector<const ShaderClass*> validShaderClasses;
};

struct PermutationField {
  enum class Kind {
    Enum,
    Bool,
    Int,
    SparseInt,
    Unknown,
  };

  PermutationField() = delete;

  PermutationField(const clang::FieldDecl* decl, llvm::StringRef name, llvm::StringRef type,
                   llvm::ArrayRef<std::string> valueOptions, Kind kind)
    : fieldDecl(decl)
    , name(name)
    , type(type)
    , valueOptions(valueOptions)
    , kind(kind)
  {
    assert(kind != Kind::Unknown && "Creating a PermutationField with Unknown kind");
  }

  Kind getKind() const { return kind; }

  const clang::FieldDecl* const fieldDecl = nullptr;
  const std::string name;
  const std::string type;
  const std::vector<std::string> valueOptions;
  const Kind kind;
};

enum class ShaderParameterSemanticKind {
  // Compute shader semantics
  SV_DispatchThreadID,
  SV_GroupThreadID,
  SV_GroupID,
  SV_GroupIndex,

  None,
  Unknown,
};


class GPUFunctionBase {
public:
  struct Parameter {
    Parameter(llvm::StringRef name, llvm::StringRef type, ShaderParameterSemanticKind kind, bool isInout)
      : name(name)
      , type(type)
      , semantic(kind)
      , isInout(isInout)
    {
      assert(kind != ShaderParameterSemanticKind::Unknown && "Creating a GPU function parameter with Unknown kind");
    }

    const std::string name;
    const std::string type;
    const ShaderParameterSemanticKind semantic;
    const bool isInout;
  };

  const clang::FunctionDecl* getFunctionDecl() const {
    return funcDecl;
  }

  llvm::StringRef getSourceName() const {
    return sourceName;
  }

  llvm::StringRef getMangledName() const {
    return mangledName;
  }

  llvm::StringRef getReturnType() const {
    return returnType;
  }

  const std::vector<Parameter>& getParameters() const {
    return parameters;
  }

protected:
  GPUFunctionBase(const clang::FunctionDecl* funcDecl, const llvm::StringRef name, const llvm::StringRef returnType)
    : funcDecl(funcDecl)
    , sourceName(name)
    , mangledName(generateMangledName(funcDecl))
    , returnType(returnType)
  {}

  void addParameter(llvm::StringRef name, llvm::StringRef type, ShaderParameterSemanticKind semantic, bool isInout = false) {
    parameters.push_back(Parameter(name.str(), type.str(), semantic, isInout));
  }

private:
  const clang::FunctionDecl* const funcDecl;
  const std::string sourceName;
  const std::string mangledName;
  const std::string returnType;
  std::vector<Parameter> parameters;
};


class GPUFunction : public GPUFunctionBase {
public:
  GPUFunction(const clang::FunctionDecl* funcDecl, const llvm::StringRef name, const llvm::StringRef returnType,
              bool hostCallable)
    : GPUFunctionBase(funcDecl, name, returnType)
    , hostCallable(hostCallable)
  {}

  void addParameter(llvm::StringRef name, llvm::StringRef type, bool isInout) {
    GPUFunctionBase::addParameter(name, type, ShaderParameterSemanticKind::None, isInout);
  }

  bool isHostCallable() const { return hostCallable; }

private:
  const bool hostCallable;
};


class EntryPoint : public GPUFunctionBase {
public:
  enum class EntryPointKind {
    Compute,
    Unknown,
  };

  void addParameter(llvm::StringRef name, llvm::StringRef type, ShaderParameterSemanticKind semantic) {
    GPUFunctionBase::addParameter(name, type, semantic);
  }

  EntryPointKind getKind() const { return kind; }

protected:
  EntryPoint(const clang::CXXMethodDecl* const methodDecl, const llvm::StringRef name,
             const llvm::StringRef returnType, const EntryPointKind kind)
    : GPUFunctionBase(methodDecl, name, returnType)
    , kind(kind)
  {
    assert(kind != EntryPointKind::Unknown && "Creating an EntryPoint with Unknown kind");
  }

private:
  const EntryPointKind kind;
};

class ComputeEntryPoint : public EntryPoint {
public:
  ComputeEntryPoint(const clang::CXXMethodDecl* const methodDecl,
                    const llvm::StringRef name, llvm::ArrayRef<llvm::StringRef> threadgroupSize)
    : EntryPoint(methodDecl, name, "void", EntryPoint::EntryPointKind::Compute)
    , threadgroupSize{threadgroupSize[0].str(), threadgroupSize[1].str(), threadgroupSize[2].str()}
  {
    assert(threadgroupSize.size() == 3 && "threadgroupSize must contain exactly 3 elements");
  }

  static bool classof(const EntryPoint* entryPoint) {
    return entryPoint->getKind() == EntryPointKind::Compute;
  }

  const std::string threadgroupSize[3];
};

class OtherGPUDecl {
public:
  OtherGPUDecl(const clang::Decl* decl, bool isHost, bool hasOutputOverride = false, const std::string outputOverride = "")
    : decl(decl)
    , isHostBool(isHost)
    , hasOutputOverrideBool(hasOutputOverride)
    , outputOverride(outputOverride)
  {}

  const clang::Decl* getDecl() const { return decl; }
  bool isHost() const { return isHostBool; }
  bool hasOutputOverride() const { return hasOutputOverrideBool; }
  llvm::StringRef getOutputOverride() const { return outputOverride; }

private:
  const clang::Decl* decl;
  const bool isHostBool;
  const bool hasOutputOverrideBool;
  const std::string outputOverride;
};

class PermutationShaderClassSelection {
public:
  PermutationShaderClassSelection(llvm::StringRef fieldName, const ShaderClass* shaderClass,
    unsigned int typeIDRelative, const ImplClass* implClass)
    : fieldName(fieldName)
    , shaderClass(shaderClass)
    , typeIDRelative(typeIDRelative)
    , implClass(implClass)
  {}

  const std::string fieldName;
  const ShaderClass* const shaderClass;
  const unsigned int typeIDRelative;
  const ImplClass* const implClass;
};

class ImplClass {
  // The code relies on stable pointers to ImplClass instances in various places,
  // so ImplClass objects can only be created as unique_ptrs.
public:
  static std::unique_ptr<ImplClass> create(const ShaderClass* parent, unsigned int id, std::vector<PermutationShaderClassSelection>& selections) {
    return std::unique_ptr<ImplClass>(new ImplClass(parent, id, selections));
  }

  ImplClass(const ImplClass&) = delete;

  llvm::StringRef getName() const { return name; }
  unsigned int getID() const { return id; }

  const std::vector<PermutationShaderClassSelection>& getSelections() const { return selections; }

private:
  ImplClass(const ShaderClass* parent, unsigned int id, const std::vector<PermutationShaderClassSelection>& selections)
    : parent(parent)
    , name(generateImplClassName(parent, selections))
    , id(id)
    , selections(selections)
  {}

  static std::string generateImplClassName(const ShaderClass* parent, const std::vector<PermutationShaderClassSelection>& selections);

  const ShaderClass* const parent;
  const std::string name;
  const unsigned int id;
  const std::vector<PermutationShaderClassSelection> selections;
};

struct ShaderClass {
  // The code relies on stable pointers to ShaderClass instances in various places,
  // so ShaderClass objects can only be created as unique_ptrs.
public:
  ShaderClass(const ShaderClass&) = delete;

  static std::unique_ptr<ShaderClass> create() {
    return std::unique_ptr<ShaderClass>(new ShaderClass);
  }

  void addGPUMethod(std::unique_ptr<GPUFunction> gpuFunc);

  const std::vector<std::unique_ptr<GPUFunction>>& getGPUMethods() const { return gpuMethods; }

  void setBaseShaderClass(const ShaderClass* base) {
    if (base == nullptr)
      return;

    baseShaderClass = base;

    const ShaderClass* nextBase = base;
    while (nextBase) {
      typeIDMap.insert(std::make_pair(nextBase, nextBase->getNextTypeID()));
      nextBase = nextBase->baseShaderClass;
    }

    // TODO This way of handling base ShaderClasses (i.e., manually pulling in its elements)
    //      runs into issues if a subclass shadows a base class declaration or if the subclass
    //      (or its user) needs to call a base class version of a virtual function that has been
    //      overridden in the derived class.

    for (const auto& e : base->uniformFields) { uniformFields.push_back(e); }
    for (const auto& e : base->permutationShaderClasses) { permutationShaderClasses.push_back(e); }
    for (const auto& e : base->permutationFields) { permutationFields.push_back(e); }
    for (const auto& e : base->otherGPUDecls) { otherGPUDecls.push_back(e); }
    for (const auto& e : base->gpuMethods) { gpuMethods.push_back(std::make_unique<GPUFunction>(*e)); }

    // TODO Handle virtual functions appropriately

    // TODO Maybe pull in from base:
    //      - entryPoints
    //      - hlslPrecode / hlslPostcode
  }

  llvm::StringRef getName() const { return name; }

  const clang::CXXRecordDecl* classDecl = nullptr;

  std::string name;
  const ShaderClass* baseShaderClass = nullptr;
  bool hasShouldCompilePermutation = false;

  std::vector<UniformField> uniformFields;
  std::vector<PermutationShaderClass> permutationShaderClasses;
  std::vector<PermutationField> permutationFields;
  std::vector<OtherGPUDecl> otherGPUDecls;
  std::vector<std::unique_ptr<EntryPoint>> entryPoints;

  std::string hlslPrecode;
  std::string hlslPostcode;

  bool isIntPermutationField(llvm::StringRef fieldName) const {
    for (const auto& p : permutationFields) {
      if (fieldName == p.name && p.fieldDecl->getType()->isIntegralType(p.fieldDecl->getASTContext()))
        return true;
    }

    return false;
  }

  bool isUniformField(const clang::FieldDecl* fieldDecl, UniformField::Kind kind = UniformField::Kind::Unknown) const {
    for (const auto& uniform : uniformFields) {
      if (fieldDecl == uniform.fieldDecl && (kind == uniform.getKind() || kind == UniformField::Kind::Unknown)) {
        return true;
      }
    }

    return false;
  }

  bool isPermutationField(const clang::Decl* decl) const {
    for (const auto& permField : permutationFields) {
      if (permField.fieldDecl == decl)
        return true;
    }

    return false;
  }

  bool isPermutationShaderClassField(const clang::Decl* decl) const {
    for (const auto& permShaderClass : permutationShaderClasses) {
      if (permShaderClass.getFieldDecl() == decl) {
        return true;
      }

      if (permShaderClass.getShaderClassType()->isPermutationShaderClassField(decl)) {
        return true;
      }
    }

    return false;
  }

  bool isOtherGPUDecl(const clang::Decl* decl) const {
    for (const auto& gpuDecl : otherGPUDecls) {
      if (gpuDecl.getDecl() == decl)
        return true;
    }

    return false;
  }

  unsigned int getTypeIDRelativeTo(const ShaderClass* base) const {
    const auto mapEntry = typeIDMap.find(base);
    assert(mapEntry != typeIDMap.end() && "Cannot get TypeID relative to a non-base class");

    return mapEntry->second;
  }

  void generateImplClasses() {
    std::vector<PermutationShaderClassSelection> curSelections;
    generateImplClassesRecur(curSelections, 0);
  }

  const std::vector<std::unique_ptr<const ImplClass>>& getImplClasses() const { return implClasses; }

private:
  ShaderClass() {
    typeIDMap.insert(std::make_pair(this, getNextTypeID()));
  }

  unsigned int getNextTypeID() const { return nextTypeID++; }

  void generateImplClassesRecur(std::vector<PermutationShaderClassSelection>& curSelections, unsigned int fieldPos) {
    if (fieldPos >= permutationShaderClasses.size()) {
      implClasses.push_back(ImplClass::create(this, (unsigned int)implClasses.size(), curSelections));
      return;
    }

    const PermutationShaderClass& field = permutationShaderClasses[fieldPos];

    for (const auto* fieldShaderClass : field.getValdShaderClasses()) {
      for (const auto& fieldImplClass : fieldShaderClass->getImplClasses()) {
        curSelections.push_back(PermutationShaderClassSelection(field.getFieldName(), fieldShaderClass,
          fieldShaderClass->getTypeIDRelativeTo(field.getShaderClassType()), fieldImplClass.get()));
        generateImplClassesRecur(curSelections, fieldPos + 1);
        curSelections.pop_back();
      }
    }
  }

  std::vector<std::unique_ptr<GPUFunction>> gpuMethods;

  mutable unsigned int nextTypeID = 0;
  std::vector<std::unique_ptr<const ImplClass>> implClasses;
  llvm::DenseMap<const ShaderClass*, unsigned int> typeIDMap;
};

struct ShaderIR {
  std::vector<OtherGPUDecl> otherGPUDecls;
  std::vector<std::unique_ptr<GPUFunction>> gpuFunctions;

  inline
  void addShaderClass(std::unique_ptr<ShaderClass> shaderClass) {
    clangClassToShaderClass.insert(std::make_pair(shaderClass->classDecl, shaderClass.get()));
    shaderClass->generateImplClasses();
    shaderClasses.push_back(std::move(shaderClass));
  }

  inline
  const std::vector<std::unique_ptr<ShaderClass>>& getShaderClasses() const {
    return shaderClasses;
  }

  inline
  const ShaderClass* getShaderClass(const clang::CXXRecordDecl* classDecl) const {
    if (classDecl == nullptr) return nullptr;

    const auto mapEntry = clangClassToShaderClass.find(classDecl);
    return (mapEntry == clangClassToShaderClass.end()) ? nullptr : mapEntry->second;
  }

private:
  std::vector<std::unique_ptr<ShaderClass>> shaderClasses;
  llvm::DenseMap<const clang::CXXRecordDecl*, const thusc::ShaderClass*> clangClassToShaderClass;
};

}
