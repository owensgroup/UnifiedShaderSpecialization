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

#include "ThuscBackendHLSL.h"

#include "ClangHelperFunctions.h"
#include "TypeHelpers.h"

DISABLE_WARNINGS_LLVM
#include "clang/AST/ASTContext.h"
#include "clang/Basic/FileManager.h"
#include "clang/Basic/SourceManager.h"

#include "llvm/Support/FileSystem.h"
#include "llvm/Support/ToolOutputFile.h"
ENABLE_WARNINGS_LLVM

#include <map>

using namespace clang;
using namespace llvm;
using namespace thusc;

// Helper functions

const std::map<thusc::ShaderParameterSemanticKind, std::string> semanticMap {
  {thusc::ShaderParameterSemanticKind::SV_DispatchThreadID, "SV_DispatchThreadID"},
  {thusc::ShaderParameterSemanticKind::SV_GroupThreadID, "SV_GroupThreadID"},
  {thusc::ShaderParameterSemanticKind::SV_GroupID, "SV_GroupID"},
  {thusc::ShaderParameterSemanticKind::SV_GroupIndex, "SV_GroupIndex"},
};

StringRef getHLSLSemanticSpelling(ShaderParameterSemanticKind semantic) {
  auto spelling = semanticMap.find(semantic);

  assert(spelling != semanticMap.end() && "Unsupported parameter semantic kind");

  return spelling->second;
}


// Helper AST Visitor

class FindUsedGPUFunctionsVisitor : public RecursiveASTVisitor<FindUsedGPUFunctionsVisitor> {
public:
  FindUsedGPUFunctionsVisitor(const std::vector<std::unique_ptr<GPUFunction>>& gpuFunctions) {
    for (const auto& gpuFunc : gpuFunctions) {
      funcUsedMap.insert(std::make_pair(gpuFunc->getFunctionDecl(), false));
    }
  }

  bool isFunctionUsed(const GPUFunction* gpuFunc) const {
    const auto it = funcUsedMap.find(gpuFunc->getFunctionDecl());
    if (it == funcUsedMap.end()) {
      return false;
    }
    else {
      return it->second;
    }
  };

  bool VisitCallExpr(CallExpr* callExpr) {
    const FunctionDecl* calledFuncDecl = callExpr->getDirectCallee();
    auto it = funcUsedMap.find(calledFuncDecl);

    // If the function is not in the map or has already been marked as used, skip it.
    if (it == funcUsedMap.end() || it->second == true) {
      return true;
    }
    // Otherwise, mark the function as used and recurse to find the GPU functions it calls.
    else {
      it->second = true;
      TraverseStmt(calledFuncDecl->getBody());
    }

    return true;
  }

private:
  llvm::DenseMap<const FunctionDecl*, bool> funcUsedMap;
};


// Public methods

bool ThuscBackendHLSL::Output(const FileID currentFileID) {
  // GPUFunctions
  // Nothing to do for GPUFunctions (they will be output per ShaderClass if needed)

  // ShaderClasses
  for (const auto& shaderClass : shaderIR.getShaderClasses()) {
    // Only output a ShaderClass if it is defined in the file we're currently processing
    if (sourceMgr.getFileID(shaderClass->classDecl->getLocation()) == currentFileID) {
      if (!OutputShaderClass(shaderClass.get())) {
        return false;
      }
    }
  }

  return true;
}


// Private methods

bool ThuscBackendHLSL::OutputShaderClass(const ShaderClass* shaderClass) {
  // If the ShaderClass has no entry points, we do not need to output a file for it
  if (shaderClass->entryPoints.size() == 0) {
    return true;
  }

  const std::string shaderName = shaderClass->name;
  const std::string shaderOutputFilename = gpuOutputDir + "\\" + shaderName + ".usf";

  translator = std::make_unique<CppToHLSLVisitor>(clangFuncToThuscFunc, shaderClass, rewriter);

  // Precode - Code that will be inserted prior to the hardcoded input file
  std::string generatedPrecode;
  {
    raw_string_ostream precodeStream(generatedPrecode);

    // Other GPU Decls
    // TODO Should check if each decl is actually need by the ShaderClass,
    //      rather than just always outputting everything
    for (const auto& gpuDecl : shaderIR.otherGPUDecls) {
      if (gpuDecl.hasOutputOverride())
        precodeStream << gpuDecl.getOutputOverride() << "\n";
      else
        precodeStream << rewriter.getRewrittenText(gpuDecl.getDecl()->getSourceRange()) << ";\n";
      precodeStream << "\n";
    }

    // Find all GPU functions called by the GPU methods of this ShaderClass
    FindUsedGPUFunctionsVisitor visitor(shaderIR.gpuFunctions);
    for (const auto& gpuMethod : shaderClass->getGPUMethods()) {
      visitor.TraverseStmt(gpuMethod->getFunctionDecl()->getBody());
    }

    // GPU function
    // Only output a GPU function if it is used in a GPU method of this ShaderClass
    //  or if the --outputAllGPUFuncs command line option was specified
    for (const auto& gpuFunc : shaderIR.gpuFunctions) {
      if (thusc::outputAllGPUFuncs || visitor.isFunctionUsed(gpuFunc.get())) {
        OutputGPUFunction(gpuFunc.get(), precodeStream);
      }
    }

    // Permutation ShaderClass parameters
    OutputPermutationShaderClassFields(shaderClass, precodeStream);

    // Permutation parameters
    OutputPermutationFields(shaderClass, precodeStream);

    // Uniform parameters
    OutputUniformFields(shaderClass, precodeStream);

    // ShaderClass Other GPU Decls
    for (const auto& gpuDecl : shaderClass->otherGPUDecls) {
      if (gpuDecl.hasOutputOverride())
        precodeStream << gpuDecl.getOutputOverride() << "\n";
      else
        precodeStream << rewriter.getRewrittenText(gpuDecl.getDecl()->getSourceRange()) << ";\n";
      precodeStream << "\n";
    }

    // Flush the stream to the string
    precodeStream.str();
  }


  // Postcode - Code that will be inserted after the hardcoded input file
  std::string generatedPostcode;
  {
    raw_string_ostream postcodeStream(generatedPostcode);

    // GPU methods
    for (const auto& gpuMethod : shaderClass->getGPUMethods()) {
      OutputGPUFunction(gpuMethod.get(), postcodeStream);
    }

    // Entry point functions
    for (const auto& entryPoint : shaderClass->entryPoints) {
      if (const auto* computeEntryPoint = dyn_cast<const ComputeEntryPoint>(entryPoint.get())) {
        OutputComputeShaderEntry(computeEntryPoint, postcodeStream);
      }
      else {
        diagError(diagEngine, entryPoint->getFunctionDecl()->getLocation(), "Entry point type unsupported in HLSL backend");
      }
    }

    // Flush the stream to the string
    postcodeStream.str();
  }

  std::error_code ec;
  ToolOutputFile shaderOutputFile(shaderOutputFilename, ec, llvm::sys::fs::OpenFlags::OF_None);
  if (ec) {
    llvm::errs() << "Error: Could not open GPU shader code output file '" << shaderOutputFilename << "': " << ec.message() << "\n";
    return false;
  }

  const SourceManager& sourceMgr = shaderClass->classDecl->getASTContext().getSourceManager();
  StringRef thuscShaderFilename = sourceMgr.getFileEntryForID(sourceMgr.getFileID(shaderClass->classDecl->getLocation()))->getName();

  shaderOutputFile.os() << "// Generated from thusc shader '" << shaderName << "' in file " << thuscShaderFilename << "\n\n";

  shaderOutputFile.os() << "#define HARDCODED_HLSL\n\n";

  if(!shaderClass->hlslPrecode.empty())
    shaderOutputFile.os() << "// Hardcoded HLSL Precode\n" << shaderClass->hlslPrecode << "\n";

  shaderOutputFile.os() << "// Generated Precode\n" << generatedPrecode << "\n";

  if(!shaderClass->hlslPostcode.empty())
    shaderOutputFile.os() << "// Hardcoded HLSL Postcode\n" << shaderClass->hlslPostcode << "\n";

  shaderOutputFile.os() << "// Generated Postcode\n" << generatedPostcode << "\n";

  shaderOutputFile.keep();

  return true;
}

// Permutation ShaderClass fields
void ThuscBackendHLSL::OutputPermutationShaderClassFields(const ShaderClass* shaderClass, raw_ostream& stream) const {
  // For GPU code output, all of the relevant information we need for Permutation ShaderClass Fields is aggregated into
  // the ImplClass IR structure, so no need to use the ShaderClass:permutationShaderClasses field directly.

  for (auto ii = shaderClass->getImplClasses().begin(), ie = shaderClass->getImplClasses().end(); ii != ie; ++ii) {
    const auto* implClass = ii->get();

    if (ii == shaderClass->getImplClasses().begin()) {
      stream << "#if defined(";
    }
    else {
      stream << "#elif defined(";
    }

    stream << implClass->getName() << ")\n";

    for (const auto& s : implClass->getSelections()) {
      OutputSelection(s, stream);
    }
  }

  stream << "#else\n";
  stream << "  #error No ThuscImpl_ class defines were set\n";
  stream << "#endif\n";
  stream << "\n";
}

void ThuscBackendHLSL::OutputSelection(const PermutationShaderClassSelection& selection, raw_ostream& stream, llvm::StringRef prevPrefix) const {
  const auto* shaderClass = selection.shaderClass;

  std::string prefix = (prevPrefix + selection.fieldName + "_").str();

  stream << shaderClass->hlslPrecode << "\n";

  // Permutation parameters
  // TODO Prefix for permutation parameters (maybe)
  //      Currently, the only output for permutation parameters happens for Enum types,
  //      which simply simply outputs a #define for each enum value. Rather than handling
  //      that per permutation parameter, it might be better to just output each enum once.
  OutputPermutationFields(shaderClass, stream);

  // Uniform parameters
  OutputUniformFields(shaderClass, stream, prefix);

  // ShaderClass Other GPU Decls
  for (const auto& gpuDecl : shaderClass->otherGPUDecls) {
    // TODO Prefix for other GPU Decls
    if (gpuDecl.hasOutputOverride())
      stream << gpuDecl.getOutputOverride() << "\n";
    else
      stream << rewriter.getRewrittenText(gpuDecl.getDecl()->getSourceRange()) << ";\n";
    stream << "\n";
  }

  stream << shaderClass->hlslPostcode << "\n";

  for (auto ii = shaderClass->getImplClasses().begin(), ie = shaderClass->getImplClasses().end(); ii != ie; ++ii) {
    const auto* implClass = ii->get();

    for (const auto& s : implClass->getSelections()) {
      OutputSelection(s, stream, prefix);
    }
  }

  // GPU method
  for (const auto& gpuMethod : shaderClass->getGPUMethods()) {
    translator->pushNestedShaderClass(shaderClass, prefix);
    OutputGPUFunction(gpuMethod.get(), stream, prefix);
    translator->popNestedShaderClass();
  }
}

// Permutation fields
void ThuscBackendHLSL::OutputPermutationFields(const ShaderClass* shaderClass, raw_ostream& stream) const {
  for (const auto& perm : shaderClass->permutationFields) {
    // Enum
    if (perm.kind == PermutationField::Kind::Enum) {
      assert(perm.fieldDecl->getType()->isEnumeralType() && "Field is not of type enum");

      const EnumDecl* enumType = cast<EnumType>(perm.fieldDecl->getType())->getDecl();

      // For each value in the enum
      //   Output `#define EnumTypeName_EnumValueName Value`
      for (const auto enumValue : enumType->enumerators()) {
        stream << "#define " << perm.type << "_" << enumValue->getName() << " "
          << enumValue->getInitVal() << "\n";
      }
      stream << "\n";
    }
    // Bool
    else if (perm.kind == PermutationField::Kind::Bool) {
      // Do nothing
    }
    // Int
    else if (perm.kind == PermutationField::Kind::Int) {
      // Do nothing
    }
    // SparseInt
    else if (perm.kind == PermutationField::Kind::SparseInt) {
      // Do nothing
    }
    else {
      diagError(diagEngine, perm.fieldDecl->getLocation(), "Permutation type unsupported in HLSL backend");
    }
  }
}

// Uniform fields
void ThuscBackendHLSL::OutputUniformFields(const ShaderClass* shaderClass, raw_ostream& stream, StringRef prefix) const {
  for (const auto& uniform : shaderClass->uniformFields) {
    OutputUniformField(&uniform, stream, prefix);
  }
}

void ThuscBackendHLSL::OutputUniformFieldSimple(const UniformField* uniform, raw_ostream& stream, const Twine& prefix) const {
  stream << uniform->type << " " << prefix << uniform->name << ";\n";
  stream << "\n";
}

void ThuscBackendHLSL::OutputFlattenedUniformStruct(const CXXRecordDecl* classDecl, raw_ostream& stream, const Twine& prefix) const {
  for (const auto& field : classDecl->fields()) {
    std::string gpuTypeName = getGpuTypeName(field->getType(), field->getASTContext());

    if (isPrimitiveType(gpuTypeName)) {
      stream << gpuTypeName << " " << prefix << field->getName() << ";\n";
    }
    else if (field->getType()->isStructureOrClassType()) {
      const CXXRecordDecl* nestedClassDecl = field->getType()->getAsCXXRecordDecl();
      OutputFlattenedUniformStruct(nestedClassDecl, stream, prefix + field->getName() + "_");
    }
    else {
      diagError(diagEngine, field->getLocation(), "Unsupported field type in struct used as uniform parameter");
    }
  }
}

void ThuscBackendHLSL::OutputUniformFieldStruct(const UniformField* uniform, raw_ostream& stream, const Twine& prefix) const {
  assert(uniform->fieldDecl->getType()->isStructureOrClassType() && "Struct uniform field is not a struct or class");

  const CXXRecordDecl* classDecl = uniform->fieldDecl->getType()->getAsCXXRecordDecl();
  OutputFlattenedUniformStruct(classDecl, stream, prefix + uniform->name + "_");
  stream << "\n";
}

void ThuscBackendHLSL::OutputUniformField(const UniformField* uniform, raw_ostream& stream, const Twine& prefix) const {
  // TODO Rename map
  const UniformField::Kind kind = uniform->getKind();

  if (kind == UniformField::Kind::Sampler || kind == UniformField::Kind::Texture || kind == UniformField::Kind::TextureUAV) {
    OutputUniformFieldSimple(uniform, stream, prefix);
  }
  else if (kind == UniformField::Kind::TextureSRV) {
    stream << getGpuTypeName(uniform->fieldDecl->getType(), uniform->fieldDecl->getASTContext()) << " " << prefix << uniform->name << ";\n";
    stream << "\n";
  }
  else if (kind == UniformField::Kind::Primitive) {
    stream << getGpuTypeName(uniform->fieldDecl->getType(), uniform->fieldDecl->getASTContext()) << " " << prefix << uniform->name << ";\n";
    stream << "\n";
  }
  else if (kind == UniformField::Kind::Array) {
    assert(isa<ConstantArrayType>(uniform->fieldDecl->getType()) && "Array uniform field kind specified, but fieldDecl type is not a ConstantArrayType");
    const ConstantArrayType* arrType = cast<ConstantArrayType>(uniform->fieldDecl->getType().getTypePtr());

    stream << uniform->type << " " << prefix << uniform->name << "[" << arrType->getSize() << "];\n";
    stream << "\n";
  }
  else if (kind == UniformField::Kind::GlobalUniformBuffer) {
    // Do nothing
    // In UE4, globally named shader parameter structs get spliced in to the shader automatically,
    // so no need to output a definition here.
  }
  else if (kind == UniformField::Kind::Struct) {
    OutputUniformFieldStruct(uniform, stream, prefix);
  }
  else {
    diagError(diagEngine, uniform->fieldDecl->getLocation(), "Uniform parameter type unsupported in HLSL backend");
  }
}

void ThuscBackendHLSL::OutputFunctionDeclaration(const GPUFunctionBase* gpuFunc, raw_ostream& stream, bool shouldOutputSemantics, StringRef prefix) const {
  stream << gpuFunc->getReturnType() << " " << prefix << gpuFunc->getMangledName() << "(";

  for (auto ii = gpuFunc->getParameters().begin(), ie = gpuFunc->getParameters().end(); ii != ie; ++ii) {
    if (ii != gpuFunc->getParameters().begin()) {
      stream << ", ";
    }

    if (ii->isInout) {
      stream << "inout ";
    }

    stream << ii->type << " " << ii->name;

    if (shouldOutputSemantics) {
      stream << " : " << getHLSLSemanticSpelling(ii->semantic);
    }
    else {
      assert(ii->semantic == ShaderParameterSemanticKind::None && "GPU function parameter has unused semantic");
    }
  }

  stream << ")\n";
}

void ThuscBackendHLSL::OutputFunctionBody(const FunctionDecl* funcDecl, raw_ostream& stream) const {
  assert(funcDecl->hasBody() && "GPU function does not have a function body");

  // Translate the function body and output it
  translator->TraverseStmt(funcDecl->getBody());
  stream << rewriter.getRewrittenText(funcDecl->getBody()->getSourceRange()) << "\n\n";

  // Undo the changes that the translator made
  // This lets the host backend and other ShaderClasses that output this function use the original function
  translator->undoRewrites();
}

void ThuscBackendHLSL::OutputGPUFunction(const GPUFunction* gpuFunc, raw_ostream& stream, StringRef prefix) const {
  // TODO Rename map
  OutputFunctionDeclaration(gpuFunc, stream, false, prefix);
  OutputFunctionBody(gpuFunc->getFunctionDecl(), stream);
}

// Entry functions
void ThuscBackendHLSL::OutputShaderEntryCommon(const EntryPoint* entryPoint, raw_ostream& stream) const {
  OutputFunctionDeclaration(entryPoint, stream, true);
  OutputFunctionBody(entryPoint->getFunctionDecl(), stream);
}

void ThuscBackendHLSL::OutputComputeShaderEntry(const ComputeEntryPoint* entryPoint, raw_ostream& stream) const {
  // Function annotation
  stream << "[numthreads(" << entryPoint->threadgroupSize[0] << ", " <<
    entryPoint->threadgroupSize[1] << ", " << entryPoint->threadgroupSize[2] << ")]\n";

  OutputShaderEntryCommon(entryPoint, stream);
}

