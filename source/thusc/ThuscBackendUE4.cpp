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

#include "ThuscBackendUE4.h"

#include "ClangHelperFunctions.h"
#include "TypeHelpers.h"

DISABLE_WARNINGS_LLVM
#include "clang/AST/PrettyPrinter.h"
ENABLE_WARNINGS_LLVM

using namespace clang;
using namespace llvm;
using namespace thusc;


// Public methods

bool ThuscBackendUE4Host::Output(const FileID currentFileID) {
  // OtherGPUDecls
  for (const auto& gpuDecl : shaderIR.otherGPUDecls) {
    // Only output a GPUDecl if it is defined in the file we're currently processing
    if (sourceMgr.getFileID(gpuDecl.getDecl()->getLocation()) == currentFileID) {
      if (!gpuDecl.isHost()) {
        rewriter.RemoveText(gpuDecl.getDecl()->getSourceRange());
      }
    }
  }

  // GPUFunctions
  for (const auto& gpuFunc : shaderIR.gpuFunctions) {
    // Only output a GPUFunction if it is defined in the file we're currently processing
    if (sourceMgr.getFileID(gpuFunc->getFunctionDecl()->getLocation()) == currentFileID) {
      if (!OutputGPUFunction(gpuFunc.get())) {
        return false;
      }
    }
  }

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

// Uniform fields
inline
void OutputUniformFieldHelper(const StringRef name, const StringRef type, const StringRef macroName, raw_ostream& stream) {
  stream << macroName << "(" << type << ", " << name << ")\n";
}

void ThuscBackendUE4Host::OutputUniformFieldStruct(const UniformField* uniform, raw_ostream& stream) const {
  assert(uniform->fieldDecl->getType()->isStructureOrClassType() && "Struct uniform field is not a struct or class");

  const CXXRecordDecl* classDecl = uniform->fieldDecl->getType()->getAsCXXRecordDecl();

  // Check for a declaration of the FTypeInfo struct with in the classDecl.
  bool foundFTypeInfo = false;
  for (const auto innerDecl : classDecl->decls()) {
    if (const NamedDecl* innerNamedDecl = dyn_cast<NamedDecl>(innerDecl)) {
      if (innerNamedDecl->getNameAsString() == "FTypeInfo") {
        foundFTypeInfo = true;
        break;
      }
    }
  }

  // If found, then this type was created using the UE4 BEGIN_SHADER_PARAMETER_STRUCT() macro,
  // so no additional processing is required. Else, we need to do some extra work.
  if (foundFTypeInfo) {
    OutputUniformFieldHelper(uniform->name, uniform->type, "SHADER_PARAMETER_STRUCT", stream);
  }
  else {
    // TODO We should eventually handle using struct types that were not declared with the UE4 macro
    diagError(diagEngine, uniform->fieldDecl->getLocation(),
      "When using a struct type uniform parameter, the struct type must be delcared using the UE4 BEGIN_SHADER_PARAMETER_STRUCT() macro");
  }
}

void ThuscBackendUE4Host::OutputUniformField(const UniformField* uniform, raw_ostream& stream) const {
  const UniformField::Kind kind = uniform->getKind();

  if (kind == UniformField::Kind::Sampler) {
    OutputUniformFieldHelper(uniform->name, uniform->type, "SHADER_PARAMETER_SAMPLER", stream);
  }
  else if (kind == UniformField::Kind::Texture) {
    OutputUniformFieldHelper(uniform->name, uniform->type, "SHADER_PARAMETER_RDG_TEXTURE", stream);
  }
  else if (kind == UniformField::Kind::TextureSRV) {
    OutputUniformFieldHelper(uniform->name, getGpuTypeName(uniform->fieldDecl->getType(), uniform->fieldDecl->getASTContext()), "SHADER_PARAMETER_RDG_TEXTURE_SRV", stream);
  }
  else if (kind == UniformField::Kind::TextureUAV) {
    OutputUniformFieldHelper(uniform->name, uniform->type, "SHADER_PARAMETER_RDG_TEXTURE_UAV", stream);
  }
  else if (kind == UniformField::Kind::Primitive) {
    OutputUniformFieldHelper(uniform->name, uniform->type, "SHADER_PARAMETER", stream);
  }
  else if (kind == UniformField::Kind::Array) {
    assert(isa<ConstantArrayType>(uniform->fieldDecl->getType()) && "Array uniform field kind specified, but fieldDecl type is not a ConstantArrayType");
    const ConstantArrayType* arrType = cast<ConstantArrayType>(uniform->fieldDecl->getType().getTypePtr());

    stream << "SHADER_PARAMETER_ARRAY(" << uniform->type << ", " << uniform->name << ", " << "[" << arrType->getSize() << "]" << ")\n";
  }
  else if (kind == UniformField::Kind::GlobalUniformBuffer) {
    OutputUniformFieldHelper(uniform->name, uniform->type, "SHADER_PARAMETER_STRUCT_REF", stream);
  }
  else if (kind == UniformField::Kind::Struct) {
    OutputUniformFieldStruct(uniform, stream);
  }
  else {
    diagError(diagEngine, uniform->fieldDecl->getLocation(), "Uniform parameter type unsupported in UE4 backend");
  }
}

void ThuscBackendUE4Host::OutputUniformFields(const ShaderClass* shaderClass, const ImplClass* implClass, raw_ostream& stream) const {
  // TODO Handle case where a ShaderClass (and its baseShaderClass if applicable) has no uniform fields

  // FParameters struct declaration
  stream << "    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )\n";

  for (const auto& s : implClass->getSelections()) {
    stream << "      " << "SHADER_PARAMETER_STRUCT(" << s.shaderClass->getName() << "::" << s.implClass->getName() << "::FParameters, " << s.fieldName << ")\n";
  }

  for (const auto& uniform : shaderClass->uniformFields) {
    stream << "      ";
    OutputUniformField(&uniform, stream);
  }
  stream << "    END_SHADER_PARAMETER_STRUCT()\n";
  stream << "\n";

  // getPassParameters() function
  stream << "    static void getPassParameters(const " << shaderClass->name << "& inst, FParameters* PassParameters) {\n";

  for (const auto& s : implClass->getSelections()) {
    stream << "      " << s.shaderClass->getName() << "::" << s.implClass->getName() << "::getPassParameters(*static_cast<" << s.shaderClass->getName() << "*>(inst." << s.fieldName << "), &PassParameters->" << s.fieldName << ");\n";
  }

  for (const auto& uniform : shaderClass->uniformFields) {
    if (uniform.getKind() == UniformField::Kind::Array) {
      assert(isa<ConstantArrayType>(uniform.fieldDecl->getType()) && "Array uniform field kind specified, but fieldDecl type is not a ConstantArrayType");
      const ConstantArrayType* arrType = cast<ConstantArrayType>(uniform.fieldDecl->getType().getTypePtr());

      stream << "      for (int i = 0; i < " << arrType->getSize() << "; ++i) {\n";
      stream << "        PassParameters->" << uniform.name << "[i] = inst." << uniform.name << "[i];\n";
      stream << "      }\n";
    }
    else {
      stream << "      PassParameters->" << uniform.name << " = inst." << uniform.name << ";\n";
    }
  }
  stream << "    }\n";
  stream << "\n";
}

// Permutation fields

// Error-checking code (if applicable based on kind)
void ThuscBackendUE4Host::OutputPermutationErrorChecking(const PermutationField& perm, raw_ostream& stream) const {
  if (perm.kind == PermutationField::Kind::Enum) {
    // Do nothing
  }
  else if (perm.kind == PermutationField::Kind::Bool) {
    // Do nothing
  }
  else if (perm.kind == PermutationField::Kind::Int) {
    stream << "      ";
    stream << "checkf(";

    int value;
    // Using 10 as the Radix to indicate that the integers should be specified in decimal form
    if (StringRef(perm.valueOptions.front()).trim().getAsInteger(10, value)) {
      diagError(diagEngine, perm.fieldDecl->getLocation(), "Unable to convert string to int for permutation Int");
      return;
    }

    for (int i = 0; i < value; ++i) {
      if (i != 0)
        stream << " || ";
      stream << "inst." << perm.name << " == " << i;
    }
    stream << ", TEXT(\"Invalid value %i assigned to permutation parameter " << perm.name << ".\"), " << "inst." << perm.name << ");\n";
  }
  else if (perm.kind == PermutationField::Kind::SparseInt) {
    stream << "      ";
    stream << "checkf(";
    for (auto ii = perm.valueOptions.begin(), ie = perm.valueOptions.end(); ii != ie; ++ii) {
      if (ii != perm.valueOptions.begin())
        stream << " || ";
      stream << "inst." << perm.name << " == " << *ii;
    }
    stream << ", TEXT(\"Invalid value %i assigned to permutation parameter " << perm.name << ".\"), " << "inst." << perm.name << ");\n";
  }
  else {
    diagError(diagEngine, perm.fieldDecl->getLocation(), "Permutation type unsupported in UE4 backend (for error checking)");
  }
}

void ThuscBackendUE4Host::OutputPermutationFields(const ShaderClass* shaderClass, const ImplClass* implClass, raw_ostream& stream) const {
  // Permutation parameter declarations
  for (const auto& perm : shaderClass->permutationFields) {
    std::string macroName;

    if (perm.kind == PermutationField::Kind::Enum)
      macroName = "SHADER_PERMUTATION_ENUM_CLASS";
    else if (perm.kind == PermutationField::Kind::Bool)
      macroName = "SHADER_PERMUTATION_BOOL";
    else if (perm.kind == PermutationField::Kind::Int)
      macroName = "SHADER_PERMUTATION_INT";
    else if (perm.kind == PermutationField::Kind::SparseInt)
      macroName = "SHADER_PERMUTATION_SPARSE_INT";
    else
      diagError(diagEngine, perm.fieldDecl->getLocation(), "Permutation type unsupported in UE4 backend");

    stream << "    ";
    stream << "class " << permutationClassPrefix << perm.name << " : " << macroName << "(\"" << perm.name << "\"";

    for (const auto& option : perm.valueOptions) {
      stream << ", " << option;
    }
    stream << ");\n";
  }

  // FPermutationDomain declaration
  stream << "    using FPermutationDomain = TShaderPermutationDomain<";

  for (auto ii = implClass->getSelections().begin(), ie = implClass->getSelections().end(); ii != ie; ++ii) {
    if (ii != implClass->getSelections().begin()) {
      stream << ", ";
    }
    stream << ii->shaderClass->getName() << "::" << ii->implClass->getName() << "::FPermutationDomain";
  }

  for (auto ii = shaderClass->permutationFields.begin(), ie = shaderClass->permutationFields.end(); ii != ie; ++ii) {
    if (ii != shaderClass->permutationFields.begin() || implClass->getSelections().size() > 0) {
      stream << ", ";
    }
    stream << permutationClassPrefix << ii->name;
  }
  stream << ">;\n";
  stream << "\n";

  // getPermutationVector() function
  stream << "    static FPermutationDomain getPermutationVector(const " << shaderClass->name << "& inst) {\n";
  stream << "      FPermutationDomain PermutationVector;\n";
  stream << "\n";

  for (const auto& s : implClass->getSelections()) {
    stream << "      ";
    stream << "PermutationVector.Set<" << s.shaderClass->getName() << "::" << s.implClass->getName() << "::FPermutationDomain" << ">("
      << s.shaderClass->getName() << "::" << s.implClass->getName() << "::getPermutationVector(*static_cast<" << s.shaderClass->getName() << "*>(inst." << s.fieldName << ")));\n";
  }

  for (const auto& perm : shaderClass->permutationFields) {
    OutputPermutationErrorChecking(perm, stream);
    stream << "      ";
    stream << "PermutationVector.Set<" << permutationClassPrefix << perm.name << ">(" << "inst." << perm.name << ");\n";
  }
  stream << "\n";

  stream << "      return PermutationVector;\n";
  stream << "    }\n";
  stream << "\n";
}


// Entry functions

// TODO actually do stuff with compute shader function
void ThuscBackendUE4Host::OutputComputeShaderEntry(const ComputeEntryPoint* entryPoint, const ShaderClass* shaderClass, raw_ostream& stream) const {
  stream << "void AddComputePass(FRDGBuilder& GraphBuilder, FRDGEventName&& PassName, const FGlobalShaderMap* ShaderMap, FIntVector GroupCount)\n";
  stream << "{\n";
  stream << "  unsigned int implClassID = " << thuscgetImplClassIDFunctionName << "();\n";
  stream << "\n";

  for (unsigned int i = 0; i < shaderClass->getImplClasses().size(); ++i) {
    assert(i == shaderClass->getImplClasses()[i]->getID() && "Invalid ImplClass ID");
    const auto* implClass = shaderClass->getImplClasses()[i].get();
    const auto implClassName = implClass->getName();

    if (i == 0) {
      stream << "  if ";
    }
    else {
      stream << "  else if ";
    }
    stream << "(" << "implClassID == " << implClass->getID() << ")\n";
    stream << "  {\n";

    // Permutation parameters
    stream << "    // Permutation parameters\n";
    stream << "    " << implClassName << "::FPermutationDomain PermutationVector = " << implClassName << "::getPermutationVector(*this);\n";
    stream << "\n";

    // Uniform parameters
    stream << "    // Uniform parameters\n";
    stream << "    " << implClassName << "::FParameters* PassParameters = GraphBuilder.AllocParameters<" << implClassName << "::FParameters>();\n";
    stream << "    " << implClassName << "::getPassParameters(*this, PassParameters);\n";
    stream << "\n";

    // Add pass
    stream << "    // Add pass to render graph\n";
    stream << "    TShaderMapRef<" + implClassName + "> ShaderRef" << "(ShaderMap, PermutationVector);\n";
    stream << "    FComputeShaderUtils::AddPass(GraphBuilder, Forward<FRDGEventName>(PassName), ShaderRef, PassParameters, GroupCount);\n";
    stream << "  }\n";
  }

  stream << "  else\n";
  stream << "  {\n";
  stream << "    " << "checkf(false, TEXT(\"Unsupported " << thuscgetImplClassIDFunctionName << "() for " << shaderClass->name << ". Something probably went wrong with thusc code generation.\"));\n";
  stream << "  }\n";

  stream << "}\n";
}


bool ThuscBackendUE4Host::OutputImplClass(const ShaderClass* shaderClass, const ImplClass* implClass, raw_ostream& stream) const {
  std::string baseClassName;
  if (shaderClass->classDecl->getNumBases() == 0 || shaderClass->baseShaderClass) {
    baseClassName = "FGlobalShader";
  }
  else if (shaderClass->classDecl->getNumBases() == 1) {
    // TODO Might want to ensure that the base class is derived from FShader
    baseClassName = shaderClass->classDecl->bases_begin()->getType().getAsString(langOpts);
  }
  else {
    diagError(diagEngine, shaderClass->classDecl->getLocation(), "When using the UE4 backend, thusc shaders must have zero or one base class");
    return false;
  }

  llvm::StringRef implClassName = implClass->getName();

  // Begin UE4 shader C++ shader class
  stream << "  class " << implClassName << " : public " << baseClassName << "\n";
  stream << "  {\n";
  stream << "  public:\n";
  stream << "    DECLARE_GLOBAL_SHADER(" << implClassName << ");\n";
  stream << "    SHADER_USE_PARAMETER_STRUCT(" << implClassName << ", " << baseClassName << ");\n";
  stream << "\n";

  // Uniform parameters
  OutputUniformFields(shaderClass, implClass, stream);

  // TODO Hardcode begin
  // (But this might be fine to leave as-is)
  stream << "    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)\n";
  stream << "    {\n";
  stream << "      FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);\n";
  stream << "      OutEnvironment.SetDefine(TEXT(\"" << implClassName << "\"), 1);\n";
  stream << "    }\n";
  stream << "\n";
  // TODO Hardcode end

  // If ShaderClass has ShouldCompilePermutation() method, output the Impl method that will forward to it
  if (shaderClass->hasShouldCompilePermutation) {
    stream << "    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)\n";
    stream << "    {\n";
    stream << "      FPermutationDomain PermutationVector(Parameters.PermutationId);\n";
    stream << "\n";
    stream << "      " << shaderClass->name << " inst;\n";
    stream << "\n";
    for (const auto& s : implClass->getSelections()) {
      stream << "      " << s.shaderClass->getName() << " " << s.fieldName << ";\n";
      stream << "      " << "inst." << s.fieldName << " = " << "&" << s.fieldName << ";\n";
    }
    stream << "\n";
    for (const auto& perm : shaderClass->permutationFields) {
      stream << "      " << "inst." << perm.name << " = " << "PermutationVector.Get<" << permutationClassPrefix << perm.name << ">();\n";
    }
    stream << "\n";
    stream << "      return " << shaderClass->name << "::ShouldCompilePermutation(inst, Parameters);\n";
    stream << "    }\n";
    stream << "\n";
  }

  // Permutation parameters
  OutputPermutationFields(shaderClass, implClass, stream);

  // End UE4 shader C++ shader class
  stream << "  };\n";
  stream << "\n";

  return true;
}

bool ThuscBackendUE4Host::OutputShaderClass(const ShaderClass* shaderClass) {
  std::string stringForOStream;
  raw_string_ostream stream(stringForOStream);

  stream << "public:\n";
  for (const auto& implClass : shaderClass->getImplClasses()) {
    if (!OutputImplClass(shaderClass, implClass.get(), stream))
      return false;
  }

  // getTypeID functions
  stream << "  virtual unsigned int " << thuscTypeIDPrefix << shaderClass->name << "() const { return " << shaderClass->getTypeIDRelativeTo(shaderClass) << "; }\n";

  const auto* baseShaderClass = shaderClass->baseShaderClass;
  while (baseShaderClass) {
    stream << "  virtual unsigned int " << thuscTypeIDPrefix << baseShaderClass->name << "() const override { return " << shaderClass->getTypeIDRelativeTo(baseShaderClass) << "; }\n";
    baseShaderClass = baseShaderClass->baseShaderClass;
  }
  stream << "\n";

  // getImplClassID function
  stream << "  virtual unsigned int " << thuscgetImplClassIDFunctionName << "() const\n";
  stream << "  {\n";
  if (shaderClass->getImplClasses().size() == 1) {
    assert(shaderClass->permutationShaderClasses.size() == 0 && "ShaderClass has ShaderClass permutation fields, but only one ImplClass was generated");
    stream << "    return 0;\n";
  }
  else {
    for (const auto& field : shaderClass->permutationShaderClasses) {
      stream << "    " << "unsigned int " << field.getFieldName() << "_shaderClassID = "
        << field.getFieldName() << "->" << thuscTypeIDPrefix << field.getShaderClassType()->name << "()" << ";\n";
      stream << "    " << "unsigned int " << field.getFieldName() << "_implClassID = "
        << field.getFieldName() << "->" << thuscgetImplClassIDFunctionName << "()" << ";\n";
    }
    stream << "\n";

    for (unsigned int i = 0; i < shaderClass->getImplClasses().size(); ++i) {
      if (i == 0) {
        stream << "    " << "if ( ";
      }
      else {
        stream << "    " << "else if ( ";
      }

      const auto* implClass = shaderClass->getImplClasses()[i].get();
      for (auto s = implClass->getSelections().begin(), sEnd = implClass->getSelections().end(); s != sEnd; ++s) {
        if (s != implClass->getSelections().begin()) {
          stream << " && ";
        }
        stream << "(" << s->fieldName << "_shaderClassID == " << s->typeIDRelative << " && " << s->fieldName << "_implClassID == " << s->implClass->getID() << ")";
      }
      stream << " ) {\n";
      stream << "      " << "return " << i << "; // " << implClass->getName() << "\n";
      stream << "    " << "}\n";
    }
    stream << "    " << "else {\n";
    stream << "      " << "checkf(false, TEXT(\"Thusc_getImplClassID() found an invalid configuration. Something probably went wrong with thusc code generation.\"));\n";
    stream << "      " << "return 0;\n";
    stream << "    " << "}\n";
  }
  stream << "  }\n";
  stream << "\n";

  // Other GPU Decls
  for (const auto& gpuDecl : shaderClass->otherGPUDecls) {
    if (!gpuDecl.isHost()) {
      rewriter.RemoveText(gpuDecl.getDecl()->getSourceRange());
    }
  }

  // GPU functions
  for (const auto& gpuFunc : shaderClass->getGPUMethods()) {
    if (!OutputGPUFunction(gpuFunc.get())) {
      return false;
    }
  }

  // Shader entry functions
  std::string ueShaderType;
  for (const auto& entryPoint : shaderClass->entryPoints) {
    rewriter.RemoveText(entryPoint->getFunctionDecl()->getSourceRange());

    if (const auto* computeEntryPoint = dyn_cast<const ComputeEntryPoint>(entryPoint.get())) {
      ueShaderType = "SF_Compute";
      OutputComputeShaderEntry(computeEntryPoint, shaderClass, stream);
    }
    else {
      diagError(diagEngine, entryPoint->getFunctionDecl()->getLocation(), "Entry point type unsupported in UE4 backend");
    }
  }

  // Output the changes to the rewriter
  rewriter.InsertText(shaderClass->classDecl->getEndLoc().getLocWithOffset(-1), "\n\n" + stream.str(), true, true);

  if (shaderClass->entryPoints.size() == 0) {
    // Do nothing
  }
  else if (shaderClass->entryPoints.size() == 1) {
    const std::string shaderName = shaderClass->name;
    const std::string gpuOutputFilename = gpuOutputDir + "/" + shaderName + ".usf";

    std::string finalLine = "";
    for (const auto& implClass : shaderClass->getImplClasses()) {
      finalLine += ("IMPLEMENT_GLOBAL_SHADER(" + shaderName + "::" + implClass->getName() + ", "
        + "\"" + gpuOutputFilename + "\"" + ", \"" + shaderClass->entryPoints[0]->getMangledName() + "\", " + ueShaderType + ");\n").str();
    }
    rewriter.InsertText(shaderClass->classDecl->getEndLoc().getLocWithOffset(2), "\n\n" + finalLine, true, true);
  }
  else {
    diagError(diagEngine, shaderClass->classDecl->getLocation(), "Shader classes must have zero or one entry point. Shader classes with two or more entry points are currently unimplemented.");
    return false;
  }

  return true;
}

bool ThuscBackendUE4Host::OutputGPUFunction(const GPUFunction* gpuFunc) {
  if (!gpuFunc->isHostCallable()) {
    rewriter.RemoveText(gpuFunc->getFunctionDecl()->getSourceRange());
  }

  return true;
}

