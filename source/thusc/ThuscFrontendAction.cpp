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

#include "ThuscFrontendAction.h"

#include "ClangHelperFunctions.h"
#include "TypeHelpers.h"

DISABLE_WARNINGS_LLVM
#include "clang/AST/Attr.h"
#include "clang/AST/Type.h"
#include "clang/Basic/Diagnostic.h"
#include "clang/Lex/Lexer.h"
#include "clang/Rewrite/Core/Rewriter.h"

#include "llvm/Support/raw_ostream.h"
ENABLE_WARNINGS_LLVM

using namespace clang;
using namespace llvm;
using namespace thusc;


// Helper functions

CharSourceRange removeTrailingWhitespace(const CharSourceRange range, const SourceManager& sourceMgr, const LangOptions& langOpts) {
  SourceLocation endLocation = range.getEnd();
  if (range.isTokenRange()) {
    endLocation = Lexer::getLocForEndOfToken(endLocation, 0, sourceMgr, langOpts);
  }

  bool isInvalid = false;
  const char* endBuf = sourceMgr.getCharacterData(endLocation, &isInvalid);

  if (isInvalid) {
    llvm::errs() << "Warning: Invalid character data range. Trailing whitespace may not be removed.\n";
    return range;
  }

  const char* endPos = endBuf;
  while (*endPos == ' ' || *endPos == '\t') {
    ++endPos;
  }

  int offset = (int)(endPos - endBuf);
  return CharSourceRange(SourceRange(range.getBegin(), endLocation.getLocWithOffset(offset)), false);
}

const AnnotateAttr* findAnnotateAttribute(const Decl* decl, const StringRef spelling, bool isFindPrefix = false) {
  for (const auto& attr : decl->attrs()) {
    if (attr->getKind() == attr::Kind::Annotate) {
      AnnotateAttr* annotateAttr = cast<AnnotateAttr>(attr);
      StringRef annotation = annotateAttr->getAnnotation();

      if (isFindPrefix && annotation.startswith(spelling))
        return annotateAttr;
      else if (annotation.equals(spelling))
        return annotateAttr;
    }
  }

  return nullptr;
}

std::vector<const AnnotateAttr*> findAnnotateAttributeMultiple(const Decl* decl, const StringRef spelling, bool isFindPrefix = false) {
  std::vector<const AnnotateAttr*> ret;

  for (const auto& attr : decl->attrs()) {
    if (attr->getKind() == attr::Kind::Annotate) {
      AnnotateAttr* annotateAttr = cast<AnnotateAttr>(attr);
      StringRef annotation = annotateAttr->getAnnotation();

      if (isFindPrefix && annotation.startswith(spelling))
        ret.push_back(annotateAttr);
      else if (annotation.equals(spelling))
        ret.push_back(annotateAttr);
    }
  }

  return ret;
}

// Remove the attribute because the underlying compiler won't understand it.
void removeCXX11AttrMacro(const Attr* attr, Rewriter& rewriter) {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  const LangOptions& langOpts = rewriter.getLangOpts();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  if (!attr->isCXX11Attribute()) {
    diagError(diagEngine, attr->getLocation(), "Shader attributes must use the C++11 attribute syntax [[...]]");
    return;
  }

  CharSourceRange macroCallRange = sourceMgr.getExpansionRange(attr->getRange());
  SourceLocation beginLocation = macroCallRange.getBegin();
  SourceLocation endLocation = macroCallRange.getEnd();

  if (macroCallRange.isTokenRange()) {
    endLocation = Lexer::getLocForEndOfToken(endLocation, 0, sourceMgr, langOpts);
  }

  int beginOffset;
  int endOffset;

  // Find starting bracket position
  {
    bool isInvalid = false;
    const char* beginBuf = sourceMgr.getCharacterData(beginLocation, &isInvalid);
    const char* beginPos = beginBuf;

    if (isInvalid) {
      diagError(diagEngine, attr->getLocation(), "Unable to find starting brackets for attribute. Invalid character data range.");
      return;
    }

    --beginPos;

    // Trim whitespace
    while (*beginPos == ' ' || *beginPos == '\t') {
      --beginPos;
    }

    // Find "[["
    if (*beginPos != '[') {
      diagError(diagEngine, attr->getLocation(), "Unable to find starting brackets for C++11 attribute after trimming whitespace.");
      return;
    }
    --beginPos;
    if (*beginPos != '[') {
      diagError(diagEngine, attr->getLocation(), "Unable to find starting brackets for C++11 attribute after trimming whitespace.");
      return;
    }

    beginOffset = (int)(beginPos - beginBuf);
  }


  // Find ending bracket position
  {
    bool isInvalid = false;
    const char* endBuf = sourceMgr.getCharacterData(endLocation, &isInvalid);
    const char* endPos = endBuf;

    if (isInvalid) {
      diagError(diagEngine, attr->getLocation(), "Unable to find ending brackets for attribute. Invalid character data range.");
      return;
    }

    // Trim whitespace
    while (*endPos == ' ' || *endPos == '\t') {
      ++endPos;
    }

    // Find "]]"
    if (*endPos != ']') {
      diagError(diagEngine, attr->getLocation(), "Unable to find ending brackets for C++11 attribute after trimming whitespace.");
      return;
    }
    ++endPos;
    if (*endPos != ']') {
      diagError(diagEngine, attr->getLocation(), "Unable to find ending brackets for C++11 attribute after trimming whitespace.");
      return;
    }
    ++endPos;

    // Trim whitespace
    while (*endPos == ' ' || *endPos == '\t') {
      ++endPos;
    }

    endOffset = (int)(endPos - endBuf);
  }

  CharSourceRange removalRange(SourceRange(beginLocation.getLocWithOffset(beginOffset),
    endLocation.getLocWithOffset(endOffset)), false);

  rewriter.RemoveText(removalRange);
}

template<unsigned int N>
bool isTypeHelper(const StringRef typeName, const std::string (& typeNames)[N]) {
  for (const auto& name : typeNames) {
    if (name == typeName)
      return true;
  }

  return false;
}

bool isSamplerType(const StringRef typeName) {
  const std::string typeNames[] = {
    "SamplerState",
  };

  return isTypeHelper(typeName, typeNames);
}

bool isTextureType(const StringRef typeName) {
  const std::string typeNames[] = {
    "Texture2D",
  };

  return isTypeHelper(typeName, typeNames);
}

bool isTextureSRVType(const StringRef typeName) {
  const std::string typeNames[] = {
    "Texture2DSRV<uint2>",
  };

  return isTypeHelper(typeName, typeNames);
}

bool isTextureUAVType(const StringRef typeName) {
  const std::string typeNames[] = {
    "RWTexture2D<float4>",
  };

  return isTypeHelper(typeName, typeNames);
}

std::string getTypeNameWithoutShaderClassPrefix(const QualType& type, const CXXRecordDecl* shaderClassDecl, const PrintingPolicy& printingPolicy) {
  std::string prefix = (shaderClassDecl->getName() + "::").str();
  std::string typeName = type.getAsString(printingPolicy);

  StringRef typeRef = typeName;
  typeRef.consume_front(prefix);

  // TODO Figure out a better long-term solution rather than having special cases
  // Special cases
  if (typeName == ("RWTexture2D<" + prefix + "float4>"))
    return "RWTexture2D<float4>";

  return typeRef.str();
}


// ThuscASTVisitor Helper methods
struct HLSLSemanticInfo {
  std::string thuscSpelling;
  ShaderParameterSemanticKind semanticKind;
  std::vector<std::string> validTypes;
};

const HLSLSemanticInfo kSupportedHLSLComputeShaderSemantics[] = {
  { THUSC_SV_DISPATCH_THREAD_ID_SPELLING, ShaderParameterSemanticKind::SV_DispatchThreadID, {"uint", "uint2", "uint3"} },
  { THUSC_SV_GROUP_THREAD_ID_SPELLING, ShaderParameterSemanticKind::SV_GroupThreadID, {"uint", "uint2", "uint3"} },
  { THUSC_SV_GROUP_ID_SPELLING, ShaderParameterSemanticKind::SV_GroupID, {"uint", "uint2", "uint3"} },
  { THUSC_SV_GROUP_INDEX_SPELLING, ShaderParameterSemanticKind::SV_GroupIndex, {"uint"} },
};

ShaderParameterSemanticKind ThuscASTVisitor::getComputeEntryParameterSemantic(const ParmVarDecl* param,
                                                                              const CXXRecordDecl* shaderClassDecl) const {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  const LangOptions& langOpts = rewriter.getLangOpts();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  std::vector<const AnnotateAttr*> attrs = findAnnotateAttributeMultiple(param, THUSC_HLSL_SEMANTIC_COMPUTESHADER_PREFIX, true);

  if (attrs.size() != 1) {
    diagError(diagEngine, param->getLocation(), "Computer shader entry function parameters must have exactly one HLSL semantic (e.g., [[SV_DispatchThreadID]])");
    return ShaderParameterSemanticKind::Unknown;
  }

  StringRef annotation = attrs[0]->getAnnotation();
  std::string typeName = getTypeNameWithoutShaderClassPrefix(param->getType(), shaderClassDecl, printingPolicy);

  for (const auto& semanticInfo : kSupportedHLSLComputeShaderSemantics) {
    if (annotation != semanticInfo.thuscSpelling)
      continue;

    for (const StringRef t : semanticInfo.validTypes) {
      if (typeName == t) {
        // Found valid semantic
        return semanticInfo.semanticKind;
      }
    }

    diagError(diagEngine, param->getTypeSpecStartLoc(), "Invalid type for specified compute shader semantic");
    return ShaderParameterSemanticKind::Unknown;
  }

  diagError(diagEngine, attrs[0]->getLocation(), "Unsupported compute shader semantic");
  return ShaderParameterSemanticKind::Unknown;
}


// ThuscASTVisitor functions

bool ThuscASTVisitor::VisitCXXRecordDecl(CXXRecordDecl* classDecl) {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  // If this is just a GPU struct
  const AnnotateAttr* declAttr = findAnnotateAttribute(classDecl, THUSC_GPU_FN_SPELLING);
  if (declAttr != nullptr) {
    removeCXX11AttrMacro(declAttr, rewriter);

    if (classDecl->method_begin() != classDecl->method_begin()) {
      diagError(diagEngine, classDecl->getLocation(), "GPU structs cannot contain methods");
      return false;
    }

    // TODO Add support for decls to also remain in host code
    shaderIR->otherGPUDecls.push_back(OtherGPUDecl(classDecl, false));
    return true;
  }


  // If this class doesn't have the ThuscShader attribute, then return
  const Attr* classAttr = findAnnotateAttribute(classDecl, THUSC_SHADER_SPELLING);
  if (classAttr == nullptr)
    return true;

  std::unique_ptr<ShaderClass> shaderClass = ShaderClass::create();

  shaderClass->classDecl = classDecl;
  shaderClass->name = classDecl->getName().str();

  // Remove the macro call that generated this annotation, because the underlying compiler won't understand it.
  removeCXX11AttrMacro(classAttr, rewriter);

  // Look for a ShaderClass base class
  if (shaderClass->classDecl->getNumBases() == 1) {
    const CXXRecordDecl* baseDecl = shaderClass->classDecl->bases_begin()->getType()->getAsCXXRecordDecl();
    shaderClass->setBaseShaderClass(shaderIR->getShaderClass(baseDecl));
  }

  if (!HandleUniformFields(classDecl, shaderClass.get()))
    return false;

  if (!HandlePermutationFields(classDecl, shaderClass.get()))
    return false;

  if (!HandleHLSLPrecodePostcode(classDecl, shaderClass.get()))
    return false;

  if (!HandleOtherGPUDecls(classDecl, shaderClass.get()))
    return false;

  if (!HandleGPUMethods(classDecl, shaderClass.get()))
    return false;

  if (!HandleShaderEntryFunctions(classDecl, shaderClass.get()))
    return false;

  // Check for ShouldCompilePermutation() method
  for (const auto method : classDecl->methods()) {
    if (method->getNameAsString() == "ShouldCompilePermutation") {
      shaderClass->hasShouldCompilePermutation = true;
      break;
    }
  }

  // Find the call to the THUSC_SHADER() macro and replace it with other code
  for (const auto method : classDecl->methods()) {
    if (method->getNameAsString() == "ThuscShaderStubFunc") {
      auto macroCall = sourceMgr.getExpansionLoc(method->getLocation());

      // Comment out the call to the THUSC_SHADER() macro, because the underlying compiler won't understand it
      rewriter.InsertText(macroCall, "//", true, true);

      break;
    }
  }

  shaderIR->addShaderClass(std::move(shaderClass));

  return true;
}

bool ThuscASTVisitor::VisitFunctionDecl(clang::FunctionDecl* funcDecl) {
  // Class methods will be handled separately, so nothing to do here for them
  if (isa<CXXMethodDecl>(funcDecl)) {
    return true;
  }

  const AnnotateAttr* funcAttr = findAnnotateAttribute(funcDecl, THUSC_GPU_FN_SPELLING);
  if (funcAttr == nullptr)
    return true;

  removeCXX11AttrMacro(funcAttr, rewriter);

  shaderIR->gpuFunctions.push_back(CreateGPUFunctionIR(funcDecl));

  return true;
}

bool ThuscASTVisitor::VisitDecl(Decl* decl) {
  const AnnotateAttr* declAttr = findAnnotateAttribute(decl, THUSC_GPU_FN_SPELLING);
  if (declAttr == nullptr)
    return true;

  if (const VarDecl* varDecl = dyn_cast<VarDecl>(decl)) {
    if (!varDecl->getType().isConstQualified()) {
      diagError(diagEngine, varDecl->getLocation(), "GPU global variable must be const");
      return false;
    }

    if (varDecl->isStaticDataMember()) {
      // Handled within the ShaderClass
      return true;
    }

    // Special case for varaibles that are assigned using simple integer literals
    if ((varDecl->getType()->isIntegerType() || varDecl->getType()->isUnsignedIntegerType())
        && isa<IntegerLiteral>(varDecl->getAnyInitializer()->IgnoreImpCasts())) {
      const auto* intLiteral = cast<IntegerLiteral>(varDecl->getAnyInitializer()->IgnoreImpCasts());
      std::string output = "#define " + varDecl->getNameAsString() + " " + rewriter.getRewrittenText(intLiteral->getSourceRange()) + "\n";
      // TODO Support host decls too
      shaderIR->otherGPUDecls.push_back(OtherGPUDecl(varDecl, false, true, output));
    }
    else {
      // TODO Probably want to check the scope here too, because we might want to handle
      //      const vars declared at ShaderClass scope separately

      // TODO Support host decls too
      shaderIR->otherGPUDecls.push_back(OtherGPUDecl(varDecl, false));
    }
  }
  else {
    return true;
  }

  removeCXX11AttrMacro(declAttr, rewriter);

  return true;
}

bool ThuscASTVisitor::HandleUniformFields(CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass) {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  const LangOptions& langOpts = rewriter.getLangOpts();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  // Foreach field that is a uniform parameter
  for (const auto field : shaderClassDecl->fields()) {
    const AnnotateAttr* fieldAttr = findAnnotateAttribute(field, THUSC_UNIFORM_PREFIX, true);
    if (fieldAttr == nullptr)
      continue;

    // Remove the macro call that generated this annotation, because the underlying compiler won't understand it.
    removeCXX11AttrMacro(fieldAttr, rewriter);

    std::string typeString = getTypeNameWithoutShaderClassPrefix(field->getType(), shaderClassDecl, printingPolicy);

    UniformField::Kind kind = UniformField::Kind::Unknown;

    // Determine the uniform's type
    if (isSamplerType(typeString)) {
      kind = UniformField::Kind::Sampler;
    }
    else if (isTextureType(typeString)) {
      kind = UniformField::Kind::Texture;
    }
    else if (isTextureSRVType(typeString)) {
      kind = UniformField::Kind::TextureSRV;
    }
    else if (isTextureUAVType(typeString)) {
      kind = UniformField::Kind::TextureUAV;
    }
    else if (isPrimitiveType(getGpuTypeName(field->getType(), field->getASTContext()))) {
      kind = UniformField::Kind::Primitive;
    }
    else if (field->getType()->isConstantArrayType()) {
      kind = UniformField::Kind::Array;

      const ConstantArrayType* arrType = cast<ConstantArrayType>(field->getType().getTypePtr());
      typeString = getTypeNameWithoutShaderClassPrefix(arrType->getElementType(), shaderClassDecl, printingPolicy);

      if (!isPrimitiveType(getGpuTypeName(arrType->getElementType(), field->getASTContext()))) {
        diagError(diagEngine, field->getLocation(), "Array uniform parameters must be arrays of primitive types");
        return false;
      }
    }
    else if (StringRef(typeString).startswith("GlobalUniformBuffer")) {
      kind = UniformField::Kind::GlobalUniformBuffer;

      StringRef typeStringRef(typeString);

      if (!typeStringRef.consume_front("GlobalUniformBuffer<") || !typeStringRef.consume_back(">")) {
        diagError(diagEngine, field->getLocation(), "Cannot get template parameter type for GlobalUniformBuffer type. Expected type syntax to be 'GlobalUniformBuffer<structType>'.");
        return false;
      }

      typeString = typeStringRef.str();
    }
    else if (field->getType()->isStructureOrClassType()) {
      kind = UniformField::Kind::Struct;
    }
    else if (field->getType()->isPointerType()) {
      diagError(diagEngine, field->getLocation(), "Pointer type uniform parameters are not allowed");
      return false;
    }
    else {
      diagError(diagEngine, field->getLocation(), "Unsupported uniform parameter type");
      return false;
    }

    UniformField uniformField(field, field->getName(), typeString, kind);
    shaderClass->uniformFields.push_back(std::move(uniformField));
  }

  return true;
}

bool ThuscASTVisitor::HandlePermutationFields(CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass) {
  // Foreach field that is a permutation parameter
  for (const auto field : shaderClassDecl->fields()) {
    const AnnotateAttr* fieldAttr = findAnnotateAttribute(field, THUSC_PERMUTATION_PREFIX, true);
    if (fieldAttr == nullptr)
      continue;

    // Remove the macro call that generated this annotation, because the underlying compiler won't understand it.
    removeCXX11AttrMacro(fieldAttr, rewriter);

    // Determine which permutation type it is, and create a ShaderIR member accordingly
    bool shouldContinue = true;
    StringRef annotation = fieldAttr->getAnnotation();

    // Handle ShaderClass permutations separately
    if (annotation.consume_front(THUSC_PERMUTATION_SHADERCLASS_SPELLING)) {
      const ShaderClass* shaderClassType = shaderIR->getShaderClass(field->getType()->getPointeeCXXRecordDecl());
      if (!shaderClassType) {
        diagError(diagEngine, field->getLocation(), "The type of a permutation ShaderClass field must be a pointer to a ShaderClass");
        continue;
      }

      shaderClass->permutationShaderClasses.push_back(PermutationShaderClass(field, field->getName(), shaderClassType, shaderIR));

      continue;
    }

    // Handle other types of permutations
    QualType typeWithoutQualifiers(QualType(field->getType().getTypePtr(), 0));
    std::string typeString = typeWithoutQualifiers.getAsString(printingPolicy);

    PermutationField::Kind kind = PermutationField::Kind::Unknown;
    SmallVector<std::string, 8> valueOptions;

    // Enum
    if (annotation.equals(THUSC_PERMUTATION_ENUM_SPELLING)) {
      if (!field->getType()->isEnumeralType()) {
        diagError(diagEngine, field->getTypeSpecStartLoc(), "Permutation enum specified, but field type is not an enum");
      }
      else {
        kind = PermutationField::Kind::Enum;
        valueOptions.push_back(typeString);
      }
    }
    // Bool
    else if (annotation.consume_front(THUSC_PERMUTATION_BOOL_SPELLING)) {
      if (!field->getType()->isBooleanType()) {
        diagError(diagEngine, field->getTypeSpecStartLoc(), "Permutation bool specified, but field type is not a bool");
      }
      else {
        kind = PermutationField::Kind::Bool;
      }
    }
    // Int (range from 0 to N)
    else if (annotation.consume_front(THUSC_PERMUTATION_INT_SPELLING)) {
      if (!field->getType()->isIntegralType(field->getASTContext())) {
        diagError(diagEngine, field->getTypeSpecStartLoc(), "Permutation Int specified, but field type is not an int");
      }
      else if (annotation.equals("")) {
        diagError(diagEngine, fieldAttr->getLocation(), "Permutation Int declaration requires one decimal integer value");
      }
      else {
        kind = PermutationField::Kind::Int;

        int value;
        // Using 10 as the Radix to indicate that the integers should be specified in decimal form
        if (annotation.trim().getAsInteger(10, value)) {
          diagError(diagEngine, fieldAttr->getLocation(), "Permutation Int declaration requires one decimal integer value");
          return false;
        }

        valueOptions.push_back(annotation.str());
      }
    }
    // Sparse Int
    else if (annotation.consume_front(THUSC_PERMUTATION_SPARSE_INT_SPELLING)) {
      if (!field->getType()->isIntegralType(field->getASTContext())) {
        diagError(diagEngine, field->getTypeSpecStartLoc(), "Permutation SparseInt specified, but field type is not an int");
      }
      else if (annotation.equals("")) {
        diagError(diagEngine, fieldAttr->getLocation(), "Permutation SparseInt declaration requires one or more decimal integer values (separated by commas)");
      }
      else {
        kind = PermutationField::Kind::SparseInt;

        SmallVector<StringRef, 8> sparseIntStrings;
        annotation.split(sparseIntStrings, ',');

        for (auto v : sparseIntStrings) {
          int value;
          // Using 10 as the Radix to indicate that the integers should be specified in decimal form
          if (v.trim().getAsInteger(10, value)) {
            diagError(diagEngine, fieldAttr->getLocation(), "Permutation SparseInt declaration requires one or more decimal integer values (separated by commas)");
            break;
          }

          valueOptions.push_back(v.str());
        }

        assert(sparseIntStrings.size() == valueOptions.size() && "When parsing Permutation SparseInt, number of parsed integers does not equal number of string elements");
      }
    }
    // TODO Add other permutation types
    else {
      diagError(diagEngine, fieldAttr->getLocation(), "Unrecognized permutation type");
    }

    PermutationField permField(field, field->getName(), typeString, std::move(valueOptions), kind);
    shaderClass->permutationFields.push_back(std::move(permField));

    if (!shouldContinue)
      return false;
  }

  return true;
}

bool ThuscASTVisitor::HandleShaderEntryFunctions(const CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass) {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  const LangOptions& langOpts = rewriter.getLangOpts();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  for (const auto method : shaderClassDecl->methods()) {
    const AnnotateAttr* methodAttr = findAnnotateAttribute(method, THUSC_SHADER_ENTRY_PREFIX, true);
    if (methodAttr == nullptr)
      continue;

    if (method->getAttrs().size() != 1) {
      diagError(diagEngine, method->getLocation(), "Shader entry functions must have only one attribute");
    }

    removeCXX11AttrMacro(methodAttr, rewriter);

    if (!method->getReturnType()->isVoidType()) {
      diagError(diagEngine, method->getReturnTypeSourceRange().getBegin(), "Shader entry functions must return void");
    }

    if (!method->isConst()) {
      diagError(diagEngine, method->getLocation(), "Shader entry functions must be 'const'");
    }

    if (!method->hasBody()) {
      diagError(diagEngine, method->getLocation(), "Shader entry functions must have a function body");
    }

    // Determine which shader entry type it is, and create a ShaderIR member accordingly
    bool shouldContinue = true;
    StringRef annotation = methodAttr->getAnnotation();

    std::unique_ptr<EntryPoint> entryPoint;

    // Compute shader entry
    if (annotation.consume_front(THUSC_ENTRY_COMPUTE_SPELLING)) {
      // Get threadgroup size info
      SmallVector<StringRef, 8> threadgroupSize;
      annotation.split(threadgroupSize, ',');

      assert(threadgroupSize.size() == 3 && "When parsing compute shader entry, number of groupsize elements does not equal 3");

      // Verify that threadgroupSize parameters are either integer literals or permutation parameters
      for (const auto s : threadgroupSize) {
        // Integer literal
        int value;
        if (!s.trim().getAsInteger(10, value)) {
          // Do nothing
        }
        // Permutation parameter
        else if (shaderClass->isIntPermutationField(s)) {
          // Do nothing
        }
        else {
          // TODO Commenting this out for now, since we can also use [[gpu]] const integers that are initialized with simple integer literals,
          //      but we don't have a good way to map from the annotation string to the such a declaration right now.
          //diagError(diagEngine, methodAttr->getLocation(), "Threadgroup size values for compute shader entry functions must be either integer literals or integer permtuation parameters");
        }
      }

      std::unique_ptr<ComputeEntryPoint> computeEntry = std::make_unique<ComputeEntryPoint>(method, method->getName(), threadgroupSize);

      // Add entry point parameters, including HLSL semantics info
      for (const auto param : method->parameters()) {
        ShaderParameterSemanticKind semantic = getComputeEntryParameterSemantic(param, shaderClassDecl);

        if (semantic == ShaderParameterSemanticKind::Unknown) {
          // Error already reported
          return false;
        }
        computeEntry->addParameter(param->getName(), getTypeNameWithoutShaderClassPrefix(param->getType(), shaderClassDecl, printingPolicy), semantic);
      }

      entryPoint = std::move(computeEntry);
    }
    // TODO Add other types of shader entry functions
    else {
      diagError(diagEngine, methodAttr->getLocation(), "Unrecognized shader entry type");
    }

    shaderClass->entryPoints.push_back(std::move(entryPoint));

    if (!shouldContinue)
      return false;
  }

  return true;
}

bool ThuscASTVisitor::HandleOtherGPUDecls(const CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass) {
  for (const auto* decl : shaderClassDecl->decls()) {
    const AnnotateAttr* declAttr = findAnnotateAttribute(decl, THUSC_GPU_FN_SPELLING);
    if (declAttr == nullptr)
      continue;

    removeCXX11AttrMacro(declAttr, rewriter);

    if (const FieldDecl* fieldDecl = dyn_cast<FieldDecl>(decl)) {
      if (fieldDecl->getType().isConstQualified()) {
        std::string output = "static " + rewriter.getRewrittenText(fieldDecl->getSourceRange()) + ";";
        // TODO add support for host side too
        shaderClass->otherGPUDecls.push_back(OtherGPUDecl(fieldDecl, false, true, output));
      }
      else {
        // TODO add support for host side too
        shaderClass->otherGPUDecls.push_back(OtherGPUDecl(fieldDecl, false));
      }
    }
    else if (const VarDecl* varDecl = dyn_cast<VarDecl>(decl)) {
      if (!varDecl->isStaticDataMember()) {
        diagError(diagEngine, decl->getLocation(), "Unsupported [[gpu]] decl in Shader Class (must be static)");
      }
      else {
        // TODO add support for host side too
        shaderClass->otherGPUDecls.push_back(OtherGPUDecl(varDecl, false));
      }
    }
    else if (isa<CXXMethodDecl>(decl)) {
      // Do nothing (handled in HandleGPUMethods() function)
    }
    // TODO add other cases
    else {
      diagError(diagEngine, decl->getLocation(), "Unsupported [[gpu]] decl in Shader Class");
    }
  }

  return true;
}

bool ThuscASTVisitor::HandleGPUMethods(const CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass) {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  const LangOptions& langOpts = rewriter.getLangOpts();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  for (const auto method : shaderClassDecl->methods()) {
    const AnnotateAttr* methodAttr = findAnnotateAttribute(method, THUSC_GPU_FN_SPELLING);
    if (methodAttr == nullptr)
      continue;

    removeCXX11AttrMacro(methodAttr, rewriter);

    if (!method->isConst()) {
      diagError(diagEngine, method->getLocation(), "ShaderClass GPU methods must be 'const'");
    }

    shaderClass->addGPUMethod(CreateGPUFunctionIR(method));
  }

  return true;
}

bool ThuscASTVisitor::HandleHLSLPrecodePostcode(const CXXRecordDecl* shaderClassDecl, thusc::ShaderClass* shaderClass) {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  const LangOptions& langOpts = rewriter.getLangOpts();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  for (const auto* method : shaderClassDecl->methods()) {
    const AnnotateAttr* methodAttr = findAnnotateAttribute(method, THUSC_HLSL_PRECODE_SPELLING);

    bool isPrecode = false;
    if (methodAttr != nullptr) {
      isPrecode = true;
    }
    else {
      methodAttr = findAnnotateAttribute(method, THUSC_HLSL_POSTCODE_SPELLING);
    }

    if (methodAttr == nullptr) {
      continue;
    }

    if (method->getAttrs().size() != 1) {
      diagError(diagEngine, method->getLocation(), "PrecodeHLSL and PostcodeHLSL functions must have only one attribute");
    }

    removeCXX11AttrMacro(methodAttr, rewriter);

    if (!method->isConst()) {
      diagError(diagEngine, method->getLocation(), "PrecodeHLSL and PostcodeHLSL class functions must be 'const'");
    }

    if (!method->hasBody()) {
      diagError(diagEngine, method->getLocation(), "PrecodeHLSL and PostcodeHLSL functions must have a function body");
    }

    std::string code = rewriter.getRewrittenText(
      SourceRange(method->getBody()->getBeginLoc().getLocWithOffset(1), method->getBodyRBrace().getLocWithOffset(-1)));

    if (isPrecode) {
      shaderClass->hlslPrecode.append(code);
    }
    else {
      shaderClass->hlslPostcode.append(code);
    }

    rewriter.RemoveText(method->getSourceRange());
  }

  return true;
}

std::unique_ptr<GPUFunction> ThuscASTVisitor::CreateGPUFunctionIR(const FunctionDecl* funcDecl) {
  const SourceManager& sourceMgr = rewriter.getSourceMgr();
  const LangOptions& langOpts = rewriter.getLangOpts();
  DiagnosticsEngine& diagEngine = sourceMgr.getDiagnostics();

  // TODO commenting this out, because c++ override counts as an attribute. Figure out a better solution.
  //if (funcDecl->getAttrs().size() != 1) {
  //  diagError(diagEngine, funcDecl->getLocation(), "GPU functions must have only one attribute");
  //}

  if (!funcDecl->hasBody()) {
    diagError(diagEngine, funcDecl->getLocation(), "GPU functions must have a function body");
  }

  // TODO Add support for host-callable GPU functions
  std::unique_ptr<GPUFunction> gpuFunction = std::make_unique<GPUFunction>(funcDecl, funcDecl->getName(),
    funcDecl->getReturnType().getAsString(printingPolicy), false);

  for (const auto param : funcDecl->parameters()) {
    bool isInout = (findAnnotateAttribute(param, THUSC_HLSL_INOUT_SPELLING)) ? true : false;

    gpuFunction->addParameter(param->getName(), param->getType().getAsString(printingPolicy), isInout);
  }

  return std::move(gpuFunction);
}

