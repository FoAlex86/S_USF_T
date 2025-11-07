// VTURaymarchCS.h
#pragma once
#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "ShaderParameters.h"
#include "RenderGraphResources.h"
#include "DataDrivenShaderPlatformInfo.h"

BEGIN_SHADER_PARAMETER_STRUCT(FVTURaymarchCSParams, )
    // Sortie
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D, OutTex)

    // Ecran
    SHADER_PARAMETER(FUintVector2, RTSize)
    SHADER_PARAMETER(uint32, RTWidth)
    SHADER_PARAMETER(uint32, RTHeight)

    // Caméra (base + FOV/Aspect)
    SHADER_PARAMETER(FMatrix44f, InvViewProj)
    SHADER_PARAMETER(FVector4f, BasisRight_TanY)  // .xyz = RightWS, .w = TanHalfFovY
    SHADER_PARAMETER(FVector4f, BasisUp_Aspect)   // .xyz = UpWS,    .w = Aspect
    SHADER_PARAMETER(FVector4f, BasisFwd_Flip)    // .xyz = FwdWS,   .w = bFlipY (0/1)
    SHADER_PARAMETER(FVector4f, CamPosWS)         // .xyz = origin

    // Objet & clip
    SHADER_PARAMETER(FMatrix44f, WorldToLocal)
    SHADER_PARAMETER(FVector4f, ClipPlaneWS)
    SHADER_PARAMETER(uint32, UseClip)
    SHADER_PARAMETER(uint32, Pad0)
    SHADER_PARAMETER(uint32, Pad1)
    SHADER_PARAMETER(uint32, Pad2)

    // AABB locale
    SHADER_PARAMETER(FVector4f, UnionMinLS4)
    SHADER_PARAMETER(FVector4f, UnionMaxLS4)

    // ====== SRV VTU ======
    SHADER_PARAMETER_SRV(StructuredBuffer<float3>, Points)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, Conn)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, Offs)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, Types)
    SHADER_PARAMETER_SRV(StructuredBuffer<float>, FeatureVals)

    SHADER_PARAMETER(uint32, NumPoints)
    SHADER_PARAMETER(uint32, NumCells)
END_SHADER_PARAMETER_STRUCT()

class FVTURaymarchCS : public FGlobalShader
{
    DECLARE_EXPORTED_SHADER_TYPE(FVTURaymarchCS, Global, );
    using FParameters = FVTURaymarchCSParams;
    SHADER_USE_PARAMETER_STRUCT(FVTURaymarchCS, FGlobalShader);

public:
    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.SetDefine(TEXT("THREADGROUP_SIZE_X"), 8);
        OutEnvironment.SetDefine(TEXT("THREADGROUP_SIZE_Y"), 8);
        OutEnvironment.SetDefine(TEXT("THREADGROUP_SIZE_Z"), 1);
    }
};