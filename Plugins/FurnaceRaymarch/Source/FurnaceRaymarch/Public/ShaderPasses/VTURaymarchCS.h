// VTURaymarchCS.h
#pragma once
#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "ShaderParameters.h"
#include "RenderGraphResources.h"
#include "DataDrivenShaderPlatformInfo.h"

/*
 * --------------------------------------------------------------------------
 * FVTURaymarchCSParams
 * --------------------------------------------------------------------------
 * Bloc de paramètres RDG passé au compute shader (VTURaymarchCS.usf).
 */
BEGIN_SHADER_PARAMETER_STRUCT(FVTURaymarchCSParams, )
    // ------------------ Sortie (UAV) ------------------
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D, OutTex)

    // ------------------ Ecran -------------------------
    SHADER_PARAMETER(FUintVector2, RTSize)
    SHADER_PARAMETER(uint32, RTWidth)
    SHADER_PARAMETER(uint32, RTHeight)

    // ------------------ Caméra ------------------------
    SHADER_PARAMETER(FMatrix44f, InvViewProj)

    // ------------------ Objet & clip ------------------
    // Matrice monde->local de l’ACTEUR
    SHADER_PARAMETER(FMatrix44f, WorldToLocal)
    // Plan de coupe en WS
    SHADER_PARAMETER(FVector4f, ClipPlaneWS)
    // UseClip : 1=clip activé, 0=off.
    SHADER_PARAMETER(uint32, UseClip)
    SHADER_PARAMETER(uint32, Pad0)
    SHADER_PARAMETER(uint32, Pad1)
    SHADER_PARAMETER(uint32, Pad2)

    // ------------------ AABB locale (volume actif) ----
    SHADER_PARAMETER(FVector4f, UnionMinLS4)
    SHADER_PARAMETER(FVector4f, UnionMaxLS4)

    // ------------------ SRV VTU (grille) --------------
    SHADER_PARAMETER_SRV(StructuredBuffer<float3>, Points)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, Conn)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, Offs)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, Types)
    SHADER_PARAMETER_SRV(StructuredBuffer<float>, FeatureVals)
    SHADER_PARAMETER_SRV(StructuredBuffer<int>, Faces)
    SHADER_PARAMETER_SRV(StructuredBuffer<int>, FaceOffsets)

    // Compteurs grille :
    SHADER_PARAMETER(uint32, NumPoints)
    SHADER_PARAMETER(uint32, NumCells)

    // ------------------ Octree Nodes (SoA) ------------
    // Boîtes englobantes des nœuds
    SHADER_PARAMETER_SRV(StructuredBuffer<FVector3f>, OctNodeCenter)
    SHADER_PARAMETER_SRV(StructuredBuffer<FVector3f>, OctNodeExtent)
    // Arborescence (indices enfants)
    SHADER_PARAMETER_SRV(StructuredBuffer<int>, OctNodeFirstChild)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, OctNodeChildCount)
    SHADER_PARAMETER_SRV(StructuredBuffer<int>, OctNodeFirstElem)
    SHADER_PARAMETER_SRV(StructuredBuffer<uint>, OctNodeElemCount)

    // --- Children list aplatie ---
    SHADER_PARAMETER_SRV(StructuredBuffer<int>, NodeChildIndex)

    // ------------------ Eléments (SoA) ----------------
    SHADER_PARAMETER_SRV(StructuredBuffer<FVector3f>, ElemMin)
    SHADER_PARAMETER_SRV(StructuredBuffer<FVector3f>, ElemMax)
    SHADER_PARAMETER_SRV(StructuredBuffer<int>, ElemCell)

    // ------------------ Compteurs & racine ------------
    SHADER_PARAMETER(uint32, NumNodes)
    SHADER_PARAMETER(uint32, NumElems)
    SHADER_PARAMETER(uint32, RootIndex)

    // ------------------ Shading / debug ----------------
    SHADER_PARAMETER(uint32, DebugDrawMode) // 0=off, 1=nodes-only, 2=elems AABB
    SHADER_PARAMETER(uint32, UseFeatureVals) // 0 = normale, 1 = FeatureVals[cell]

    SHADER_PARAMETER(float, VMin)
    SHADER_PARAMETER(float, VMax)

    SHADER_PARAMETER(float, EpsCm)
    SHADER_PARAMETER(FVector3f, CameraPosWS)
END_SHADER_PARAMETER_STRUCT()

class FVTURaymarchCS : public FGlobalShader
{
    DECLARE_EXPORTED_SHADER_TYPE(FVTURaymarchCS, Global, );
    using FParameters = FVTURaymarchCSParams;
    // Binde la struct de params à la classe shader.
    SHADER_USE_PARAMETER_STRUCT(FVTURaymarchCS, FGlobalShader);

public:
    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        // Définit les constantes de compilation pour la grille de threads.
        // Ces #define sont vus côté HLSL (VTURaymarchCS.usf) pour dimensionner numthreads().
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.SetDefine(TEXT("THREADGROUP_SIZE_X"), 8);
        OutEnvironment.SetDefine(TEXT("THREADGROUP_SIZE_Y"), 8);
        OutEnvironment.SetDefine(TEXT("THREADGROUP_SIZE_Z"), 1);
    }
};