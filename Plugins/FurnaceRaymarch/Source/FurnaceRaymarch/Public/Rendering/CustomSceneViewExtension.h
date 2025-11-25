// CustomSceneViewExtension.h
#pragma once

#include "CoreMinimal.h"
#include "SceneViewExtension.h"
#include "PostProcess/PostProcessing.h"
#include "RHI.h"
#include "RHIResources.h"

class UTextureRenderTarget2D;

// Ressources VTU en StructuredBuffer (on garde les buffers + leurs SRV)
struct FVTUGPUResourcesBA
{
	// Buffers RHI (doivent vivre tant que les SRV sont utilisés)
	FBufferRHIRef              PointsBuffer;
	FBufferRHIRef              ConnBuffer;
	FBufferRHIRef              OffsBuffer;
	FBufferRHIRef              TypesBuffer;
	FBufferRHIRef              FeatureValsBuffer;
	FBufferRHIRef              FacesBuffer;
	FBufferRHIRef              FaceOffsetsBuffer;

	// Views associées (ce que lit le shader)
	FShaderResourceViewRHIRef  PointsSRV;
	FShaderResourceViewRHIRef  ConnSRV;
	FShaderResourceViewRHIRef  OffsSRV;
	FShaderResourceViewRHIRef  TypesSRV;
	FShaderResourceViewRHIRef  FeatureValsSRV;
	FShaderResourceViewRHIRef  FacesSRV;
	FShaderResourceViewRHIRef  FaceOffsetsSRV;

	uint32 NumPoints = 0;
	uint32 NumCells = 0;

	// --- Octree (buffers à garder en vie + SRV) ---
	FBufferRHIRef             OctNodeCenterBuffer;
	FBufferRHIRef             OctNodeExtentBuffer;
	FBufferRHIRef             OctNodeFirstChildBuffer;
	FBufferRHIRef             OctNodeChildCountBuffer;
	FBufferRHIRef             OctNodeFirstElemBuffer;
	FBufferRHIRef             OctNodeElemCountBuffer;
	FBufferRHIRef             NodeChildIndexBuffer;

	// --- Octree (SRV) ---
	FShaderResourceViewRHIRef  OctNodeCenterSRV;
	FShaderResourceViewRHIRef  OctNodeExtentSRV;
	FShaderResourceViewRHIRef  OctNodeFirstChildSRV;
	FShaderResourceViewRHIRef  OctNodeChildCountSRV;
	FShaderResourceViewRHIRef  OctNodeFirstElemSRV;
	FShaderResourceViewRHIRef  OctNodeElemCountSRV;

	FShaderResourceViewRHIRef  NodeChildIndexSRV;

	// --- Elements attachés aux noeuds ---
	FBufferRHIRef             ElemMinBuffer;
	FBufferRHIRef             ElemMaxBuffer;
	FBufferRHIRef             ElemCellBuffer;

	FShaderResourceViewRHIRef  ElemMinSRV;
	FShaderResourceViewRHIRef  ElemMaxSRV;
	FShaderResourceViewRHIRef  ElemCellSRV;

	uint32 NumNodes = 0;
	uint32 NumElems = 0;
	uint32 RootIndex = 0;

	// shading
	//uint32 DebugDrawMode = 0;
	uint32 UseFeatureVals = 0;
	float  VMin = 0.f;
	float  VMax = 1.f;

	float  EpsCm = 0.1f;

	bool bSetUseFeatureVals = false;
	bool bSetVMinMax = false;
	bool bSetEps = false;

	void Reset()
	{
		*this = FVTUGPUResourcesBA(); // remet à zéro les refs et compteurs
	}
};

class FURNACERAYMARCH_API FCustomSceneViewExtension : public FSceneViewExtensionBase
{
public:
	FCustomSceneViewExtension(const FAutoRegister& AutoRegister);

	virtual void SetupViewFamily(FSceneViewFamily& InViewFamily) override {};
	virtual void SetupView(FSceneViewFamily& InViewFamily, FSceneView& InView) override {};
	virtual void BeginRenderViewFamily(FSceneViewFamily& InViewFamily) override {};

	virtual void PostRenderBasePassDeferred_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView, const FRenderTargetBindingSlots& RenderTargets, TRDGUniformBufferRef<FSceneTextureUniformParameters> SceneTextures) override {};
	virtual void PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily) override {};
	virtual void PreRenderView_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView) override {};
	virtual void PostRenderView_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView) override {};
	virtual void PrePostProcessPass_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View, const FPostProcessingInputs& Inputs) override;
	virtual void PostRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily) override {};

	void SetRaymarchTargetObject(UTextureRenderTarget2D* InRT);

	//setter des SRV ByteAddress
	void SetVTUGPUResources(const FVTUGPUResourcesBA& In);

	void ClearVTUGPUResources_RT();

	// Appelés depuis le GameThread via ENQUEUE_RENDER_COMMAND
	void SetWorldToLocal_RT(const FMatrix44f& In) { WorldToLocal_RT = In; bHasWorldToLocal = true; }
	/*void SetDebugBoundsWS_RT(const FVector3f& MinWS, const FVector3f& MaxWS) { DebugMinWS = MinWS; DebugMaxWS = MaxWS; bHasDebugBounds = true; }*/
	void SetBoundsLS_RT(const FVector3f& MinLS, const FVector3f& MaxLS) { BoundsMinLS = MinLS; BoundsMaxLS = MaxLS; bHasBoundsLS = true; }
	void SetDebugDrawMode_RT(uint32 InMode) {
		DebugDrawMode_RT = InMode; UE_LOG(LogTemp, Verbose, TEXT("[Raymarch] DebugDrawMode=%u"), DebugDrawMode_RT);
	}

private:
	FScreenPassTexture CustomPostProcessFunction(FRDGBuilder& GraphBuilder, const FSceneView& SceneView, const FPostProcessMaterialInputs& Inputs);

	TWeakObjectPtr<UTextureRenderTarget2D> RaymarchRT_Object;

	// Ressources GPU (visibles du Render Thread)
	FVTUGPUResourcesBA GPUResBA;
	bool bHasGPUResBA = false;
	uint32 DebugDrawMode_RT = 0;

	bool      bHasWorldToLocal = false;
	FMatrix44f WorldToLocal_RT = FMatrix44f::Identity;

	bool      bHasBoundsLS = false;
	FVector3f BoundsMinLS = FVector3f::ZeroVector;
	FVector3f BoundsMaxLS = FVector3f::ZeroVector;
};