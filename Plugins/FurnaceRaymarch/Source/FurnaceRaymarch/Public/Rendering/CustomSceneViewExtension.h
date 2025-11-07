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

	// Views associées (ce que lit le shader)
	FShaderResourceViewRHIRef  PointsSRV;
	FShaderResourceViewRHIRef  ConnSRV;
	FShaderResourceViewRHIRef  OffsSRV;
	FShaderResourceViewRHIRef  TypesSRV;
	FShaderResourceViewRHIRef  FeatureValsSRV;

	uint32 NumPoints = 0;
	uint32 NumCells = 0;
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

#if ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION >= 5
	virtual void SubscribeToPostProcessingPass(EPostProcessingPass Pass, const FSceneView& View, FAfterPassCallbackDelegateArray& InOutPassCallbacks, bool bIsPassEnabled);
#else
	virtual void SubscribeToPostProcessingPass(EPostProcessingPass Pass, FAfterPassCallbackDelegateArray& InOutPassCallbacks, bool bIsPassEnabled);
#endif

	void SetRaymarchTargetObject(UTextureRenderTarget2D* InRT);

	// NEW: setter des SRV ByteAddress (à appeler côté Subsystem)
	void SetVTUGPUResources(const FVTUGPUResourcesBA& In);
	void ClearVTUGPUResources_RT();

	// Appelés depuis le GameThread via ENQUEUE_RENDER_COMMAND
	void SetWorldToLocal_RT(const FMatrix44f& InWorldToLocal) { WorldToLocal_RT = InWorldToLocal; bHasWorldToLocal = true; }
	void SetDebugBoundsWS_RT(const FVector3f& MinWS, const FVector3f& MaxWS) { DebugMinWS = MinWS; DebugMaxWS = MaxWS; bHasDebugBounds = true; }

private:
	FScreenPassTexture CustomPostProcessFunction(FRDGBuilder& GraphBuilder, const FSceneView& SceneView, const FPostProcessMaterialInputs& Inputs);

	TWeakObjectPtr<UTextureRenderTarget2D> RaymarchRT_Object;

	// Ressources GPU (visibles du Render Thread)
	FVTUGPUResourcesBA GPUResBA;
	bool bHasGPUResBA = false;

	bool      bHasWorldToLocal = false;
	FMatrix44f WorldToLocal_RT = FMatrix44f::Identity;

	bool      bHasDebugBounds = false;
	FVector3f DebugMinWS = FVector3f(-50);
	FVector3f DebugMaxWS = FVector3f(50);
};