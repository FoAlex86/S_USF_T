#include "Rendering/CustomSceneViewExtension.h"

#include "ShaderPasses/VTURaymarchCS.h"   // FVTURaymarchCS

#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderGraphEvent.h"

#include "ShaderParameterStruct.h"
#include "ShaderParameterUtils.h"         // SetShaderParameters / UnsetShaderUAVs
#include "PipelineStateCache.h"           // SetComputePipelineState
#include "RHICommandList.h"               // FRHIComputeCommandList
#include "Rendering/CustomSceneViewExtension.h"
#include "Engine/TextureRenderTarget2D.h" // UTextureRenderTarget2D
#include "SceneView.h"

// ------------------------------------------------------------------
// Construction / liaison de la RenderTarget
// ------------------------------------------------------------------

FCustomSceneViewExtension::FCustomSceneViewExtension(const FAutoRegister& AutoRegister)
	: FSceneViewExtensionBase(AutoRegister)
{
}

void FCustomSceneViewExtension::SetRaymarchTargetObject(UTextureRenderTarget2D* InRT)
{
	// Stocke juste l’objet ; on touchera la ressource côté Render Thread
	RaymarchRT_Object = InRT;
}

void FCustomSceneViewExtension::PrePostProcessPass_RenderThread(
	FRDGBuilder& GraphBuilder, const FSceneView& View, const FPostProcessingInputs& Inputs)
{
	// 0) Récupère l’objet RT et sa ressource (sur le Render Thread)
	UTextureRenderTarget2D* RTObj = RaymarchRT_Object.Get();
	if (!RTObj)
	{
		return;
	}

	FTextureRenderTargetResource* RTRes = RTObj->GetRenderTargetResource();
	if (!RTRes || !RTRes->IsInitialized())
	{
		return;
	}

	FTextureRHIRef RHITexture = RTRes->GetRenderTargetTexture();
	if (!RHITexture.IsValid())
	{
		return;
	}

	// 1) Expose la RT externe au Render Graph (destination du copy)
	FRDGTextureRef RDGOut = GraphBuilder.RegisterExternalTexture(
		CreateRenderTarget(RHITexture, TEXT("FurnaceRaymarch_RT"))
	);

	const FIntPoint SizeXY = RTRes->GetSizeXY();

	// 2) Crée une texture RDG INTERNE avec flag UAV
	//    PF_FloatRGBA correspond à RTF_RGBA16f.
	FRDGTextureDesc Desc = FRDGTextureDesc::Create2D(
		SizeXY,                        // FIntPoint
		PF_FloatRGBA,                  // EPixelFormat (match RTF_RGBA16f)
		FClearValueBinding::Transparent,
		TexCreate_ShaderResource | TexCreate_UAV   // <-- 4e arg obligatoire
	);
	FRDGTextureRef RDGInternal = GraphBuilder.CreateTexture(Desc, TEXT("FurnaceRaymarch_Internal"));

	 //3) Paramètres du compute : UAV = texture interne (PAS la RT externe)
	FVTURaymarchCS::FParameters* Params = GraphBuilder.AllocParameters<FVTURaymarchCS::FParameters>();
	Params->OutTex = GraphBuilder.CreateUAV(RDGInternal);
	Params->RTSize = FUintVector2((uint32)SizeXY.X, (uint32)SizeXY.Y);

	// Taille écran/RT
	Params->RTWidth = (uint32)SizeXY.X;
	Params->RTHeight = (uint32)SizeXY.Y;

	// InvViewProj (éviter le jitter si possible)
	Params->InvViewProj = FMatrix44f(View.ViewMatrices.GetInvViewProjectionMatrix());

	// Objet & clip
	Params->ClipPlaneWS = FVector4f(0, 0, 0, 0);
	Params->UseClip = 0;
	Params->Pad0 = Params->Pad1 = Params->Pad2 = 0;

	// === WorldToLocal & AABB monde venant des setters ===
	// (si non fournis, fallback: petite box devant la caméra)
	Params->WorldToLocal = bHasWorldToLocal ? WorldToLocal_RT : FMatrix44f::Identity;

	if (bHasDebugBounds)
	{
		Params->UnionMinLS4 = FVector4f(DebugMinWS, 0.f);
		Params->UnionMaxLS4 = FVector4f(DebugMaxWS, 0.f);
	}
	else
	{
		const FMatrix InvView = View.ViewMatrices.GetInvViewMatrix();
		const FVector FwdWS = InvView.GetUnitAxis(EAxis::X);
		const FVector3f CenterWS = (FVector3f)(View.ViewMatrices.GetViewOrigin() + 50.f * FwdWS);
		const FVector3f ExtentWS(50.f, 50.f, 50.f);
		Params->UnionMinLS4 = FVector4f(CenterWS - ExtentWS, 0.f);
		Params->UnionMaxLS4 = FVector4f(CenterWS + ExtentWS, 0.f);
	}


	// =================================================SRV======================================================
	// 1) Fallback SRV structuré 4 octets (créé une seule fois)
	static FBufferRHIRef               GNullBuffer;
	static FShaderResourceViewRHIRef   GNullSRV;

	if (!GNullSRV.IsValid())
	{
		FRHICommandListImmediate& RHICmdList = FRHICommandListExecutor::GetImmediateCommandList();

		FRHIResourceCreateInfo CreateInfo(TEXT("FRM_NullStructured4B"));
		const uint32 Stride = sizeof(uint32);
		const uint32 NumBytes = sizeof(uint32);

		GNullBuffer = RHICmdList.CreateStructuredBuffer(
			Stride, NumBytes,
			BUF_ShaderResource | BUF_Static,
			CreateInfo
		);

		void* Dest = RHICmdList.LockBuffer(GNullBuffer, 0, NumBytes, RLM_WriteOnly);
		uint32 Zero = 0;
		FMemory::Memcpy(Dest, &Zero, sizeof(uint32));
		RHICmdList.UnlockBuffer(GNullBuffer);

		// Vue STRUCTURÉE (pas typée) -> OK pour StructuredBuffer<T> dans l’USF
		GNullSRV = RHICmdList.CreateShaderResourceView(GNullBuffer);
	}

	// 2) Helpers pour piocher une SRV valide ou fallback
	auto PickSRV = [&](const FShaderResourceViewRHIRef& SRV) -> FShaderResourceViewRHIRef
		{
			return SRV.IsValid() ? SRV : GNullSRV;
		};

	// 3) Assigner TOUJOURS des SRV non-null aux paramètres shader
	Params->Points = GNullSRV;
	Params->Conn = GNullSRV;
	Params->Offs = GNullSRV;
	Params->Types = GNullSRV;
	Params->FeatureVals = GNullSRV;
	Params->NumPoints = 0;
	Params->NumCells = 0;

	if (bHasGPUResBA)
	{
		Params->Points = PickSRV(GPUResBA.PointsSRV);
		Params->Conn = PickSRV(GPUResBA.ConnSRV);
		Params->Offs = PickSRV(GPUResBA.OffsSRV);
		Params->Types = PickSRV(GPUResBA.TypesSRV);
		Params->FeatureVals = PickSRV(GPUResBA.FeatureValsSRV);

		Params->NumPoints = GPUResBA.NumPoints;
		Params->NumCells = GPUResBA.NumCells;
	}

	// 4) Shader handle
	const FGlobalShaderMap* GSM = GetGlobalShaderMap(GMaxRHIFeatureLevel);
	TShaderMapRef<FVTURaymarchCS> CS(GSM);
	const FComputeShaderRHIRef ShaderRHI = CS.GetComputeShader();

	// 5) Group count (numthreads 8x8x1)
	const FIntVector GroupCount(
		FMath::DivideAndRoundUp(SizeXY.X, 8),
		FMath::DivideAndRoundUp(SizeXY.Y, 8),
		1
	);

	RDG_EVENT_SCOPE(GraphBuilder, "VTURaymarchCS");

	const FVector3f Cam = (FVector3f)View.ViewMatrices.GetViewOrigin();
	UE_LOG(LogTemp, Warning, TEXT("[Raymarch] Cam=(%.2f,%.2f,%.2f)  RT=%dx%d"), Cam.X, Cam.Y, Cam.Z, SizeXY.X, SizeXY.Y);
	UE_LOG(LogTemp, Warning, TEXT("[Raymarch] Bounds Min=(%.2f,%.2f,%.2f) Max=(%.2f,%.2f,%.2f)"), Params->UnionMinLS4.X, Params->UnionMinLS4.Y, Params->UnionMinLS4.Z, Params->UnionMaxLS4.X, Params->UnionMaxLS4.Y, Params->UnionMaxLS4.Z);
	UE_LOG(LogTemp, Warning, TEXT("[Raymarch] NumPoints=%u  NumCells=%u  (hasGPU=%d)"), Params->NumPoints, Params->NumCells, bHasGPUResBA ? 1 : 0);

	// 6) Pass compute : bind / dispatch / unbind
	GraphBuilder.AddPass(
		RDG_EVENT_NAME("VTURaymarchCS_Dispatch"),
		Params,
		ERDGPassFlags::Compute,
		[Params, CS, ShaderRHI, GroupCount](FRHIComputeCommandList& RHICmdList)
		{
			SetComputePipelineState(RHICmdList, ShaderRHI);
			// Debug log

			SetShaderParameters(
				RHICmdList,
				CS,                          // TShaderRef
				ShaderRHI.GetReference(),    // FRHIComputeShader*
				*Params
			);

			RHICmdList.DispatchComputeShader(GroupCount.X, GroupCount.Y, GroupCount.Z);

			// lifetime :
			UnsetShaderUAVs(
				RHICmdList,
				CS,
				ShaderRHI.GetReference()
			);
		}
	);

	// 7) Copie la texture interne -> RT externe 
	AddCopyTexturePass(GraphBuilder, RDGInternal, RDGOut);
}

#if ENGINE_MAJOR_VERSION == 5 && ENGINE_MINOR_VERSION >= 5
void FCustomSceneViewExtension::SubscribeToPostProcessingPass(
	EPostProcessingPass Pass, const FSceneView& View,
	FAfterPassCallbackDelegateArray& InOutPassCallbacks, bool bIsPassEnabled)
#else
void FCustomSceneViewExtension::SubscribeToPostProcessingPass(
	EPostProcessingPass Pass, FAfterPassCallbackDelegateArray& InOutPassCallbacks, bool bIsPassEnabled)
#endif
{
	// Intentionnellement vide
}

FScreenPassTexture FCustomSceneViewExtension::CustomPostProcessFunction(
	FRDGBuilder& GraphBuilder, const FSceneView& SceneView, const FPostProcessMaterialInputs& Inputs)
{
	return FScreenPassTexture();
}

void FCustomSceneViewExtension::SetVTUGPUResources(const FVTUGPUResourcesBA& In)
{
	GPUResBA = In;       // copie légère de FBufferRHIRef + counters
	bHasGPUResBA = (In.PointsSRV.IsValid() || In.ConnSRV.IsValid() || In.OffsSRV.IsValid() || In.TypesSRV.IsValid());
}

void FCustomSceneViewExtension::ClearVTUGPUResources_RT()
{
	GPUResBA = FVTUGPUResourcesBA(); // remet tous les refs à null + compteurs à 0
	bHasGPUResBA = false;
}