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
// Extension enregistrée
FCustomSceneViewExtension::FCustomSceneViewExtension(const FAutoRegister& AutoRegister)
	: FSceneViewExtensionBase(AutoRegister)
{
}

// Stocke uniquement l’UObject de la RT (côté GT). La ressource RHI est récupérée plus tard sur le Render Thread (dans PrePostProcessPass_RenderThread)
void FCustomSceneViewExtension::SetRaymarchTargetObject(UTextureRenderTarget2D* InRT)
{
	// Stocke juste l’objet ; on touchera la ressource côté Render Thread
	RaymarchRT_Object = InRT;
}

// ------------------------------------------------------------------
// post-process : on prépare/dispatch le compute et on copie vers la RT.
// C'est exécuté sur le Render Thread, dans le Render Graph.
// ------------------------------------------------------------------
void FCustomSceneViewExtension::PrePostProcessPass_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View, const FPostProcessingInputs& Inputs)
{
	// 0) Récupère l’objet RT et sa ressource (sur le Render Thread)
	UTextureRenderTarget2D* RTObj = RaymarchRT_Object.Get();
	if (!RTObj)
	{
		return; // pas de cible = on ne fait rien ce frame
	}

	FTextureRenderTargetResource* RTRes = RTObj->GetRenderTargetResource();
	if (!RTRes || !RTRes->IsInitialized())
	{
		return; // RT non initialisée
	}

	FTextureRHIRef RHITexture = RTRes->GetRenderTargetTexture();
	if (!RHITexture.IsValid())
	{
		return; // ressource RHI absente
	}

	// 1) Expose la RT externe au Render Graph (destination du copy)
	FRDGTextureRef RDGOut = GraphBuilder.RegisterExternalTexture(
		CreateRenderTarget(RHITexture, TEXT("FurnaceRaymarch_RT"))
	);

	const FIntPoint SizeXY = RTRes->GetSizeXY();

	// 2) Crée une texture RDG INTERNE avec flag UAV
	//    PF_FloatRGBA correspond à RTF_RGBA16f.
	FRDGTextureDesc Desc = FRDGTextureDesc::Create2D(
		SizeXY,                        // dimensions
		PF_FloatRGBA,                  // format (float16 RGBA)
		FClearValueBinding::Transparent,
		TexCreate_ShaderResource | TexCreate_UAV   // nécessaire pour UAV
	);
	FRDGTextureRef RDGInternal = GraphBuilder.CreateTexture(Desc, TEXT("FurnaceRaymarch_Internal"));

	// 3) Alloue et remplit la structure de paramètres du compute
	FVTURaymarchCS::FParameters* Params = GraphBuilder.AllocParameters<FVTURaymarchCS::FParameters>();
	// --- Sortie UAV : on bind l’UAV de la texture interne, PAS la RT externe.
	Params->OutTex = GraphBuilder.CreateUAV(RDGInternal);

	//==========================================================================================================
	// Taille écran/RT
	Params->RTSize = FUintVector2((uint32)SizeXY.X, (uint32)SizeXY.Y);
	Params->RTWidth = (uint32)SizeXY.X;
	Params->RTHeight = (uint32)SizeXY.Y;

	// --- Matrices caméra : inverse ViewProj pour reconstruire rayons VS/WS si besoin.
	Params->InvViewProj = FMatrix44f(View.ViewMatrices.GetInvViewProjectionMatrix());

	// --- Position caméra en monde : pour encoder la profondeur relative en alpha.
	Params->CameraPosWS = (FVector3f)View.ViewMatrices.GetViewOrigin();

	// --- LOG InvViewProj / sanity matrices ---*******************************************************************
	//const FMatrix44f VP = FMatrix44f(View.ViewMatrices.GetViewProjectionMatrix());

	const FMatrix44f InvVP = Params->InvViewProj;
	const FVector3f  CamWS = Params->CameraPosWS;

	// Utilitaires
	auto MatrixHasFinite = [](const FMatrix44f& M)->bool
	{
		for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++)
			if (!FMath::IsFinite(M.M[r][c])) return false;
		return true;
	};

	auto MatToStr = [](const FMatrix44f& M)->FString
	{
		return FString::Printf(
			TEXT("| %+ .6f %+ .6f %+ .6f %+ .6f |\n")
			TEXT("| %+ .6f %+ .6f %+ .6f %+ .6f |\n")
			TEXT("| %+ .6f %+ .6f %+ .6f %+ .6f |\n")
			TEXT("| %+ .6f %+ .6f %+ .6f %+ .6f |"),
			M.M[0][0], M.M[0][1], M.M[0][2], M.M[0][3],
			M.M[1][0], M.M[1][1], M.M[1][2], M.M[1][3],
			M.M[2][0], M.M[2][1], M.M[2][2], M.M[2][3],
			M.M[3][0], M.M[3][1], M.M[3][2], M.M[3][3]);
	};

	// Logs
	static uint64 LastFrameLogged = 0;
	if (GFrameCounter - LastFrameLogged > 60) // ~1s @60fps
	{
		LastFrameLogged = GFrameCounter;

		UE_LOG(LogTemp, Display, TEXT("[VTU] InvViewProj finite=%d det=%.6g"),
			MatrixHasFinite(InvVP) ? 1 : 0,
			(double)InvVP.Determinant());

		UE_LOG(LogTemp, Display, TEXT("[VTU] CameraPosWS = (%.6f, %.6f, %.6f)"),
			(double)CamWS.X, (double)CamWS.Y, (double)CamWS.Z);

		UE_LOG(LogTemp, Display, TEXT("[VTU] InvViewProj:\n%s"), *MatToStr(InvVP));
	}
	// --- LOG InvViewProj / sanity matrices ---********************************************************************

	// --- Clip plane (désactivé par défaut, champs padding à 0).
	Params->ClipPlaneWS = FVector4f(0, 0, 0, 0);
	Params->UseClip = 0;
	Params->Pad0 = Params->Pad1 = Params->Pad2 = 0;

	// --- WorldToLocal du volume (défini via setters ; sinon identité).
	//     Sert à passer de WS -> LS (espace local du proxy box pour raymarch).
	Params->WorldToLocal = bHasWorldToLocal ? WorldToLocal_RT : FMatrix44f::Identity;

	// --- AABB locale d’union (par ex. union des clones ou le 1/8 de base)
	//     Si non fourni, fallback sur petit cube [-1..1] en LS.
	if (bHasBoundsLS)
	{
		Params->UnionMinLS4 = FVector4f(BoundsMinLS, 0.f);
		Params->UnionMaxLS4 = FVector4f(BoundsMaxLS, 0.f);
	}
	else
	{
		//// mini fallback local (petit cube au centre local)
		Params->UnionMinLS4 = FVector4f(-1, -1, -1, 0);
		Params->UnionMaxLS4 = FVector4f(1, 1, 1, 0);
	}


	// ================================================= SRV / FALLBACK ==================================================
	// 1) SRV de repli (buffer structuré 4 octets) : alloué une seule fois.
	//    Permet d’éviter d’avoir des SRV null quand certaines ressources ne sont pas encore uploadées.
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

	// 2) Helper pour choisir entre SRV valide et fallback
	auto PickSRV = [&](const FShaderResourceViewRHIRef& SRV) -> FShaderResourceViewRHIRef
	{
		return SRV.IsValid() ? SRV : GNullSRV;
	};

	// ==== Initialisation par défaut (fallback) pour tous les SRV ====
	// Grille VTU
	Params->Points = GNullSRV;
	Params->Conn = GNullSRV;
	Params->Offs = GNullSRV;
	Params->Types = GNullSRV;
	Params->FeatureVals = GNullSRV;
	Params->NumPoints = 0;
	Params->NumCells = 0;

	// ==== Octree & éléments ====
	Params->OctNodeCenter = GNullSRV;
	Params->OctNodeExtent = GNullSRV;
	Params->OctNodeFirstChild = GNullSRV;
	Params->OctNodeChildCount = GNullSRV;
	Params->OctNodeFirstElem = GNullSRV;
	Params->OctNodeElemCount = GNullSRV;

	Params->NodeChildIndex = GNullSRV;

	Params->ElemMin = GNullSRV;
	Params->ElemMax = GNullSRV;
	Params->ElemCell = GNullSRV;

	Params->NumNodes = 0;
	Params->NumElems = 0;
	Params->RootIndex = 0;

	// Shading/feature par défaut
	Params->UseFeatureVals = 0;
	Params->VMin = 0.55f;
	Params->VMax = 1.2f;
	Params->EpsCm = 0.15f;

	// Si on a reçu un "resource patch" (GPUResBA) auparavant, on remplace les SRV/compteurs par les vrais.
	if (bHasGPUResBA)
	{
		// VTU
		Params->Points = PickSRV(GPUResBA.PointsSRV);
		Params->Conn = PickSRV(GPUResBA.ConnSRV);
		Params->Offs = PickSRV(GPUResBA.OffsSRV);
		Params->Types = PickSRV(GPUResBA.TypesSRV);
		Params->FeatureVals = PickSRV(GPUResBA.FeatureValsSRV);
		Params->Faces = PickSRV(GPUResBA.FacesSRV);
		Params->FaceOffsets = PickSRV(GPUResBA.FaceOffsetsSRV);

		Params->NumPoints = GPUResBA.NumPoints;
		Params->NumCells = GPUResBA.NumCells;

		// Octree
		Params->OctNodeCenter = PickSRV(GPUResBA.OctNodeCenterSRV);
		Params->OctNodeExtent = PickSRV(GPUResBA.OctNodeExtentSRV);
		Params->OctNodeFirstChild = PickSRV(GPUResBA.OctNodeFirstChildSRV);
		Params->OctNodeChildCount = PickSRV(GPUResBA.OctNodeChildCountSRV);
		Params->OctNodeFirstElem = PickSRV(GPUResBA.OctNodeFirstElemSRV);
		Params->OctNodeElemCount = PickSRV(GPUResBA.OctNodeElemCountSRV);

		Params->NodeChildIndex = PickSRV(GPUResBA.NodeChildIndexSRV);

		Params->ElemMin = PickSRV(GPUResBA.ElemMinSRV);
		Params->ElemMax = PickSRV(GPUResBA.ElemMaxSRV);
		Params->ElemCell = PickSRV(GPUResBA.ElemCellSRV);

		Params->NumNodes = GPUResBA.NumNodes;
		Params->NumElems = GPUResBA.NumElems;
		Params->RootIndex = GPUResBA.RootIndex;

		// Modes/debug/échelle valeurs
		Params->DebugDrawMode = DebugDrawMode_RT;
		Params->UseFeatureVals = GPUResBA.UseFeatureVals;
		Params->VMin = GPUResBA.VMin;
		Params->VMax = GPUResBA.VMax;
		Params->EpsCm = GPUResBA.EpsCm;

		//// logs de contrôle
		//UE_LOG(LogTemp, Warning, TEXT("[Raymarch]  Octree: Nodes=%u Elems=%u Root=%u  (hasGPU=%d)"), Params->NumNodes, Params->NumElems, Params->RootIndex, bHasGPUResBA ? 1 : 0);
		//UE_LOG(LogTemp, Warning, TEXT("[Raymarch] UseFeat=%u FeatSRV=%d VMin=%.3f VMax=%.3f Eps=%.3f"), GPUResBA.UseFeatureVals, GPUResBA.FeatureValsSRV.IsValid() ? 1 : 0, GPUResBA.VMin, GPUResBA.VMax, GPUResBA.EpsCm);
	}

	// 4) Récupère le shader compute global
	const FGlobalShaderMap* GSM = GetGlobalShaderMap(GMaxRHIFeatureLevel);
	TShaderMapRef<FVTURaymarchCS> CS(GSM);
	const FComputeShaderRHIRef ShaderRHI = CS.GetComputeShader();

	// 5) Taille de dispatch (numthreads 8x8x1 dans l’USF) -> nb de groupes
	const FIntVector GroupCount( FMath::DivideAndRoundUp(SizeXY.X, 8), FMath::DivideAndRoundUp(SizeXY.Y, 8), 1);

	RDG_EVENT_SCOPE(GraphBuilder, "VTURaymarchCS");

	const FVector3f Cam = (FVector3f)View.ViewMatrices.GetViewOrigin();
	//UE_LOG(LogTemp, Warning, TEXT("[Raymarch] CamWS=(%.2f,%.2f,%.2f)  RT=%dx%d"), Cam.X, Cam.Y, Cam.Z, SizeXY.X, SizeXY.Y);
	//UE_LOG(LogTemp, Warning, TEXT("[Raymarch] BoundsLS Min=(%.2f,%.2f,%.2f) Max=(%.2f,%.2f,%.2f)"), Params->UnionMinLS4.X, Params->UnionMinLS4.Y, Params->UnionMinLS4.Z, Params->UnionMaxLS4.X, Params->UnionMaxLS4.Y, Params->UnionMaxLS4.Z);
	//UE_LOG(LogTemp, Warning, TEXT("[Raymarch] NumPoints=%u  NumCells=%u  (hasGPU=%d)"), Params->NumPoints, Params->NumCells, bHasGPUResBA ? 1 : 0);

	// 6) Ajoute le pass compute au RDG : bind des paramètres, dispatch, unbind des UAV
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
				*Params						// bind struct params -> root signature
			);

			RHICmdList.DispatchComputeShader(GroupCount.X, GroupCount.Y, GroupCount.Z);

			// Nettoyage des UAV bindés 
			UnsetShaderUAVs(
				RHICmdList,
				CS,
				ShaderRHI.GetReference()
			);
		}
	);

	// 7) Copie la texture interne (écrite par le compute) vers la RT externe fournie par le gameplay.
	AddCopyTexturePass(GraphBuilder, RDGInternal, RDGOut);
}

// Extension interface optionnelle : ici, pas d’override de post-process matériel
FScreenPassTexture FCustomSceneViewExtension::CustomPostProcessFunction(FRDGBuilder& GraphBuilder, const FSceneView& SceneView, const FPostProcessMaterialInputs& Inputs)
{
	return FScreenPassTexture();
}

// ------------------------------------------------------------------
// Réception / fusion des "resource patches" (SRV/compteurs/params) côté RT.
// Pousser des morceaux de ressources (grille seule, octree seul, features seuls), et on merge dans un stockage persistant GPUResBA.
// ------------------------------------------------------------------
void FCustomSceneViewExtension::SetVTUGPUResources(const FVTUGPUResourcesBA& In)
{
	// helpers de merge : si le nouveau SRV/Buffer est valide, on remplace l’actuel
	auto PickSRV = [](const FShaderResourceViewRHIRef& NewSRV, FShaderResourceViewRHIRef& CurSRV)
	{
		if (NewSRV.IsValid()) CurSRV = NewSRV;
	};
	auto PickBuf = [](const FBufferRHIRef& NewBuf, FBufferRHIRef& CurBuf)
	{
		if (NewBuf.IsValid()) CurBuf = NewBuf;
	};

	// flags pour savoir quel paquet on vient de recevoir
	const bool HasGridPack =
		In.PointsSRV.IsValid() || In.ConnSRV.IsValid() || In.OffsSRV.IsValid() ||
		In.TypesSRV.IsValid() || In.FacesSRV.IsValid() || In.FaceOffsetsSRV.IsValid();

	const bool HasFeatPack = In.FeatureValsSRV.IsValid();

	const bool HasOctPack =
		In.OctNodeCenterSRV.IsValid() || In.OctNodeExtentSRV.IsValid() ||
		In.OctNodeFirstChildSRV.IsValid() || In.OctNodeChildCountSRV.IsValid() ||
		In.OctNodeFirstElemSRV.IsValid() || In.OctNodeElemCountSRV.IsValid() ||
		In.NodeChildIndexSRV.IsValid() || In.ElemMinSRV.IsValid() ||
		In.ElemMaxSRV.IsValid() || In.ElemCellSRV.IsValid();

	// ---------------- GRID ---------------- (buffers + SRV + compteurs)
	PickBuf(In.PointsBuffer, GPUResBA.PointsBuffer);
	PickBuf(In.ConnBuffer, GPUResBA.ConnBuffer);
	PickBuf(In.OffsBuffer, GPUResBA.OffsBuffer);
	PickBuf(In.TypesBuffer, GPUResBA.TypesBuffer);
	PickBuf(In.FeatureValsBuffer, GPUResBA.FeatureValsBuffer);

	PickBuf(In.FacesBuffer, GPUResBA.FacesBuffer);
	PickBuf(In.FaceOffsetsBuffer, GPUResBA.FaceOffsetsBuffer);

	PickSRV(In.PointsSRV, GPUResBA.PointsSRV);
	PickSRV(In.ConnSRV, GPUResBA.ConnSRV);
	PickSRV(In.OffsSRV, GPUResBA.OffsSRV);
	PickSRV(In.TypesSRV, GPUResBA.TypesSRV);
	PickSRV(In.FeatureValsSRV, GPUResBA.FeatureValsSRV);

	PickSRV(In.FacesSRV, GPUResBA.FacesSRV);
	PickSRV(In.FaceOffsetsSRV, GPUResBA.FaceOffsetsSRV);

	if (HasGridPack)
	{
		GPUResBA.NumPoints = In.NumPoints; // copie directe, 0 autorisé
		GPUResBA.NumCells = In.NumCells;
	}

	// ---------------- OCTREE ---------------- (buffers + SRV + compteurs)
	PickBuf(In.OctNodeCenterBuffer, GPUResBA.OctNodeCenterBuffer);
	PickBuf(In.OctNodeExtentBuffer, GPUResBA.OctNodeExtentBuffer);
	PickBuf(In.OctNodeFirstChildBuffer, GPUResBA.OctNodeFirstChildBuffer);
	PickBuf(In.OctNodeChildCountBuffer, GPUResBA.OctNodeChildCountBuffer);
	PickBuf(In.OctNodeFirstElemBuffer, GPUResBA.OctNodeFirstElemBuffer);
	PickBuf(In.OctNodeElemCountBuffer, GPUResBA.OctNodeElemCountBuffer);
	PickBuf(In.NodeChildIndexBuffer, GPUResBA.NodeChildIndexBuffer);

	PickBuf(In.ElemMinBuffer, GPUResBA.ElemMinBuffer);
	PickBuf(In.ElemMaxBuffer, GPUResBA.ElemMaxBuffer);
	PickBuf(In.ElemCellBuffer, GPUResBA.ElemCellBuffer);

	PickSRV(In.OctNodeCenterSRV, GPUResBA.OctNodeCenterSRV);
	PickSRV(In.OctNodeExtentSRV, GPUResBA.OctNodeExtentSRV);
	PickSRV(In.OctNodeFirstChildSRV, GPUResBA.OctNodeFirstChildSRV);
	PickSRV(In.OctNodeChildCountSRV, GPUResBA.OctNodeChildCountSRV);
	PickSRV(In.OctNodeFirstElemSRV, GPUResBA.OctNodeFirstElemSRV);
	PickSRV(In.OctNodeElemCountSRV, GPUResBA.OctNodeElemCountSRV);
	PickSRV(In.NodeChildIndexSRV, GPUResBA.NodeChildIndexSRV);

	PickSRV(In.ElemMinSRV, GPUResBA.ElemMinSRV);
	PickSRV(In.ElemMaxSRV, GPUResBA.ElemMaxSRV);
	PickSRV(In.ElemCellSRV, GPUResBA.ElemCellSRV);

	if (HasOctPack)
	{
		GPUResBA.NumNodes = In.NumNodes;     // 0 autorisé
		GPUResBA.NumElems = In.NumElems;
		GPUResBA.RootIndex = In.RootIndex;    //  0 autorisé
	}

	// ---------------- PARAMS additionnels ----------------
	// Fenêtre des couleurs / usage de features / epsilon géométrique
	UE_LOG(LogTemp, Warning, TEXT("[Raymarch]  SetVTUGPUResources : UseFeatureVals=%u VMin=%.3f VMax=%.3f Eps=%.3f"), In.UseFeatureVals, In.VMin, In.VMax, In.EpsCm);
	if (In.bSetUseFeatureVals) GPUResBA.UseFeatureVals = In.UseFeatureVals;
	if (In.bSetVMinMax) { GPUResBA.VMin = In.VMin; GPUResBA.VMax = In.VMax; }
	if (In.bSetEps)            GPUResBA.EpsCm = In.EpsCm;

	// Marqueur : on a désormais au moins un pack valide, le pass compute peut bind des SRV réels.
	bHasGPUResBA = true;
}

// ------------------------------------------------------------------
// Reset complet des ressources GPU (appelé sur RT)
// ------------------------------------------------------------------
void FCustomSceneViewExtension::ClearVTUGPUResources_RT()
{
	GPUResBA = FVTUGPUResourcesBA(); // remet tous les refs à null + compteurs à 0
	bHasGPUResBA = false;
}