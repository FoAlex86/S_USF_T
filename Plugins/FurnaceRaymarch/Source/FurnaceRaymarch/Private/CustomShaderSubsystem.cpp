// CustomShaderSubsystem.cpp
#include "CustomShaderSubsystem.h"
#include "SceneViewExtension.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Rendering/CustomSceneViewExtension.h"
#include "RHI.h"
#include "RHIResources.h"
#include "RenderResource.h"
#include "RHICommandList.h"

#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Camera/PlayerCameraManager.h"

void UCustomShaderSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// 1) Crée une RT (ex: 1024x1024 RGBA16f) pour recevoir le raymarch
    //    - format RTF_RGBA16f : bonne précision pour stocker des couleurs/valeurs,
    //    - Clear transparent (utile si alpha encode la profondeur ou pour blending),
    //    - pas de mips (lecture/écriture directe par le compute).
	RaymarchRT = NewObject<UTextureRenderTarget2D>(this, TEXT("FurnaceRaymarch_RT"));
	RaymarchRT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
	RaymarchRT->ClearColor = FLinearColor::Transparent;
	RaymarchRT->bAutoGenerateMips = false;
	RaymarchRT->InitAutoFormat(256, 256);
	RaymarchRT->UpdateResourceImmediate(true); // force la création de la ressource RHI

	// 2) Crée/branche la SceneViewExtension qui va au pipeline de rendu,
	CustomSceneViewExtension = FSceneViewExtensions::NewExtension<FCustomSceneViewExtension>();

	// 3) passe l'UObject à la ViewExtension pour qu'elle puisse binder l'UAV/outTex côté RDG lors de l'exécution du compute
	if (CustomSceneViewExtension.IsValid())
	{
		CustomSceneViewExtension->SetRaymarchTargetObject(RaymarchRT);
	}
}

// À la désinitialisation : s'assurer de libérer les ressources GPU possédées par la ViewExtension (buffers SRV/UAV, etc.).
void UCustomShaderSubsystem::Deinitialize()
{
    if (CustomSceneViewExtension.IsValid())
    {
        TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
        ENQUEUE_RENDER_COMMAND(ClearVTUEnd)(
            [Ext](FRHICommandListImmediate&)
            {
                if (Ext.IsValid()) { Ext->ClearVTUGPUResources_RT(); }
            });
    }
    Super::Deinitialize();
}

// Permet de remplacer dynamiquement la RT cible (taille différente, etc.).
void UCustomShaderSubsystem::SetRaymarchTargetRT(UTextureRenderTarget2D* RT)
{
    RaymarchRT = RT;
    if (CustomSceneViewExtension.IsValid())
        CustomSceneViewExtension->SetRaymarchTargetObject(RT);
}

// Demande explicite de purge des ressources VTU côté GPU (buffers SRV).
void UCustomShaderSubsystem::ClearVTUOnGPU()
{
    if (!CustomSceneViewExtension.IsValid()) return;
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
    ENQUEUE_RENDER_COMMAND(ClearVTU)(
        [Ext](FRHICommandListImmediate&)
        {
            if (Ext.IsValid())
            {
                Ext->ClearVTUGPUResources_RT();
            }
        });
}

//  crée un StructuredBuffer + SRV, copie CPU->GPU, et renvoie Buffer+SRV.
// - Stride : taille d'un élément (ex : sizeof(FVector3f))
// - NumBytes : taille totale à copier
// - BUF_ShaderResource | BUF_Static : accès shader, données statiques
static void MakeStructuredSRV_Upload(FRHICommandListImmediate& RHICmdList, const void* SrcData, uint32 NumBytes, uint32 Stride, FBufferRHIRef& OutBuffer, FShaderResourceViewRHIRef& OutSRV, const TCHAR* DebugName)
{
    FRHIResourceCreateInfo Info(DebugName);
    OutBuffer = RHICmdList.CreateStructuredBuffer(Stride, NumBytes, BUF_ShaderResource | BUF_Static, Info);
    void* Dest = RHICmdList.LockBuffer(OutBuffer, 0, NumBytes, RLM_WriteOnly);
    FMemory::Memcpy(Dest, SrcData, NumBytes);
    RHICmdList.UnlockBuffer(OutBuffer);
    OutSRV = RHICmdList.CreateShaderResourceView(OutBuffer);
}

// Upload d'un tableau de scalaires (par cellule) pour shading/colormap côté GPU.
bool UCustomShaderSubsystem::UploadFeatureVals(const TArray<float>& Values)
{
    if (!CustomSceneViewExtension.IsValid() || Values.Num() == 0) return false;

    TArray<float> Copy = Values;
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;

    // Création du StructuredBuffer + SRV côté RT, puis signalement à la ViewExtension.
    // On utilise un "Resource Patch" (FVTUGPUResourcesBA) : seul le champ non nul/flaggué
    // sera effectivement consommé par la ViewExtension (pattern de mise à jour partielle).
    ENQUEUE_RENDER_COMMAND(UploadFeatureVals_RT)(
        [Ext, Copy = MoveTemp(Copy)](FRHICommandListImmediate& RHICmdList)
        {
            if (!Ext.IsValid()) return;

            FVTUGPUResourcesBA Res; // on ne touche qu’au pack "features"
            MakeStructuredSRV_Upload(
                RHICmdList,
                Copy.GetData(), Copy.Num() * sizeof(float), sizeof(float),
                Res.FeatureValsBuffer, Res.FeatureValsSRV, TEXT("VTU_FeatureVals"));

            Res.UseFeatureVals = 1;                 // <- IMPORTANT active l'usage des scalar per-cell
            Res.NumCells = (uint32)Copy.Num(); // (utile pour logs)
			Res.bSetUseFeatureVals = true; 	  // indique que UseFeatureVals est "à appliquer"

            Ext->SetVTUGPUResources(Res);
        });

    return true;
}

// Upload d’une grille VTU vers le GPU (RT)
static void UploadVTU_ToGPU_RT( TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext, TArray<FVector3f>&& PtsWS, TArray<uint32>&& Conn, TArray<uint32>&& Offs, TArray<uint32>&& Types, TArray<int32>&& Faces, TArray<int32>&& FaceOffsets)
{
    ENQUEUE_RENDER_COMMAND(UploadVTUArraysRT)(
        [Ext, Pts = MoveTemp(PtsWS), ConnV = MoveTemp(Conn), OffsV = MoveTemp(Offs), TypesV = MoveTemp(Types), FacesV = MoveTemp(Faces), FaceOffV = MoveTemp(FaceOffsets)]
        (FRHICommandListImmediate& RHICmdList)
        {
            FVTUGPUResourcesBA Res;
            // Chaque tableau est transféré dans un StructuredBuffer + SRV.
            if (Pts.Num() > 0)
            {
                MakeStructuredSRV_Upload(RHICmdList, Pts.GetData(), Pts.Num() * sizeof(FVector3f), sizeof(FVector3f), Res.PointsBuffer, Res.PointsSRV, TEXT("VTU_Points"));
                Res.NumPoints = (uint32)Pts.Num();
            }
            if (ConnV.Num() > 0)
            {
                MakeStructuredSRV_Upload(RHICmdList,
                    ConnV.GetData(), ConnV.Num() * sizeof(uint32), sizeof(uint32),
                    Res.ConnBuffer, Res.ConnSRV, TEXT("VTU_Conn"));
            }
            if (OffsV.Num() > 0)
            {
                MakeStructuredSRV_Upload(RHICmdList,
                    OffsV.GetData(), OffsV.Num() * sizeof(uint32), sizeof(uint32),
                    Res.OffsBuffer, Res.OffsSRV, TEXT("VTU_Offs"));
            }
            if (TypesV.Num() > 0)
            {
                MakeStructuredSRV_Upload(RHICmdList,
                    TypesV.GetData(), TypesV.Num() * sizeof(uint32), sizeof(uint32),
                    Res.TypesBuffer, Res.TypesSRV, TEXT("VTU_Types"));
                Res.NumCells = (uint32)TypesV.Num();
            }

            // Données de polyhedra (faces / offsets) si présentes
            if (FacesV.Num() > 0)
                MakeStructuredSRV_Upload(RHICmdList, FacesV.GetData(), FacesV.Num() * sizeof(int32), sizeof(int32),
                    Res.FacesBuffer, Res.FacesSRV, TEXT("VTU_Faces"));
            if (FaceOffV.Num() > 0)
                MakeStructuredSRV_Upload(RHICmdList, FaceOffV.GetData(), FaceOffV.Num() * sizeof(int32), sizeof(int32),
                    Res.FaceOffsetsBuffer, Res.FaceOffsetsSRV, TEXT("VTU_FaceOffsets"));


            //Res.FeatureValsSRV = nullptr; // pas encore
            UE_LOG(LogTemp, Warning, TEXT("[VTU SRV] NumPoints=%u  NumCells=%u  SRV(P,C,O,T,F) valid=%d,%d,%d,%d,%d"),
                Res.NumPoints, Res.NumCells, Res.PointsSRV.IsValid(), Res.ConnSRV.IsValid(), Res.OffsSRV.IsValid(), Res.TypesSRV.IsValid(), Res.FeatureValsSRV.IsValid());

            UE_LOG(LogTemp, Warning, TEXT("[VTU SRV] SRV Faces=%d FaceOffs=%d"), Res.FacesSRV.IsValid(), Res.FaceOffsetsSRV.IsValid());

            // Patch appliqué côté RT : la ViewExtension garde les SRV et les bindera au compute.
            if (Ext.IsValid())
            {
                Ext->SetVTUGPUResources(Res);
            }
        });
}

// Convertit les données CPU (grille VTU) en buffers GPU :
// NB : les bounds et la WorldToLocal sont poussés séparément par AVTUViewer via PushProxyBoundsToGPU_FromRayVolumePMC() -> WorldToLocal + BoundsLS.
bool UCustomShaderSubsystem::UploadVTUGrid(const FVTUGrid& Grid, float ScaleCm)
{
    if (!CustomSceneViewExtension.IsValid()) return false;

    // Points LS (ScaleCm appliqué)
    TArray<FVector3f> PtsLS; PtsLS.Reserve(Grid.Points.Num());
    for (const FVector3f& p : Grid.Points) PtsLS.Add(p * ScaleCm);

    // Connexions/Offsets/Types convertis en uint32 pour StructuredBuffers
    TArray<uint32> ConnV;      ConnV.Reserve(Grid.Connectivity.Num());
    TArray<uint32> OffsV;      OffsV.Reserve(Grid.Offsets.Num());
    TArray<uint32> TypesV;     TypesV.Reserve(Grid.Types.Num());
    TArray<int32>  FacesV;     FacesV = Grid.Faces;         // déjà int32
    TArray<int32>  FaceOffsV;  FaceOffsV = Grid.FaceOffsets;

    for (int32 v : Grid.Connectivity) ConnV.Add((uint32)v);
    for (int32 v : Grid.Offsets)     OffsV.Add((uint32)v);
    for (uint8 v : Grid.Types)      TypesV.Add((uint32)v);

    auto Ext = CustomSceneViewExtension;
    UploadVTU_ToGPU_RT(Ext, MoveTemp(PtsLS), MoveTemp(ConnV), MoveTemp(OffsV), MoveTemp(TypesV),
        MoveTemp(FacesV), MoveTemp(FaceOffsV));
    return true;
}

// Upload de l’octree CPU -> buffers GPU (SoA) + SRV, puis signalement à la ViewExtension.
bool UCustomShaderSubsystem::UploadVTUOctree(const FVTUCellOctree& Octree)
{
    UE_LOG(LogTemp, Warning, TEXT("UploadVTUOctree BEGIN"));
    if (!CustomSceneViewExtension.IsValid())
        return false;

    // 1) "Flatten" CPU : transforme la structure d’octree hiérarchique en tableaux linéaires (Structure of Arrays) adaptés aux threads GPU.
    TArray<FVector3f> NodeC, NodeE;
    TArray<int32>     NodeFirstChild, NodeFirstElem, ChildIndex, ElemCell;
    TArray<uint32>    NodeChildCount, NodeElemCount;
    TArray<FVector3f> ElemMin, ElemMax;
    uint32 RootIndex = 0;

    if (!Octree.FlattenForGPU(NodeC, NodeE, NodeFirstChild, NodeChildCount,
        NodeFirstElem, NodeElemCount, ChildIndex,
        ElemMin, ElemMax, ElemCell, RootIndex))
    {
        UE_LOG(LogTemp, Warning, TEXT("[Octree Upload] FlattenForGPU FAILED."));
        return false;
    }
    UE_LOG(LogTemp, Warning, TEXT("UploadVTUOctree flatten done  (Nodes=%d, Elems=%d, Root=%u)"),
        NodeC.Num(), ElemCell.Num(), RootIndex);

    // 2) Création des StructuredBuffers + SRV sur RT et envoi à la ViewExtension
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;

    ENQUEUE_RENDER_COMMAND(UploadVTUOctree_RT)(
    [Ext,
    NodeC = MoveTemp(NodeC),
    NodeE = MoveTemp(NodeE),
    NodeFirstChild = MoveTemp(NodeFirstChild),
    NodeChildCount = MoveTemp(NodeChildCount),
    NodeFirstElem = MoveTemp(NodeFirstElem),
    NodeElemCount = MoveTemp(NodeElemCount),
    ChildIndex = MoveTemp(ChildIndex),
    ElemMin = MoveTemp(ElemMin),
    ElemMax = MoveTemp(ElemMax),
    ElemCell = MoveTemp(ElemCell),
    RootIndex](FRHICommandListImmediate& RHICmdList)
    {
        if (!Ext.IsValid()) return;

        FVTUGPUResourcesBA Res; // rempli côté RT

        // --- Nodes ---
        if (NodeC.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                NodeC.GetData(), NodeC.Num() * sizeof(FVector3f), sizeof(FVector3f),
                Res.OctNodeCenterBuffer, Res.OctNodeCenterSRV, TEXT("Oct_NodeCenter"));

        if (NodeE.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                NodeE.GetData(), NodeE.Num() * sizeof(FVector3f), sizeof(FVector3f),
                Res.OctNodeExtentBuffer, Res.OctNodeExtentSRV, TEXT("Oct_NodeExtent"));

        if (NodeFirstChild.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                NodeFirstChild.GetData(), NodeFirstChild.Num() * sizeof(int32), sizeof(int32),
                Res.OctNodeFirstChildBuffer, Res.OctNodeFirstChildSRV, TEXT("Oct_NodeFirstChild"));

        if (NodeChildCount.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                NodeChildCount.GetData(), NodeChildCount.Num() * sizeof(uint32), sizeof(uint32),
                Res.OctNodeChildCountBuffer, Res.OctNodeChildCountSRV, TEXT("Oct_NodeChildCount"));

        if (NodeFirstElem.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                NodeFirstElem.GetData(), NodeFirstElem.Num() * sizeof(int32), sizeof(int32),
                Res.OctNodeFirstElemBuffer, Res.OctNodeFirstElemSRV, TEXT("Oct_NodeFirstElem"));

        if (NodeElemCount.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                NodeElemCount.GetData(), NodeElemCount.Num() * sizeof(uint32), sizeof(uint32),
                Res.OctNodeElemCountBuffer, Res.OctNodeElemCountSRV, TEXT("Oct_NodeElemCount"));

        if (ChildIndex.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                ChildIndex.GetData(), ChildIndex.Num() * sizeof(int32), sizeof(int32),
                Res.NodeChildIndexBuffer, Res.NodeChildIndexSRV, TEXT("Oct_NodeChildIndex"));

        // --- Elems ---
        if (ElemMin.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                ElemMin.GetData(), ElemMin.Num() * sizeof(FVector3f), sizeof(FVector3f),
                Res.ElemMinBuffer, Res.ElemMinSRV, TEXT("Oct_ElemMin"));

        if (ElemMax.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                ElemMax.GetData(), ElemMax.Num() * sizeof(FVector3f), sizeof(FVector3f),
                Res.ElemMaxBuffer, Res.ElemMaxSRV, TEXT("Oct_ElemMax"));

        if (ElemCell.Num() > 0)
            MakeStructuredSRV_Upload(RHICmdList,
                ElemCell.GetData(), ElemCell.Num() * sizeof(int32), sizeof(int32),
                Res.ElemCellBuffer, Res.ElemCellSRV, TEXT("Oct_ElemCell"));

        // Compteurs rattachés au patch (consommés côté ViewExtension)
        Res.NumNodes = (uint32)NodeC.Num();
        Res.NumElems = (uint32)ElemCell.Num();
        Res.RootIndex = RootIndex;

        // Valeurs par défaut pour le shading, si aucun FeatureVals n'est chargé
        Res.UseFeatureVals = 0;
        Res.VMin = 0.f; Res.VMax = 1.f;

        // Application du patch côté RT : la ViewExtension conservera ces SRV.
        Ext->SetVTUGPUResources(Res); // appel côté RT

        UE_LOG(LogTemp, Warning, TEXT("[Octree Upload RT] Nodes=%u  Elems=%u  Root=%u"),
            Res.NumNodes, Res.NumElems, Res.RootIndex);
    });

    return true;
}

// Pousse la matrice monde->local du volume (inverse d'ActorToWorld) vers la ViewExtension, qui la mettra dans les paramètres du compute (WorldToLocal).
void UCustomShaderSubsystem::SetRaymarchVolumeTransform(const FTransform& ActorToWorld)
{
    if (!CustomSceneViewExtension.IsValid()) return;
    const FMatrix44f WL = FMatrix44f(ActorToWorld.ToInverseMatrixWithScale()); // monde->local volume
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
    ENQUEUE_RENDER_COMMAND(SetWL)(
        [Ext, WL](FRHICommandListImmediate&)
        {
            if (Ext.IsValid()) { Ext->SetWorldToLocal_RT(WL); }
        });
}

// Pousse les bornes (AABB) en Local Space du volume : utilisée par le compute pour limiter le raymarch et calculer la boîte de travail.
void UCustomShaderSubsystem::SetRaymarchBoundsLS(const FVector& MinLS, const FVector& MaxLS)
{
    if (!CustomSceneViewExtension.IsValid()) return;
    const FVector3f MinF = (FVector3f)MinLS;
    const FVector3f MaxF = (FVector3f)MaxLS;
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
    ENQUEUE_RENDER_COMMAND(SetDbgLS)(
        [Ext, MinF, MaxF](FRHICommandListImmediate&)
        {
            if (Ext.IsValid()) { Ext->SetBoundsLS_RT(MinF, MaxF); }
        });
}

// Version GT qui accepte une FMatrix (double) et la convertit en FMatrix44f.
void UCustomShaderSubsystem::SetWorldToLocal(const FMatrix& W2L)
{
    if (!CustomSceneViewExtension.IsValid()) return;
    const FMatrix44f W2L44 = FMatrix44f(W2L);
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
    ENQUEUE_RENDER_COMMAND(SetW2L)(
        [Ext, W2L44](FRHICommandListImmediate&)
        {
            if (Ext.IsValid()) { Ext->SetWorldToLocal_RT(W2L44); }
        });
}

// Choix du mode de debug draw côté shader :
//   0 = off (raymarch standard)
//   1 = dessine seulement les noeuds d'octree
//   2 = dessine les AABB des éléments (cell proxies)
//   3 = réservé/étendu (selon implémentation shader)
void UCustomShaderSubsystem::SetDebugDrawMode(int32 Mode)
{
    DebugDrawMode_GT = FMath::Clamp(Mode, 0, 3);

    if (CustomSceneViewExtension.IsValid())
    {
        auto Ext = CustomSceneViewExtension;
        const uint32 M = (uint32)DebugDrawMode_GT;

        ENQUEUE_RENDER_COMMAND(SetDbgModeRT)(
            [Ext, M](FRHICommandListImmediate&)
            {
                if (Ext.IsValid())
                {
                    Ext->SetDebugDrawMode_RT(M);
                }
            }
            );

        UE_LOG(LogTemp, Log, TEXT("[VTU] DebugDrawMode set to %d"), DebugDrawMode_GT);
    }
}

// pour cycler les modes debug au runtime (bindé à une touche), plus utilisées pour le moment de mémoire
void UCustomShaderSubsystem::CycleDebugDrawMode()
{
    int32 Next = (DebugDrawMode_GT + 1) % 4;
    SetDebugDrawMode(Next);
}

// Définit la fenêtre de mapping (VMin/VMax) pour coloriser FeatureVals[cell]
void UCustomShaderSubsystem::SetFeatureWindow(float InVMin, float InVMax)
{
    if (!CustomSceneViewExtension.IsValid()) return;
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;

    ENQUEUE_RENDER_COMMAND(SetFeatureWin_RT)(
        [Ext, InVMin, InVMax](FRHICommandListImmediate&)
        {
            if (!Ext.IsValid()) return;
            FVTUGPUResourcesBA Res; Res.VMin = InVMin; Res.VMax = InVMax;
            Res.bSetVMinMax = true;
            Ext->SetVTUGPUResources(Res);
        });
}
// Définit l'epsilon géométrique (en cm) utilisé par les tests d'intersection
void UCustomShaderSubsystem::SetRaymarchEpsilon(float InEpsCm)
{
    if (!CustomSceneViewExtension.IsValid()) return;
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;

    ENQUEUE_RENDER_COMMAND(SetEps_RT)(
        [Ext, InEpsCm](FRHICommandListImmediate&)
        {
            if (!Ext.IsValid()) return;
            FVTUGPUResourcesBA Res; Res.EpsCm = InEpsCm;
            Res.bSetEps = true; // flag : mettre à jour Eps dans les params shader
            Ext->SetVTUGPUResources(Res);
        });
}

//bool UCustomShaderSubsystem::UploadVTUGrid(const FVTUGrid& Grid, float ScaleCm)
//{
//    if (!CustomSceneViewExtension.IsValid())
//    {
//        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] ViewExtension invalid."));
//        return false;
//    }
//
//    // --- Sanity checks CPU-side ---
//    const int32 NumPts = Grid.Points.Num();
//    const int32 NumCells = Grid.NumCells();
//    const int32 NumConn = Grid.Connectivity.Num();
//    const int32 NumOffs = Grid.Offsets.Num();
//    const int32 NumTypes = Grid.Types.Num();
//
//    UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Points=%d  Cells=%d  Conn=%d  Offs=%d  Types=%d"),
//        NumPts, NumCells, NumConn, NumOffs, NumTypes);
//
//    // Conditions minimales pour HEX dummy: au moins 1 cellule, Offs aligné à Types, Offs[last] <= Conn.Num()
//    if (NumPts <= 0 || NumCells <= 0 || NumOffs != NumTypes)
//    {
//        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Invalid sizes (need Points>0, Cells>0, Offs==Types)."));
//        return false;
//    }
//    if (NumOffs > 0 && Grid.Offsets.Last() > NumConn)
//    {
//        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Offs[last]=%d > Conn=%d (invalid)."), Grid.Offsets.Last(), NumConn);
//        return false;
//    }
//
//    if (NumCells > 0)
//    {
//        const int32 end0 = Grid.Offsets[0];
//        const int32 start0 = 0;
//        const int32 n0 = end0 - start0;
//        const uint8 ty0 = Grid.Types[0];
//
//        FString firstConn;
//        const int32 show = FMath::Min(8, end0);
//        for (int32 k = 0; k < show; ++k)
//        {
//            firstConn += FString::Printf(TEXT("%d%s"), Grid.Connectivity[k], (k + 1 < show ? TEXT(",") : TEXT("")));
//        }
//
//        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Types[0]=%u  n0=%d  Conn[0..%d]={%s}"),
//            (uint32)ty0, n0, show - 1, *firstConn);
//    }
//
//    // --- Convert CPU arrays (WORLD cm, avec scale) ---
//    TArray<FVector3f> PtsWS;  PtsWS.Reserve(NumPts);
//    for (const FVector3f& p : Grid.Points)
//    {
//        PtsWS.Add(p * ScaleCm);
//    }
//
//    TArray<uint32> ConnV; ConnV.Reserve(NumConn);
//    for (int32 v : Grid.Connectivity) ConnV.Add((uint32)v);
//
//    TArray<uint32> OffsV; OffsV.Reserve(NumOffs);
//    for (int32 v : Grid.Offsets)     OffsV.Add((uint32)v);
//
//    TArray<uint32> TypesV; TypesV.Reserve(NumTypes);
//    for (uint8 v : Grid.Types)       TypesV.Add((uint32)v);
//
//    // --- Push bounds monde (scaled) pour la box globale de debug ---
//    //SetRaymarchBoundsWS((FVector)(Grid.Bounds.Min * ScaleCm), (FVector)(Grid.Bounds.Max * ScaleCm));
//
//    // --- Enqueue sur le Render Thread la création des SRV ---
//    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
//
//    // On move les gros tableaux dans le lambda (mutable pour re-move à l’appel helper)
//    ENQUEUE_RENDER_COMMAND(UploadVTU)(
//        [Ext,
//        PtsWS = MoveTemp(PtsWS),
//        ConnV = MoveTemp(ConnV),
//        OffsV = MoveTemp(OffsV),
//        TypesV = MoveTemp(TypesV)]
//        (FRHICommandListImmediate& RHICmdList) mutable
//        {
//            // Cette fonction fait la création des buffers/SRV et appelle SetVTUGPUResources côté ViewExtension.
//            UploadVTU_ToGPU_RT(Ext, MoveTemp(PtsWS), MoveTemp(ConnV), MoveTemp(OffsV), MoveTemp(TypesV));
//        });
//
//    // Non bloquant : on renvoie "true" = upload correctement ENQUÊTÉ.
//    return true;
//}

//void UCustomShaderSubsystem::SetRaymarchBoundsWS(const FVector& MinWS, const FVector& MaxWS)
//{
//    if (!CustomSceneViewExtension.IsValid()) return;
//    const FVector3f MinF = (FVector3f)MinWS;
//    const FVector3f MaxF = (FVector3f)MaxWS;
//    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
//    ENQUEUE_RENDER_COMMAND(SetDbg)(
//        [Ext, MinF, MaxF](FRHICommandListImmediate&)
//        {
//            if (Ext.IsValid()) { Ext->SetDebugBoundsWS_RT(MinF, MaxF); }
//        });
//}