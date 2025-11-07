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
	RaymarchRT = NewObject<UTextureRenderTarget2D>(this, TEXT("FurnaceRaymarch_RT"));
	RaymarchRT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
	RaymarchRT->ClearColor = FLinearColor::Transparent;
	RaymarchRT->bAutoGenerateMips = false;
	RaymarchRT->InitAutoFormat(1024, 1024);
	RaymarchRT->UpdateResourceImmediate(true); // force la création de la ressource RHI

	// 2) Crée/branche la SceneViewExtension (elle ne modifie PAS le SceneColor)
	CustomSceneViewExtension = FSceneViewExtensions::NewExtension<FCustomSceneViewExtension>();

	// 3) passe l'UObject
	if (CustomSceneViewExtension.IsValid())
	{
		CustomSceneViewExtension->SetRaymarchTargetObject(RaymarchRT);
	}
}

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

void UCustomShaderSubsystem::SetRaymarchTargetRT(UTextureRenderTarget2D* RT)
{
    RaymarchRT = RT;
    if (CustomSceneViewExtension.IsValid())
        CustomSceneViewExtension->SetRaymarchTargetObject(RT);
}

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

void UCustomShaderSubsystem::UploadDummyVTU()
{
    if (!CustomSceneViewExtension.IsValid()) return;

    // ===== Récupérer la caméra (Game Thread) =====
    FVector CamLoc = FVector::ZeroVector;
    FVector CamFwd = FVector::ForwardVector;

    if (UWorld* W = GetWorld())
    {
        if (APlayerController* PC = UGameplayStatics::GetPlayerController(W, 0))
        {
            if (APlayerCameraManager* PCM = PC->PlayerCameraManager)
            {
                CamLoc = PCM->GetCameraLocation();
                CamFwd = PCM->GetCameraRotation().Vector();
            }
        }
    }

    // On fige ces valeurs pour la lambda render-thread
    const FVector3f CamLocF = (FVector3f)CamLoc;
    const FVector3f CamFwdF = (FVector3f)CamFwd;

    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;

    UE_LOG(LogTemp, Log, TEXT("VTU Dummy: Types[0]=%u  Offs[0]=%u  Conn[0..7]=%u,%u,%u,%u,%u,%u,%u,%u"), 12u, 8u, 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u);

    ENQUEUE_RENDER_COMMAND(UploadDummyVTU)(
        [Ext, CamLocF, CamFwdF](FRHICommandListImmediate& RHICmdList)
        {
            FVTUGPUResourcesBA Res;

            // === 1) TYPES (uint) ===
            {
                const uint32 Values[1] = { 12u }; // VTK_HEXAHEDRON
                const uint32 Stride = sizeof(uint32);
                const uint32 NumBytes = sizeof(Values);
                FRHIResourceCreateInfo Info(TEXT("FRM_Types"));
                FBufferRHIRef Buffer = RHICmdList.CreateStructuredBuffer(Stride, NumBytes, BUF_ShaderResource | BUF_Static, Info);
                void* Dest = RHICmdList.LockBuffer(Buffer, 0, NumBytes, RLM_WriteOnly);
                FMemory::Memcpy(Dest, Values, NumBytes);
                RHICmdList.UnlockBuffer(Buffer);
                Res.TypesSRV = RHICmdList.CreateShaderResourceView(Buffer);
                Res.NumCells = 1;
            }

            // === 2) OFFS (uint) ===
            {
                const uint32 Values[1] = { 8u }; // fin exclusive de la cellule 0
                const uint32 Stride = sizeof(uint32);
                const uint32 NumBytes = sizeof(Values);
                FRHIResourceCreateInfo Info(TEXT("FRM_Offs"));
                FBufferRHIRef Buffer = RHICmdList.CreateStructuredBuffer(Stride, NumBytes, BUF_ShaderResource | BUF_Static, Info);
                void* Dest = RHICmdList.LockBuffer(Buffer, 0, NumBytes, RLM_WriteOnly);
                FMemory::Memcpy(Dest, Values, NumBytes);
                RHICmdList.UnlockBuffer(Buffer);
                Res.OffsSRV = RHICmdList.CreateShaderResourceView(Buffer);
            }

            // === 3) CONN (uint[8]) ===
            {
                const uint32 Values[8] = { 0,1,2,3,4,5,6,7 };
                const uint32 Stride = sizeof(uint32);
                const uint32 NumBytes = sizeof(Values);
                FRHIResourceCreateInfo Info(TEXT("FRM_Conn"));
                FBufferRHIRef Buffer = RHICmdList.CreateStructuredBuffer(Stride, NumBytes, BUF_ShaderResource | BUF_Static, Info);
                void* Dest = RHICmdList.LockBuffer(Buffer, 0, NumBytes, RLM_WriteOnly);
                FMemory::Memcpy(Dest, Values, NumBytes);
                RHICmdList.UnlockBuffer(Buffer);
                Res.ConnSRV = RHICmdList.CreateShaderResourceView(Buffer);
            }

            // === 4) POINTS (float3[8]) : cube 100³ à 50 devant la caméra ===
            {
                const float h = 50.f;
                const FVector3f center = FVector3f(0.f, 0.f, 140.f);
                const FVector3f C = /*CamLocF + CamFwdF*/center /** h*/;

                // ordre VTK des 8 sommets (cohérent avec les faces utilisées en shader)
                FVector3f P[8] = {
                    C + FVector3f(-h, -h, -h), // 0
                    C + FVector3f(h, -h, -h), // 1
                    C + FVector3f(-h,  h, -h), // 2
                    C + FVector3f(h,  h, -h), // 3
                    C + FVector3f(-h, -h,  h), // 4
                    C + FVector3f(h, -h,  h), // 5
                    C + FVector3f(-h,  h,  h), // 6
                    C + FVector3f(h,  h,  h)  // 7
                };

                const uint32 Stride = sizeof(FVector3f);
                const uint32 NumBytes = sizeof(P);
                FRHIResourceCreateInfo Info(TEXT("FRM_Points"));
                FBufferRHIRef Buffer = RHICmdList.CreateStructuredBuffer(Stride, NumBytes, BUF_ShaderResource | BUF_Static, Info);
                void* Dest = RHICmdList.LockBuffer(Buffer, 0, NumBytes, RLM_WriteOnly);
                FMemory::Memcpy(Dest, P, NumBytes);
                RHICmdList.UnlockBuffer(Buffer);
                Res.PointsSRV = RHICmdList.CreateShaderResourceView(Buffer);
                Res.NumPoints = 8;
            }

            // (FeatureVals optionnel — pas nécessaire ici)

            if (Ext.IsValid())
            {
                Ext->SetVTUGPUResources(Res);
            }
        }
        );
}

static void MakeStructuredSRV_Upload(FRHICommandListImmediate& RHICmdList, const void* SrcData, uint32 NumBytes, uint32 Stride, FBufferRHIRef& OutBuffer, FShaderResourceViewRHIRef& OutSRV, const TCHAR* DebugName)
{
    FRHIResourceCreateInfo Info(DebugName);
    OutBuffer = RHICmdList.CreateStructuredBuffer(Stride, NumBytes, BUF_ShaderResource | BUF_Static, Info);
    void* Dest = RHICmdList.LockBuffer(OutBuffer, 0, NumBytes, RLM_WriteOnly);
    FMemory::Memcpy(Dest, SrcData, NumBytes);
    RHICmdList.UnlockBuffer(OutBuffer);
    OutSRV = RHICmdList.CreateShaderResourceView(OutBuffer);
}

// Upload d’une grille VTU vers le GPU (RT)
static void UploadVTU_ToGPU_RT(
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext,
    TArray<FVector3f>&& PtsWS, TArray<uint32>&& Conn, TArray<uint32>&& Offs, TArray<uint32>&& Types)
{
    ENQUEUE_RENDER_COMMAND(UploadVTUArraysRT)(
        [Ext,
        Pts = MoveTemp(PtsWS),
        ConnV = MoveTemp(Conn),
        OffsV = MoveTemp(Offs),
        TypesV = MoveTemp(Types)](FRHICommandListImmediate& RHICmdList)
        {
            FVTUGPUResourcesBA Res;

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

            Res.FeatureValsSRV = nullptr; // pas encore
            UE_LOG(LogTemp, Warning, TEXT("[VTU SRV] NumPoints=%u  NumCells=%u  SRV(P,C,O,T,F) valid=%d,%d,%d,%d,%d"),
                Res.NumPoints, Res.NumCells,
                Res.PointsSRV.IsValid(), Res.ConnSRV.IsValid(), Res.OffsSRV.IsValid(), Res.TypesSRV.IsValid(), Res.FeatureValsSRV.IsValid());

            if (Ext.IsValid())
            {
                Ext->SetVTUGPUResources(Res);
            }
        });
}

//void  UCustomShaderSubsystem::UploadVTUGrid(const FVTUGrid& Grid, float ScaleCm)
//{
//    if (!CustomSceneViewExtension.IsValid()) return;
//
//    UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Points=%d  Cells=%d  Conn=%d  Offs=%d  Types=%d"),
//        Grid.Points.Num(), Grid.NumCells(), Grid.Connectivity.Num(), Grid.Offsets.Num(), Grid.Types.Num());
//
//    if (Grid.NumCells() > 0)
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
//    // convert
//    TArray<FVector3f> PtsWS;  PtsWS.Reserve(Grid.Points.Num());
//    for (const FVector3f& p : Grid.Points) PtsWS.Add(p * ScaleCm);
//
//    TArray<uint32> Conn; Conn.Reserve(Grid.Connectivity.Num());
//    for (int32 v : Grid.Connectivity) Conn.Add((uint32)v);
//
//    TArray<uint32> Offs; Offs.Reserve(Grid.Offsets.Num());
//    for (int32 v : Grid.Offsets) Offs.Add((uint32)v);
//
//    TArray<uint32> Ty;   Ty.Reserve(Grid.Types.Num());
//    for (uint8 v : Grid.Types)   Ty.Add((uint32)v);
//
//    // push bounds monde (scaled)
//    SetRaymarchBoundsWS((FVector)(Grid.Bounds.Min * ScaleCm), (FVector)(Grid.Bounds.Max * ScaleCm));
//
//    UploadVTU_ToGPU_RT(CustomSceneViewExtension, MoveTemp(PtsWS), MoveTemp(Conn), MoveTemp(Offs), MoveTemp(Ty));
//}

bool UCustomShaderSubsystem::UploadVTUGrid(const FVTUGrid& Grid, float ScaleCm)
{
    if (!CustomSceneViewExtension.IsValid())
    {
        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] ViewExtension invalid."));
        return false;
    }

    // --- Sanity checks CPU-side ---
    const int32 NumPts = Grid.Points.Num();
    const int32 NumCells = Grid.NumCells();
    const int32 NumConn = Grid.Connectivity.Num();
    const int32 NumOffs = Grid.Offsets.Num();
    const int32 NumTypes = Grid.Types.Num();

    UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Points=%d  Cells=%d  Conn=%d  Offs=%d  Types=%d"),
        NumPts, NumCells, NumConn, NumOffs, NumTypes);

    // Conditions minimales pour HEX dummy: au moins 1 cellule, Offs aligné à Types, Offs[last] <= Conn.Num()
    if (NumPts <= 0 || NumCells <= 0 || NumOffs != NumTypes)
    {
        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Invalid sizes (need Points>0, Cells>0, Offs==Types)."));
        return false;
    }
    if (NumOffs > 0 && Grid.Offsets.Last() > NumConn)
    {
        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Offs[last]=%d > Conn=%d (invalid)."), Grid.Offsets.Last(), NumConn);
        return false;
    }

    if (NumCells > 0)
    {
        const int32 end0 = Grid.Offsets[0];
        const int32 start0 = 0;
        const int32 n0 = end0 - start0;
        const uint8 ty0 = Grid.Types[0];

        FString firstConn;
        const int32 show = FMath::Min(8, end0);
        for (int32 k = 0; k < show; ++k)
        {
            firstConn += FString::Printf(TEXT("%d%s"), Grid.Connectivity[k], (k + 1 < show ? TEXT(",") : TEXT("")));
        }

        UE_LOG(LogTemp, Warning, TEXT("[VTU Upload] Types[0]=%u  n0=%d  Conn[0..%d]={%s}"),
            (uint32)ty0, n0, show - 1, *firstConn);
    }

    // --- Convert CPU arrays (WORLD cm, avec scale) ---
    TArray<FVector3f> PtsWS;  PtsWS.Reserve(NumPts);
    for (const FVector3f& p : Grid.Points)
    {
        PtsWS.Add(p * ScaleCm);
    }

    TArray<uint32> ConnV; ConnV.Reserve(NumConn);
    for (int32 v : Grid.Connectivity) ConnV.Add((uint32)v);

    TArray<uint32> OffsV; OffsV.Reserve(NumOffs);
    for (int32 v : Grid.Offsets)     OffsV.Add((uint32)v);

    TArray<uint32> TypesV; TypesV.Reserve(NumTypes);
    for (uint8 v : Grid.Types)       TypesV.Add((uint32)v);

    // --- Push bounds monde (scaled) pour la box globale de debug ---
    SetRaymarchBoundsWS((FVector)(Grid.Bounds.Min * ScaleCm), (FVector)(Grid.Bounds.Max * ScaleCm));

    // --- Enqueue sur le Render Thread la création des SRV ---
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;

    // On move les gros tableaux dans le lambda (mutable pour re-move à l’appel helper)
    ENQUEUE_RENDER_COMMAND(UploadVTU)(
        [Ext,
        PtsWS = MoveTemp(PtsWS),
        ConnV = MoveTemp(ConnV),
        OffsV = MoveTemp(OffsV),
        TypesV = MoveTemp(TypesV)]
        (FRHICommandListImmediate& RHICmdList) mutable
        {
            // Cette fonction fait la création des buffers/SRV et appelle SetVTUGPUResources côté ViewExtension.
            UploadVTU_ToGPU_RT(Ext, MoveTemp(PtsWS), MoveTemp(ConnV), MoveTemp(OffsV), MoveTemp(TypesV));
        });

    // Non bloquant : on renvoie "true" = upload correctement ENQUÊTÉ.
    return true;
}

//void UCustomShaderSubsystem::SetDebugBoundsWS(const FVector& MinWS, const FVector& MaxWS)
//{
//    if (!CustomSceneViewExtension.IsValid()) return;
//    // on pousse sur le RT de façon threadsafe
//    const FVector3f MinF = (FVector3f)MinWS;
//    const FVector3f MaxF = (FVector3f)MaxWS;
//    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
//    ENQUEUE_RENDER_COMMAND(SetDbgBounds)(
//        [Ext, MinF, MaxF](FRHICommandListImmediate&)
//        {
//            if (Ext.IsValid()) { Ext->SetDebugBoundsWS(MinF, MaxF); }
//        });
//}

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

void UCustomShaderSubsystem::SetRaymarchBoundsWS(const FVector& MinWS, const FVector& MaxWS)
{
    if (!CustomSceneViewExtension.IsValid()) return;
    const FVector3f MinF = (FVector3f)MinWS;
    const FVector3f MaxF = (FVector3f)MaxWS;
    TSharedPtr<FCustomSceneViewExtension, ESPMode::ThreadSafe> Ext = CustomSceneViewExtension;
    ENQUEUE_RENDER_COMMAND(SetDbg)(
        [Ext, MinF, MaxF](FRHICommandListImmediate&)
        {
            if (Ext.IsValid()) { Ext->SetDebugBoundsWS_RT(MinF, MaxF); }
        });
}