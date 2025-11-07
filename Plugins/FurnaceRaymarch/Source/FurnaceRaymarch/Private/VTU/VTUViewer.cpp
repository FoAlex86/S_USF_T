#include "VTU/VTUViewer.h"
#include "Misc/Paths.h"
#include "Misc/MessageDialog.h"
#include "Containers/Array.h"
#include "Misc/AssertionMacros.h"
#include "KismetProceduralMeshLibrary.h"
#include "ProceduralMeshComponent.h"
#include "CustomShaderSubsystem.h" // plugin
#include "Engine/GameInstance.h"
//#include "VTURayGPU.h"
//#include "VTURay.h" 

AVTUViewer::AVTUViewer() {
	PMC = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("PMC"));
	SetRootComponent(PMC);
	PMC->bUseAsyncCooking = true;
}
static void VTUMsg(const FString& S) {
    UE_LOG(LogTemp, Display, TEXT("%s"), *S);
    if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 8.f, FColor::Yellow, S);
}

//----------------------------------------------------------------Probe VTU------------------------------------------------------
void AVTUViewer::Probe_FileInfo(const FString& Path)
{
    int64 Size = 0; bool bUG = false, bAscii = false; FString FirstKB;
    const bool ok = VTUCore::Probe_FileInfo(Path, Size, bUG, bAscii, FirstKB);
    UE_LOG(LogTemp, Display, TEXT("[VTU:BP A] ok=%d size=%lld UG=%d ascii=%d"), ok, Size, bUG, bAscii);
}

void AVTUViewer::Probe_ParseHeaders(const FString& Path)
{
    int32 nPts = 0, nCells = 0, nConn = 0, nFaces = 0, nFoffs = 0; bool bPoly = false; float sec = 0;
    const bool ok = VTUCore::Probe_ParseHeaders_ASCII(Path, nPts, nCells, nConn, nFaces, nFoffs, bPoly, sec);
    UE_LOG(LogTemp, Display, TEXT("[VTU:BP B] ok=%d Pts=%d Cells=%d Conn=%d Faces=%d FOffs=%d Poly=%d (%.2fs)"),
        ok, nPts, nCells, nConn, nFaces, nFoffs, bPoly, sec);
}

void AVTUViewer::Probe_ParseHeaders_Light(const FString& Path)
{
    int32 nPts = 0, nCells = 0; bool hasC = false, hasO = false, hasT = false, hasF = false, hasFo = false; float sec = 0;
    const bool ok = VTUCore::Probe_ParseHeaders_Light(Path, nPts, nCells, hasC, hasO, hasT, hasF, hasFo, sec);
    UE_LOG(LogTemp, Display, TEXT("[VTU:BP B] ok=%d NumPoints=%d NumCells=%d | conn=%d offs=%d types=%d faces=%d faceoffs=%d (%.2fs)"),
        ok, nPts, nCells, hasC, hasO, hasT, hasF, hasFo, sec);
}

void AVTUViewer::Probe_ParseSamples(const FString& Path, int32 MaxPointTokens, int32 MaxConnTokens)
{
    int32 pTok = 0, cTok = 0; float sP = 0, sC = 0;
    const bool ok = VTUCore::Probe_ParseSamples_ASCII(Path, MaxPointTokens, MaxConnTokens, pTok, cTok, sP, sC);
    UE_LOG(LogTemp, Display, TEXT("[VTU:BP C] ok=%d Ptokens=%d (%.2fs) Ctokens=%d (%.2fs)"),
        ok, pTok, sP, cTok, sC);
}

void AVTUViewer::Probe_ReadOffsetsLast(const FString& Path, int32 NumCells)
{
    int64 last = 0; float sec = 0;
    const bool ok = VTUCore::ReadOffsetsLast_ASCII(Path, NumCells, last, sec);
    UE_LOG(LogTemp, Display, TEXT("[VTU:BP Offs] ok=%d lastOffset=%lld (%.2fs)"), ok, last, sec);
}

void AVTUViewer::Probe_ScanCountsAndOffsets(const FString& Path)
{
    int32 NumPoints = 0, NumCells = 0;
    bool  bHasConn = false, bHasOffs = false, bHasTypes = false, bHasFaces = false, bHasFaceOffs = false;
    int64 ConnLen = 0;
    float SecsHeaders = 0.f, SecsOffsets = 0.f;

    const bool ok = VTUCore::Probe_ScanCountsAndOffsets(
        Path,
        NumPoints, NumCells,
        bHasConn, bHasOffs, bHasTypes, bHasFaces, bHasFaceOffs,
        ConnLen,
        SecsHeaders, SecsOffsets
    );

    UE_LOG(LogTemp, Display,
        TEXT("[VTU:BP ALL] ok=%d | Points=%d Cells=%d | conn=%d offs=%d types=%d faces=%d faceoffs=%d | ConnLen=%lld | headers=%.2fs offsets=%.2fs"),
        ok ? 1 : 0,
        NumPoints, NumCells,
        bHasConn ? 1 : 0, bHasOffs ? 1 : 0, bHasTypes ? 1 : 0, bHasFaces ? 1 : 0, bHasFaceOffs ? 1 : 0,
        ConnLen, SecsHeaders, SecsOffsets);

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1, 6.f, ok ? FColor::Green : FColor::Red,
            FString::Printf(TEXT("VTU: ok=%d Pts=%d Cells=%d ConnLen=%lld (%.2fs + %.2fs)"),
                ok ? 1 : 0, NumPoints, NumCells, ConnLen, SecsHeaders, SecsOffsets));
    }
}

//----------------------------------------------------------------Draw Debug------------------------------------------------------
//void AVTUViewer::DrawWorldBounds(float Duration, float Thickness)
//{
//    const FBox LocalB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
//    const FTransform Xf = GetActorTransform();
//    const FVector Cw = Xf.TransformPosition(LocalB.GetCenter());
//    // Approx AABB monde : si rotation != 0, ça reste une approximation
//    const FVector ExtLocal = LocalB.GetExtent();
//    const FVector X = Xf.TransformVectorNoScale(FVector(ExtLocal.X, 0, 0)).GetAbs();
//    const FVector Y = Xf.TransformVectorNoScale(FVector(0, ExtLocal.Y, 0)).GetAbs();
//    const FVector Z = Xf.TransformVectorNoScale(FVector(0, 0, ExtLocal.Z)).GetAbs();
//    const FVector Ew = FVector(X.X + Y.X + Z.X, X.Y + Y.Y + Z.Y, X.Z + Y.Z + Z.Z);
//
//    DrawDebugBox(GetWorld(), Cw, Ew, FQuat::Identity, FColor::Cyan, false, Duration, 0, Thickness);
//}

// Dessine la boîte englobante du volume VTU (OBB orientée comme l'acteur)
void AVTUViewer::DrawWorldBounds(float Duration, float Thickness, FLinearColor Color)
{
    if (GridCache.Points.Num() == 0 || GridCache.NumCells() == 0) return;

    const FBox LocalB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
    if (!LocalB.IsValid) return;

    const FTransform Xf = GetActorTransform();
    const FVector    CenterWS = Xf.TransformPosition(LocalB.GetCenter());
    const FQuat      RotWS = Xf.GetRotation();

    // Extents de l’OBB = extents locaux * échelle de l’acteur
    const FVector ExtLocal = LocalB.GetExtent();
    const FVector ExtOBB = ExtLocal * Xf.GetScale3D().GetAbs();

    DrawDebugBox(GetWorld(), CenterWS, ExtOBB, RotWS, Color.ToFColor(true), /*bPersistent*/false, Duration, /*Depth*/0, Thickness);
}

// Dessine la boîte d’une cellule (via l’octree si dispo, sinon early out)
void AVTUViewer::DrawCellBounds(int32 CellIndex, float Duration, float Thickness, FLinearColor Color)
{
    if (!CellOctree.IsValid() || !CellOctree->IsBuilt()) return;

    FBox CellLS;
    const FBox* P = CellOctree->BoundsOf(CellIndex);
    if (!P || !P->IsValid) return;
    CellLS = *P;

    const FTransform Xf = GetActorTransform();
    const FVector    CenterWS = Xf.TransformPosition(CellLS.GetCenter());
    const FQuat      RotWS = Xf.GetRotation();
    const FVector    ExtOBB = CellLS.GetExtent() * Xf.GetScale3D().GetAbs();

    DrawDebugBox(GetWorld(), CenterWS, ExtOBB, RotWS, Color.ToFColor(true), /*bPersistent*/false, Duration, /*Depth*/0, Thickness);
}

static void DrawBoxEdges(UWorld* W, const TArray<FVector>& P, const FColor& C, bool bPersistent, float Duration, float Thk)
{
    auto L = [&](int a, int b) { DrawDebugLine(W, P[a], P[b], C, bPersistent, Duration, 0, Thk); };
    // indices des 12 arêtes d’un cube
    L(0, 1); L(1, 3); L(3, 2); L(2, 0); // bas
    L(4, 5); L(5, 7); L(7, 6); L(6, 4); // haut
    L(0, 4); L(1, 5); L(2, 6); L(3, 7); // montantes
}


//----------------------------------------------------------------Load VTU------------------------------------------------------
bool AVTUViewer::LoadAndShowVTU_ASCII_Streaming(const FString& AbsolutePath)
{
    if (!PMC) return false;
    FVTUGrid G;
    if (!VTUCore::LoadVTU_ASCII_Streaming(AbsolutePath, G, /*bLoadFacesIfPoly=*/true)) {
        UE_LOG(LogTemp, Error, TEXT("VTU streaming load failed"));
        return false;
    }
    const bool okSurf = VTUCore::BuildSurfaceToPMC(G, PMC, ScaleCm);
    UE_LOG(LogTemp, Display, TEXT("VTU streaming: Points=%d Cells=%d Conn=%d Faces=%d FaceOffs=%d -> Surf=%d"),
        G.Points.Num(), G.NumCells(), G.Connectivity.Num(), G.Faces.Num(), G.FaceOffsets.Num(), okSurf);
    return okSurf;
}

bool AVTUViewer::LoadOrParseVTUAndShow(const FString& VtuPath, const FString& CacheFilePath)
{
    FString CachePath = CacheFilePath;
    if (CachePath.IsEmpty()) {
        CachePath = FPaths::ProjectSavedDir() / TEXT("VTUCache/furnace.vbin");
    }

    bool bLoadedFromCache = false;
    if (FPaths::FileExists(CachePath)) {
        UE_LOG(LogTemp, Display, TEXT("VTU: trying cache '%s'"), *CachePath);
        bLoadedFromCache = VTUCore::LoadGridBinary(CachePath, GridCache);
        if (!bLoadedFromCache) {
            UE_LOG(LogTemp, Warning, TEXT("VTU: cache invalid, will re-parse VTU"));
        }
    }

    if (!bLoadedFromCache) {
        UE_LOG(LogTemp, Display, TEXT("VTU: parsing '%s' (streaming)"), *VtuPath);
        if (!VTUCore::LoadVTU_ASCII_Streaming(VtuPath, GridCache, /*bLoadFacesIfPoly=*/true)) {
            UE_LOG(LogTemp, Error, TEXT("VTU: parse failed"));
            return false;
        }
        // Sauvegarde
        const FString Dir = FPaths::GetPath(CachePath);
        IPlatformFile& PF = FPlatformFileManager::Get().GetPlatformFile();
        PF.CreateDirectoryTree(*Dir);
        if (VTUCore::SaveGridBinary(CachePath, GridCache)) {
            UE_LOG(LogTemp, Display, TEXT("VTU: cache written to '%s'"), *CachePath);
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("VTU: failed to write cache '%s'"), *CachePath);
        }
    }
    else {
        UE_LOG(LogTemp, Display, TEXT("VTU: loaded grid from cache"));
    }

    // Affiche la peau (section 0)
    if (!PMC) return false;
    const bool okSurf = VTUCore::BuildSurfaceToPMC(GridCache, PMC, ScaleCm);
    UE_LOG(LogTemp, Display, TEXT("VTU: surface %s (Pts=%d Cells=%d Conn=%d)"),
        okSurf ? TEXT("OK") : TEXT("FAIL"),
        GridCache.Points.Num(), GridCache.NumCells(), GridCache.Connectivity.Num());
    return okSurf;
}
//----------------------------------------------------------------Load Material------------------------------------------------------
bool AVTUViewer::SetSurfaceMaterial(UMaterialInterface* Material, int32 SectionIndex)
{
    if (!PMC || !Material) return false;

    auto Apply = [&](int32 Sec) {
        if (Sec < 0) return;
        PMC->SetMaterial(Sec, Material);
        };

    if (SectionIndex >= 0) {
        Apply(SectionIndex);
    }
    else {
        // -1 => on couvre les sections qu'on utilise dans ce projet : 0 (peau), 1 (slice/alt surface)
        Apply(0);
        Apply(1);
    }
    return true;
}

bool AVTUViewer::SetSurfaceMaterialByPath(const FString& AssetPath, int32 SectionIndex)
{
    if (AssetPath.IsEmpty()) return false;

    FSoftObjectPath SoftPath(AssetPath);
    UMaterialInterface* Mat = Cast<UMaterialInterface>(SoftPath.TryLoad()); // synchro, une fois au démarrage
    if (!Mat) {
        UE_LOG(LogTemp, Error, TEXT("SetSurfaceMaterialByPath: not found %s"), *AssetPath);
        return false;
    }
    return SetSurfaceMaterial(Mat, SectionIndex);
}
//----------------------------------------------------------------Basic Color Display------------------------------------------------
bool AVTUViewer::LoadPredictedVectorNPY(const FString& NpyPath)
{
    if (GridCache.NumCells() <= 0) {
        UE_LOG(LogTemp, Error, TEXT("LoadPredictedVectorNPY: grid not loaded"));
        return false;
    }
    return VTUCore::LoadNPY_CellFeatures(NpyPath, GridCache.NumCells(), CellFeatures);
}

static const TArray<float>* GetFeatureArray(const FVTUCellFeatures& F, EVTUCellFeature Which)
{
    switch (Which)
    {
    case EVTUCellFeature::NOx: return &F.NOx;
    case EVTUCellFeature::CO:  return &F.CO;
    case EVTUCellFeature::OH:  return &F.OH;
    case EVTUCellFeature::H2:  return &F.H2;
    case EVTUCellFeature::H2O: return &F.H2O;
    case EVTUCellFeature::CO2: return &F.CO2;
    case EVTUCellFeature::O2:  return &F.O2;
    case EVTUCellFeature::CH4: return &F.CH4;
    case EVTUCellFeature::T:   return &F.T;
    default: return nullptr;
    }
}

bool AVTUViewer::ApplyFeatureToSurface(EVTUCellFeature Feature, float VMin, float VMax, int32 SectionIndex)
{
    if (!PMC || GridCache.NumCells() <= 0) {
        UE_LOG(LogTemp, Error, TEXT("ApplyFeatureToSurface: grid not loaded"));
        return false;
    }
    const TArray<float>* CellS = GetFeatureArray(CellFeatures, Feature);
    if (!CellS || CellS->Num() != GridCache.NumCells()) {
        UE_LOG(LogTemp, Error, TEXT("ApplyFeatureToSurface: feature array invalid (got %d, need %d)"),
            CellS ? CellS->Num() : -1, GridCache.NumCells());
        return false;
    }

    // Essaie la version colorisée via faces/faceoffsets (polyhedra)
    bool ok = VTUCore::BuildSurfaceToPMC_WithCellScalars(GridCache, PMC, ScaleCm, *CellS, VMin, VMax, SectionIndex);

    // Fallback : si échec (pas de faces/faceoffsets), on affiche la surface “blanche” existante.
    if (!ok) {
        UE_LOG(LogTemp, Warning, TEXT("ApplyFeatureToSurface: falling back to non-colored surface (no poly faces?)"));
        ok = VTUCore::BuildSurfaceToPMC(GridCache, PMC, ScaleCm); // ta version déjà utilisée
    }
    return ok;
}


//-----------------------------------------------------TEST---------------------------------------------------------
// // Applique Flip/Swap XY autour d'un pivot local (cm)
static FORCEINLINE FVector ApplyFlipSwapXY(const FVector& P, const FVector& PivotLS,
    bool bFlipX, bool bFlipY, bool bSwapXY)
{
    FVector d = P - PivotLS; // vecteur relatif au pivot
    if (bSwapXY) Swap(d.X, d.Y);
    if (bFlipX)  d.X = -d.X;
    if (bFlipY)  d.Y = -d.Y;
    return PivotLS + d;
}

// BBox d'une cellule (dans un jeu de points donné)
static FBox CellAABB_FromPoints(const FVTUGrid& G, int32 ci, const TArray<FVector3f>& PtsLS)
{
    const int32 start = (ci == 0 ? 0 : G.Offsets[ci - 1]);
    const int32 end = G.Offsets[ci];
    FBox B(ForceInitToZero);
    for (int32 k = start; k < end; ++k)
    {
        const int32 vi = G.Connectivity[k];
        if (!PtsLS.IsValidIndex(vi)) continue;
        B += FVector(PtsLS[vi]);
    }
    return B;
}

// Construit un clone (points transformés) + son octree (indexe les CellIndex du 1/8)
bool AVTUViewer::BuildOneClone_XY(bool bFlipX, bool bFlipY, bool bSwapXY,
    const FString& Label,
    int32 MaxPerLeaf, int32 MaxDepth,
    FVTUGridClone& OutClone)
{
    const FVTUGrid& G = GridCache;
    if (G.Points.Num() == 0 || G.NumCells() == 0) return false;

    // 1) Points transformés (LOCAL cm)
    OutClone.PointsLS.SetNumUninitialized(G.Points.Num());
    FBox BB(ForceInitToZero);
    for (int32 i = 0; i < G.Points.Num(); ++i)
    {
        const FVector pLS = FVector(G.Points[i]) * ScaleCm; // base en cm
        const FVector qLS = ApplyFlipSwapXY(pLS, ClonePivotLS, bFlipX, bFlipY, bSwapXY);
        OutClone.PointsLS[i] = FVector3f(qLS);
        BB += qLS;
    }
    OutClone.BoundsLS = BB;
    OutClone.Label = Label;

    // 2) Proxies (une AABB par cellule) -> octree
    TArray<FVTUCellProxy> Proxies;
    Proxies.Reserve(G.NumCells());
    for (int32 ci = 0; ci < G.NumCells(); ++ci)
    {
        const FBox CB = CellAABB_FromPoints(G, ci, OutClone.PointsLS);
        if (!CB.IsValid) continue;
        FVTUCellProxy pr;
        pr.Cell = ci;
        pr.Bounds = CB;
        Proxies.Add(pr);
    }

    // 3) Octree
    OutClone.Octree = MakeShared<FVTUCellOctree>();
    const bool ok = OutClone.Octree->BuildFromProxies(Proxies, OutClone.BoundsLS, MaxPerLeaf, MaxDepth);
    if (!ok)
    {
        OutClone.Octree.Reset();
        return false;
    }

    return true;
}

void AVTUViewer::SetClonePivotLocal(const FVector& PivotLS)
{
    ClonePivotLS = PivotLS; // typiquement (0,0,0)
}

void AVTUViewer::ClearClonedOctrees()
{
    GridClones.Empty();
}

bool AVTUViewer::BuildClonedOctrees_XY(bool bIncludeIdentity, int32 MaxPerLeaf, int32 MaxDepth)
{
    GridClones.Empty();

    auto Add = [&](bool fx, bool fy, bool sw, const TCHAR* tag)->bool
        {
            FVTUGridClone C;
            if (!BuildOneClone_XY(fx, fy, sw, tag, MaxPerLeaf, MaxDepth, C)) return false;
            GridClones.Add(MoveTemp(C));
            return true;
        };

    bool ok = true;
    if (bIncludeIdentity) ok &= Add(false, false, false, TEXT("Id"));

    // Flips (X, Y, XY)
    ok &= Add(true, false, false, TEXT("FlipX"));
    ok &= Add(false, true, false, TEXT("FlipY"));
    ok &= Add(true, true, false, TEXT("FlipXY"));

    // Swap XY et ses flips
    ok &= Add(false, false, true, TEXT("SwapXY"));
    ok &= Add(true, false, true, TEXT("SwapXY_FlipX"));
    ok &= Add(false, true, true, TEXT("SwapXY_FlipY"));
    ok &= Add(true, true, true, TEXT("SwapXY_FlipXY"));

    UE_LOG(LogTemp, Display, TEXT("Built %d clones (ok=%d)"), GridClones.Num(), ok ? 1 : 0);
    return ok;
}

//-----------------------------------------------------Octree---------------------------------------------------------
bool AVTUViewer::BuildCellOctree()
{
    if (GridCache.NumCells() <= 0) {
        UE_LOG(LogTemp, Warning, TEXT("BuildCellOctree: grid is empty"));
        return false;
    }

    // 1) Proxies (AABB par cellule, en cm monde)
    TArray<FVTUCellProxy> Proxies;
    Proxies.Reserve(GridCache.NumCells());

    int32 start = 0;
    for (int32 ci = 0; ci < GridCache.NumCells(); ++ci)
    {
        const int32 end = GridCache.Offsets[ci];
        FBox b(ForceInitToZero);
        for (int32 k = start; k < end; ++k) {
            const int32 vi = GridCache.Connectivity[k];
            if (!GridCache.Points.IsValidIndex(vi)) continue;
            b += FVector(GridCache.Points[vi]) * ScaleCm;
        }
        start = end;

        if (b.IsValid) {
            FVTUCellProxy P; P.Bounds = b; P.Cell = ci;
            Proxies.Add(P);
        }
    }

    // 2) World bounds (en cm)
    const FBox WorldB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);

    // 3) Build
    CellOctree = MakeUnique<FVTUCellOctree>();
    const double t0 = FPlatformTime::Seconds();
    CellOctree->Build(Proxies, WorldB, /*MaxPerLeaf=*/32, /*MaxDepth=*/10);
    const double dt = FPlatformTime::Seconds() - t0;

    UE_LOG(LogTemp, Display, TEXT("VTU Octree: built %d cells in %.2fs | Bounds Min(%.1f,%.1f,%.1f) Max(%.1f,%.1f,%.1f)"),
        CellOctree->NumElements(), dt,
        WorldB.Min.X, WorldB.Min.Y, WorldB.Min.Z,
        WorldB.Max.X, WorldB.Max.Y, WorldB.Max.Z);
    return true;
}

void AVTUViewer::ClearCellOctree()
{
    if (CellOctree.IsValid()) {
        CellOctree.Reset();
        UE_LOG(LogTemp, Display, TEXT("VTU Octree: cleared"));
    }
}


//-----------------------------------------------------Raymarch---------------------------------------------------------
// Ray <=> AABB (slab test) en LOCAL
static bool IntersectRayAABB_Slab(const FVector& Ro, const FVector& Rd, const FBox& Box, float& OutTMin, float& OutTMax)
{
    const FVector Inv((Rd.X != 0.f) ? 1.f / Rd.X : BIG_NUMBER,
        (Rd.Y != 0.f) ? 1.f / Rd.Y : BIG_NUMBER,
        (Rd.Z != 0.f) ? 1.f / Rd.Z : BIG_NUMBER);
    const FVector t1 = (Box.Min - Ro) * Inv;
    const FVector t2 = (Box.Max - Ro) * Inv;
    const float tmin = FMath::Max3(FMath::Min(t1.X, t2.X), FMath::Min(t1.Y, t2.Y), FMath::Min(t1.Z, t2.Z));
    const float tmax = FMath::Min3(FMath::Max(t1.X, t2.X), FMath::Max(t1.Y, t2.Y), FMath::Max(t1.Z, t2.Z));
    OutTMin = tmin; OutTMax = tmax;
    return tmax >= FMath::Max(tmin, 0.f);
}

// Colormap Viridis (stops sRGB -> convertis en LINÉAIRE, puis interpolation)
// Source stops (approx.): [ (68,1,84), (59,82,139), (33,145,140), (94,201,98), (253,231,37) ]
static FLinearColor ScalarToColor(float v, float vmin, float vmax)
{
    if (!(vmax > vmin)) return FLinearColor::Gray;

    float t = (v - vmin) / (vmax - vmin);
    t = FMath::Clamp(t, 0.f, 1.f);

    static bool bInit = false;
    static FLinearColor StopsLin[6];
    if (!bInit)
    {
        const FColor StopsSRGB[6] = {
            FColor(34,  31, 123, 255), // bleu
            FColor(36, 132, 181, 255), // cyan
            FColor(67, 174, 132, 255), // vert-bleuté
            FColor(160, 190,  80, 255), // jaune-vert
            FColor(220, 152,  66, 255), // orange
            FColor(220,  49,  32, 255)  // rouge
        };
        for (int i = 0; i < 6; ++i) StopsLin[i] = FLinearColor::FromSRGBColor(StopsSRGB[i]);
        bInit = true;
    }

    const float f = t * 5.f;                // 5 segments
    const int   i = FMath::Clamp((int)FMath::FloorToFloat(f), 0, 4);
    const float u = f - (float)i;

    return FMath::Lerp(StopsLin[i], StopsLin[i + 1], u); // A=1 par défaut
}
//// Map scalaire -> couleur (bleu->rouge simple)
//static FLinearColor ScalarToColor(float v, float vmin, float vmax)
//{
//    if (!(vmax > vmin)) return FLinearColor::Gray;
//    const float t = FMath::Clamp((v - vmin) / (vmax - vmin), 0.f, 1.f);
//    return FLinearColor(t, 1.f - FMath::Abs(2 * t - 1.f), 1.f - t, 1.f);
//}

// Itère les faces d'une cellule (tet/hex/wedge/pyramid/polyhedron) et appelle Fn(fv,nv)
static void ForEachCellFace_VTK(const FVTUGrid& G, int32 ci, TFunctionRef<void(const int32* fv, int32 nv)> Fn)
{
    const int32 start = (ci == 0 ? 0 : G.Offsets[ci - 1]);
    const int32 end = G.Offsets[ci];
    if (end <= start || !G.Types.IsValidIndex(ci)) return;

    const uint8 ty = G.Types[ci];
    const int32* v = G.Connectivity.GetData() + start;

    auto FaceCall = [&](const int32* fv, int32 nv) { if (nv >= 3) Fn(fv, nv); };

    switch ((EVTKCellType)ty)
    {
    case EVTKCellType::VTK_TETRA: {
        int32 f0[3] = { v[0],v[1],v[2] }; FaceCall(f0, 3);
        int32 f1[3] = { v[0],v[3],v[1] }; FaceCall(f1, 3);
        int32 f2[3] = { v[1],v[3],v[2] }; FaceCall(f2, 3);
        int32 f3[3] = { v[0],v[2],v[3] }; FaceCall(f3, 3);
        break;
    }
    case EVTKCellType::VTK_HEXAHEDRON: {
        int32 f0[4] = { v[0],v[1],v[2],v[3] }; FaceCall(f0, 4);
        int32 f1[4] = { v[4],v[5],v[6],v[7] }; FaceCall(f1, 4);
        int32 f2[4] = { v[0],v[4],v[5],v[1] }; FaceCall(f2, 4);
        int32 f3[4] = { v[1],v[5],v[6],v[2] }; FaceCall(f3, 4);
        int32 f4[4] = { v[2],v[6],v[7],v[3] }; FaceCall(f4, 4);
        int32 f5[4] = { v[3],v[7],v[4],v[0] }; FaceCall(f5, 4);
        break;
    }
    case EVTKCellType::VTK_WEDGE: {
        int32 f0[3] = { v[0],v[1],v[2] }; FaceCall(f0, 3);
        int32 f1[3] = { v[3],v[4],v[5] }; FaceCall(f1, 3);
        int32 f2[4] = { v[0],v[3],v[4],v[1] }; FaceCall(f2, 4);
        int32 f3[4] = { v[1],v[4],v[5],v[2] }; FaceCall(f3, 4);
        int32 f4[4] = { v[2],v[5],v[3],v[0] }; FaceCall(f4, 4);
        break;
    }
    case EVTKCellType::VTK_PYRAMID: {
        int32 f0[4] = { v[0],v[1],v[2],v[3] }; FaceCall(f0, 4);
        int32 f1[3] = { v[0],v[4],v[1] };     FaceCall(f1, 3);
        int32 f2[3] = { v[1],v[4],v[2] };     FaceCall(f2, 3);
        int32 f3[3]{ v[2],v[4],v[3] };      FaceCall(f3, 3);
        int32 f4[3]{ v[3],v[4],v[0] };      FaceCall(f4, 3);
        break;
    }
    case EVTKCellType::VTK_POLYHEDRON: {
        if (!G.HasPolyFaces()) break;
        const int32 sF = (ci == 0 ? 0 : G.FaceOffsets[ci - 1]);
        const int32 eF = G.FaceOffsets[ci];
        if (eF <= sF) break;

        int32 idx = sF;
        const int32 nFaces = G.Faces[idx++];

        for (int32 f = 0; f < nFaces && idx < eF; ++f)
        {
            if (idx >= eF) break;
            const int32 nv = G.Faces[idx++];
            if (nv < 3 || idx + nv > eF) { idx = eF; break; }

            TArray<int32, TInlineAllocator<16>> tmp;
            tmp.SetNumUninitialized(nv);
            for (int32 k = 0; k < nv; ++k) tmp[k] = G.Faces[idx + k];
            idx += nv;

            FaceCall(tmp.GetData(), nv);
        }
        break;
    }
    default: break;
    }
}

// Intersection précise rayon <=> cellule "ci" (convexe) en LOCAL cm.
// Retourne true et tEntry si le rayon entre dans la cellule.
static bool RayIntersectConvexCell_FirstT(const FVTUGrid& G, int32 ci, float ScaleCm,
    const FVector& oLS, const FVector& dLS,
    float Eps, float& outTEntry)
{
    outTEntry = TNumericLimits<float>::Max();
    if (!G.Types.IsValidIndex(ci)) return false;

    // Centroïde approx. pour orienter les normales
    const int32 start = (ci == 0 ? 0 : G.Offsets[ci - 1]);
    const int32 end = G.Offsets[ci];
    if (end <= start) return false;

    FVector C(0, 0, 0); int32 nC = 0;
    for (int32 k = start; k < end; ++k) {
        const int32 vi = G.Connectivity[k];
        if (!G.Points.IsValidIndex(vi)) continue;
        C += FVector(G.Points[vi]) * ScaleCm; ++nC;
    }
    if (nC == 0) return false;
    C /= float(nC);

    float tEnter = 0.f;
    float tExit = TNumericLimits<float>::Max();
    bool  hadFace = false;

    ForEachCellFace_VTK(G, ci, [&](const int32* fv, int32 nv)
        {
            if (nv < 3) return;
            hadFace = true;

            const FVector p0 = FVector(G.Points[fv[0]]) * ScaleCm;
            const FVector p1 = FVector(G.Points[fv[1]]) * ScaleCm;
            const FVector p2 = FVector(G.Points[fv[2]]) * ScaleCm;

            FVector n = FVector::CrossProduct(p1 - p0, p2 - p0);
            if (!n.Normalize()) return;

            // normale vers l'extérieur
            if (FVector::DotProduct(n, C - p0) > 0) n *= -1.f;

            const float Nd = FVector::DotProduct(n, dLS);
            const float No = FVector::DotProduct(n, oLS - p0);

            if (FMath::Abs(Nd) < 1e-8f)
            {
                // parallèle : si clairement dehors -> rejet
                if (No > Eps) { tEnter = 1.f; tExit = 0.f; return; }
                // sinon, contrainte inactive
                return;
            }

            const float tHit = -No / Nd; // NOTE: plus d'Eps ici

            if (Nd < 0.f) tEnter = FMath::Max(tEnter, tHit); // entrant
            else          tExit = FMath::Min(tExit, tHit); // sortant
        });

    if (!hadFace) return false;
    if (tEnter > tExit) return false;
    if (tExit < 0.f)  return false; // tout derrière

    // Vérif finale: point d'entrée réellement "à l'intérieur" (tolérance Eps)
    const FVector pEnt = oLS + dLS * FMath::Max(0.f, tEnter);
    bool inside = true;
    ForEachCellFace_VTK(G, ci, [&](const int32* fv, int32 nv)
        {
            if (!inside || nv < 3) return;
            const FVector p0 = FVector(G.Points[fv[0]]) * ScaleCm;
            const FVector p1 = FVector(G.Points[fv[1]]) * ScaleCm;
            const FVector p2 = FVector(G.Points[fv[2]]) * ScaleCm;
            FVector n = FVector::CrossProduct(p1 - p0, p2 - p0);
            if (!n.Normalize()) return;
            if (FVector::DotProduct(n, C - p0) > 0) n *= -1.f;

            const float dist = FVector::DotProduct(n, pEnt - p0); // >0 => dehors
            if (dist > Eps) inside = false;
        });
    if (!inside) return false;

    outTEntry = FMath::Max(0.f, tEnter);
    return true;
}

// Raccourci pour récupérer le tableau de la feature
static const TArray<float>* FeatureArrayRef(const FVTUCellFeatures& F, EVTUCellFeature Feat)
{
    switch (Feat) {
    case EVTUCellFeature::NOx: return &F.NOx;
    case EVTUCellFeature::CO:  return &F.CO;
    case EVTUCellFeature::OH:  return &F.OH;
    case EVTUCellFeature::H2:  return &F.H2;
    case EVTUCellFeature::H2O: return &F.H2O;
    case EVTUCellFeature::CO2: return &F.CO2;
    case EVTUCellFeature::O2:  return &F.O2;
    case EVTUCellFeature::CH4: return &F.CH4;
    case EVTUCellFeature::T:   return &F.T;
    default: return nullptr;
    }
}

static bool RayIntersectConvexCell_FirstT_WithPoints(const FVTUGrid& G,
    const TArray<FVector3f>& PtsLS,
    int32 ci,
    const FVector& oLS,
    const FVector& dLS,
    float Eps,
    float& outTEntry)
{
    outTEntry = TNumericLimits<float>::Max();
    if (!G.Types.IsValidIndex(ci)) return false;

    const int32 start = (ci == 0 ? 0 : G.Offsets[ci - 1]);
    const int32 end = G.Offsets[ci];
    if (end <= start) return false;

    // centroïde (dans le clone)
    FVector C(0, 0, 0); int32 nC = 0;
    for (int32 k = start; k < end; ++k) {
        const int32 vi = G.Connectivity[k];
        if (!PtsLS.IsValidIndex(vi)) continue;
        C += FVector(PtsLS[vi]);
        ++nC;
    }
    if (nC == 0) return false;
    C /= float(nC);

    float tEnter = 0.f;
    float tExit = TNumericLimits<float>::Max();
    bool  hadFace = false;

    ForEachCellFace_VTK(G, ci, [&](const int32* fv, int32 nv)
        {
            if (nv < 3) return;
            hadFace = true;

            const FVector p0 = FVector(PtsLS[fv[0]]);
            const FVector p1 = FVector(PtsLS[fv[1]]);
            const FVector p2 = FVector(PtsLS[fv[2]]);

            FVector n = FVector::CrossProduct(p1 - p0, p2 - p0);
            if (!n.Normalize()) return;

            if (FVector::DotProduct(n, C - p0) > 0) n *= -1.f;

            const float Nd = FVector::DotProduct(n, dLS);
            const float No = FVector::DotProduct(n, oLS - p0);

            if (FMath::Abs(Nd) < 1e-8f)
            {
                if (No > Eps) { tEnter = 1.f; tExit = 0.f; return; }
                return;
            }

            const float tHit = -No / Nd;
            if (Nd < 0.f) tEnter = FMath::Max(tEnter, tHit);
            else          tExit = FMath::Min(tExit, tHit);
        });

    if (!hadFace) return false;
    if (tEnter > tExit) return false;
    if (tExit < 0.f)  return false;

    // inside check
    const FVector pEnt = oLS + dLS * FMath::Max(0.f, tEnter);
    bool inside = true;
    ForEachCellFace_VTK(G, ci, [&](const int32* fv, int32 nv)
        {
            if (!inside || nv < 3) return;
            const FVector p0 = FVector(PtsLS[fv[0]]);
            const FVector p1 = FVector(PtsLS[fv[1]]);
            const FVector p2 = FVector(PtsLS[fv[2]]);
            FVector n = FVector::CrossProduct(p1 - p0, p2 - p0);
            if (!n.Normalize()) return;
            if (FVector::DotProduct(n, C - p0) > 0) n *= -1.f;
            const float dist = FVector::DotProduct(n, pEnt - p0);
            if (dist > Eps) inside = false;
        });

    if (!inside) return false;
    outTEntry = FMath::Max(0.f, tEnter);
    return true;
}

bool AVTUViewer::RaycastFirstCell(EVTUCellFeature Feature,
    const FVector& RayOriginWS,
    const FVector& RayDirWS,
    int32& OutCellIndex,
    FVector& OutHitWS,
    FLinearColor& OutColor,
    float StepCm,
    float VMin, float VMax, float EpsCm,
    bool  bEnableClip,
    FVector ClipOriginWS, FVector ClipNormalWS,
    float  ClipThicknessCm, bool bKeepFront)
{
    OutCellIndex = -1; OutHitWS = FVector::ZeroVector; OutColor = FLinearColor(0, 0, 0, 0);

    // Monde -> Local (cm)
    const FTransform Xf = GetActorTransform();
    const FTransform InvXf = Xf.Inverse();
    const FVector oLS = InvXf.TransformPosition(RayOriginWS);
    const FVector dLS = InvXf.TransformVectorNoScale(RayDirWS).GetSafeNormal();

    // Reutilise ta RaymarchDebug clones-path pour trouver bestCi/bestT
    int32 ci = -1;
    float eps = (EpsCm > 0.f) ? EpsCm : FMath::Max(0.05f * StepCm, 0.25f);

    // On va reprendre la même logique que dans RaymarchDebug(...) mais on garde bestT
    auto Solve = [&](int32& OutCi, float& OutT)->bool
        {
            const FVTUGrid& G = GridCache;
            const float   Eps = eps;
            const FVector pad(FMath::Max(0.5f * Eps, 0.25f));

            // Clip plan local
            float tNear = 0.f, tFar = TNumericLimits<float>::Max();
            if (bEnableClip)
            {
                const FVector Nls = InvXf.TransformVectorNoScale(ClipNormalWS).GetSafeNormal();
                const FVector Pls = InvXf.TransformPosition(ClipOriginWS);
                const float a = FVector::DotProduct(Nls, dLS);
                const float b = FVector::DotProduct(Nls, oLS - Pls);
                const float half = FMath::Max(0.f, 0.5f * ClipThicknessCm);

                if (FMath::Abs(a) < 1e-8f)
                {
                    const float s = b;
                    const bool keep = bKeepFront ? (s >= +half) : (s <= -half);
                    if (!keep) return false;
                }
                else
                {
                    if (bKeepFront)
                    {
                        const float tCut = (+half - b) / a;
                        if (a > 0) tNear = FMath::Max(tNear, tCut);
                        else       tFar = FMath::Min(tFar, tCut);
                    }
                    else
                    {
                        const float tCut = (-half - b) / a;
                        if (a > 0) tFar = FMath::Min(tFar, tCut);
                        else       tNear = FMath::Max(tNear, tCut);
                    }
                    if (tNear >= tFar) return false;
                }
            }

            float bestT = TNumericLimits<float>::Max();
            int32 bestCi = -1;

            auto TryClone = [&](const FVTUGridClone& C)
                {
                    float ta = 0.f, tb = 0.f;
                    if (!IntersectRayAABB_Slab(oLS, dLS, C.BoundsLS, ta, tb)) return;
                    ta = FMath::Max(ta, tNear);
                    tb = FMath::Min(tb, tFar);
                    if (ta >= tb) return;

                    const FVector p0 = oLS + dLS * ta;
                    const FVector p1 = oLS + dLS * tb;
                    const FVector mn(FMath::Min(p0.X, p1.X), FMath::Min(p0.Y, p1.Y), FMath::Min(p0.Z, p1.Z));
                    const FVector mx(FMath::Max(p0.X, p1.X), FMath::Max(p0.Y, p1.Y), FMath::Max(p0.Z, p1.Z));
                    const FBox segBox(mn - pad, mx + pad);

                    TArray<int32> candidates;
                    C.Octree->QueryAABB(segBox, candidates);
                    for (int32 cidx : candidates)
                    {
                        const FBox* cb = C.Octree->BoundsOf(cidx);
                        if (cb)
                        {
                            float t0 = 0.f, t1 = 0.f;
                            if (!IntersectRayAABB_Slab(oLS, dLS, *cb, t0, t1)) continue;
                            if (t1 < ta) continue;
                        }

                        float tEnt = 0.f;
                        if (RayIntersectConvexCell_FirstT_WithPoints(G, C.PointsLS, cidx, oLS, dLS, Eps, tEnt))
                        {
                            if (tEnt >= ta && tEnt < bestT) { bestT = tEnt; bestCi = cidx; }
                        }
                    }
                };

            if (bUseClonedOctrees && GridClones.Num() > 0)
            {
                for (const auto& C : GridClones) if (C.Octree.IsValid()) TryClone(C);
            }
            else
            {
                // fallback 1/8
                const FBox BaseBoxLS(G.Bounds.Min * ScaleCm, G.Bounds.Max * ScaleCm);
                float ta = 0.f, tb = 0.f;
                if (IntersectRayAABB_Slab(oLS, dLS, BaseBoxLS, ta, tb))
                {
                    ta = FMath::Max(ta, tNear);
                    tb = FMath::Min(tb, tFar);
                    if (ta < tb)
                    {
                        const FVector p0 = oLS + dLS * ta;
                        const FVector p1 = oLS + dLS * tb;
                        const FBox segBox(
                            FVector(FMath::Min(p0.X, p1.X), FMath::Min(p0.Y, p1.Y), FMath::Min(p0.Z, p1.Z)) - pad,
                            FVector(FMath::Max(p0.X, p1.X), FMath::Max(p0.Y, p1.Y), FMath::Max(p0.Z, p1.Z)) + pad);

                        TArray<int32> candidates;
                        CellOctree->QueryAABB(segBox, candidates);

                        for (int32 cidx : candidates)
                        {
                            const FBox* cb = CellOctree->BoundsOf(cidx);
                            if (cb)
                            {
                                float t0 = 0.f, t1 = 0.f;
                                if (!IntersectRayAABB_Slab(oLS, dLS, *cb, t0, t1)) continue;
                                if (t1 < ta) continue;
                            }

                            float tEnt = 0.f;
                            if (RayIntersectConvexCell_FirstT(GridCache, cidx, ScaleCm, oLS, dLS, Eps, tEnt))
                            {
                                if (tEnt >= ta && tEnt < bestT) { bestT = tEnt; bestCi = cidx; }
                            }
                        }
                    }
                }
            }

            if (bestCi < 0) return false;
            OutCi = bestCi; OutT = bestT; return true;
        };

    float tHit = 0.f;
    if (!Solve(OutCellIndex, tHit)) return false;

    const TArray<float>* Arr = FeatureArrayRef(CellFeatures, Feature);
    const float v = (*Arr)[OutCellIndex];
    OutColor = ScalarToColor(v, VMin, VMax);

    const FVector hitLS = oLS + dLS * tHit;
    OutHitWS = Xf.TransformPosition(hitLS);
    return true;
}


UTexture2D* AVTUViewer::RaymarchToTexture2D(
    int32 Width, int32 Height,
    EVTUCellFeature Feature,
    float StepCm, float VMin, float VMax, float EpsCm,
    bool bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS,
    float ClipThicknessCm, bool bKeepFront,
    bool bCropToOBB, bool bClearTransparent)
{
    if (Width <= 0 || Height <= 0) return nullptr;
    if (!GetWorld()) return nullptr;

    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC || !PC->PlayerCameraManager) return nullptr;

    int32 ViewX = 0, ViewY = 0;
    PC->GetViewportSize(ViewX, ViewY);
    if (ViewX <= 0 || ViewY <= 0) { ViewX = Width; ViewY = Height; }

    // DEPTH: calcule MaxDepthCm = diagonale des bounds actifs en MONDE
    const FBox ActiveLS = GetActiveVolumeBoundsLS();
    if (!ActiveLS.IsValid) return nullptr;
    const FTransform Xf = GetActorTransform();
    const FVector WMin = Xf.TransformPosition(ActiveLS.Min);
    const FVector WMax = Xf.TransformPosition(ActiveLS.Max);
    const float   MaxDepthCm = FVector::Distance(WMin, WMax); // borne sup (cm)

    // Texture BGRA8
    UTexture2D* Tex = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8);
    Tex->SRGB = true;

    TArray<FColor> Pixels;
    Pixels.SetNumUninitialized(Width * Height);
    const FColor Clear = bClearTransparent ? FColor(0, 0, 0, 0) : FColor(0, 0, 0, 255);
    for (int i = 0; i < Pixels.Num(); ++i) Pixels[i] = Clear;

    // plein écran par défaut
    int32 sx0 = 0, sy0 = 0, sx1 = ViewX - 1, sy1 = ViewY - 1;
    int32 tx0 = 0, ty0 = 0, tx1 = Width - 1, ty1 = Height - 1;

    if (bCropToOBB)
    {
        // projette la boîte ACTIVE (union clones)
        const FVector mn = ActiveLS.Min, mx = ActiveLS.Max;
        const FVector C[8] = {
            {mn.X,mn.Y,mn.Z},{mx.X,mn.Y,mn.Z},{mn.X,mx.Y,mn.Z},{mx.X,mx.Y,mn.Z},
            {mn.X,mn.Y,mx.Z},{mx.X,mn.Y,mx.Z},{mn.X,mx.Y,mx.Z},{mx.X,mx.Y,mx.Z}
        };

        float minSX = +FLT_MAX, minSY = +FLT_MAX, maxSX = -FLT_MAX, maxSY = -FLT_MAX;
        int projected = 0;
        for (int i = 0; i < 8; ++i)
        {
            const FVector W = Xf.TransformPosition(C[i]);
            FVector2D S;
            if (PC->ProjectWorldLocationToScreen(W, S, /*bPlayerViewportRelative=*/true))
            {
                minSX = FMath::Min(minSX, S.X);
                minSY = FMath::Min(minSY, S.Y);
                maxSX = FMath::Max(maxSX, S.X);
                maxSY = FMath::Max(maxSY, S.Y);
                ++projected;
            }
        }

        if (projected > 0)
        {
            const float pad = 2.f;
            sx0 = FMath::Clamp(int32(FMath::FloorToInt(minSX - pad)), 0, ViewX - 1);
            sy0 = FMath::Clamp(int32(FMath::FloorToInt(minSY - pad)), 0, ViewY - 1);
            sx1 = FMath::Clamp(int32(FMath::CeilToInt(maxSX + pad)), 0, ViewX - 1);
            sy1 = FMath::Clamp(int32(FMath::CeilToInt(maxSY + pad)), 0, ViewY - 1);

            tx0 = FMath::Clamp(int32((float)Width * (float(sx0) / float(ViewX))), 0, Width - 1);
            ty0 = FMath::Clamp(int32((float)Height * (float(sy0) / float(ViewY))), 0, Height - 1);
            tx1 = FMath::Clamp(int32((float)Width * (float(sx1) / float(ViewX))), 0, Width - 1);
            ty1 = FMath::Clamp(int32((float)Height * (float(sy1) / float(ViewY))), 0, Height - 1);
        }
    }

    const int32 pxCount = (tx1 - tx0 + 1);
    const int32 pyCount = (ty1 - ty0 + 1);
    const float sxStart = float(sx0), sxEnd = float(sx1);
    const float syStart = float(sy0), syEnd = float(sy1);

    auto SampleLerp = [](int32 idx, int32 count, float a, float b)->float
        {
            if (count <= 1) return 0.5f * (a + b);
            const float t = float(idx) / float(count - 1);
            return FMath::Lerp(a, b, t);
        };

    for (int32 iy = 0; iy < pyCount; ++iy)
    {
        const float sy = SampleLerp(iy, pyCount, syStart, syEnd);
        const int32 ty = ty0 + iy;

        for (int32 ix = 0; ix < pxCount; ++ix)
        {
            const float sx = SampleLerp(ix, pxCount, sxStart, sxEnd);
            const int32 tx = tx0 + ix;

            FVector CamWS, WorldD;
            if (!PC->DeprojectScreenPositionToWorld(sx, sy, CamWS, WorldD))
            {
                Pixels[ty * Width + tx] = Clear; continue;
            }

            // --- on lance notre rayon (renvoie couleur + index + hit WS)
            int32 cellIdx = -1;
            FVector HitWS = FVector::ZeroVector;
            FLinearColor ColLin = FLinearColor::Transparent;

            // Réutilise ta fonction "vérité terrain" qui renvoie HitWS
            FLinearColor tmpColor;
            if (RaycastFirstCell(Feature, CamWS, WorldD, cellIdx, HitWS, tmpColor, StepCm, VMin, VMax, EpsCm, bEnableClip, ClipOriginWS, ClipNormalWS, ClipThicknessCm, bKeepFront))
            {
                /*const FColor C = ColLin.ToFColorSRGB();
                Pixels[ty * Width + tx] = FColor(C.R, C.G, C.B, 255);*/
                ColLin = tmpColor;

                // DEPTH: distance monde caméra->hit
                const float hitDistCm = FVector::Distance(CamWS, HitWS);
                const float aNorm = FMath::Clamp(hitDistCm / FMath::Max(1.f, MaxDepthCm), 0.f, 1.f);
                const uint8 A = (uint8)FMath::RoundToInt(aNorm * 255.f);

                const FColor C = ColLin.ToFColorSRGB();
                Pixels[ty * Width + tx] = FColor(C.R, C.G, C.B, A);
            }
            else
            {
                /*Pixels[ty * Width + tx] = FColor(0, 0, 0, 0);*/
                Pixels[ty * Width + tx] = Clear; // pas de hit => alpha 0
            }
        }
    }

    // Upload
    FTexture2DMipMap& Mip = Tex->GetPlatformData()->Mips[0];
    void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
    FMemory::Memcpy(Data, Pixels.GetData(), Pixels.Num() * sizeof(FColor));
    Mip.BulkData.Unlock();
    Tex->UpdateResource();

    // Pousse les scalars côté MID (si dispo)
    /*
    if (RayVolumeMID)
    {
        RayVolumeMID->SetScalarParameterValue(TEXT("MaxDepthCm"), MaxDepthCm);
        RayVolumeMID->SetScalarParameterValue(TEXT("DepthEpsCm"), 1.0f); // expose en BP si besoin
    }*/

    return Tex;
}



FLinearColor AVTUViewer::RaymarchDebug(EVTUCellFeature Feature,
    const FVector& RayOriginWS,
    const FVector& RayDirWS,
    int32& OutCellIndex,
    float StepCm,
    /*int32 MaxSteps, float Density,*/
    float VMin,
    float VMax,
    float EpsCm,
    bool  bEnableClip,
    FVector ClipOriginWS,
    FVector ClipNormalWS,
    float  ClipThicknessCm,
    bool   bKeepFront)
{
    OutCellIndex = -1;
    const FVTUGrid& G = GridCache;
    if (G.Points.Num() == 0 || G.NumCells() == 0) return FLinearColor(0, 0, 0, 0);

    const TArray<float>* A = FeatureArrayRef(CellFeatures, Feature);
    if (!A || A->Num() != G.NumCells()) return FLinearColor(0, 0, 0, 0);

    // Monde -> Local
    const FTransform Xf = GetActorTransform();
    const FTransform InvXf = Xf.Inverse();
    const FVector oLS = InvXf.TransformPosition(RayOriginWS);
    const FVector dLS = InvXf.TransformVectorNoScale(RayDirWS).GetSafeNormal();

    // Clip plan en LOCAL (même pour clones)
    float tNear = 0.f, tFar = 0.f;
    {
        // On démarre avec +/- inf ; on restreindra par clone bbox plus bas
        tNear = 0.f; tFar = TNumericLimits<float>::Max();
        if (bEnableClip)
        {
            const FVector Nls = InvXf.TransformVectorNoScale(ClipNormalWS).GetSafeNormal();
            const FVector Pls = InvXf.TransformPosition(ClipOriginWS);
            const float a = FVector::DotProduct(Nls, dLS);
            const float b = FVector::DotProduct(Nls, oLS - Pls);
            const float half = FMath::Max(0.f, 0.5f * ClipThicknessCm);

            if (FMath::Abs(a) < 1e-8f)
            {
                const float s = b;
                const bool keep = bKeepFront ? (s >= +half) : (s <= -half);
                if (!keep) return FLinearColor(0, 0, 0, 0);
            }
            else
            {
                if (bKeepFront)
                {
                    const float tCut = (+half - b) / a;
                    if (a > 0) tNear = FMath::Max(tNear, tCut);
                    else       tFar = FMath::Min(tFar, tCut);
                }
                else
                {
                    const float tCut = (-half - b) / a;
                    if (a > 0) tFar = FMath::Min(tFar, tCut);
                    else       tNear = FMath::Max(tNear, tCut);
                }
                if (tNear >= tFar) return FLinearColor(0, 0, 0, 0);
            }
        }
    }

    const float   Eps = (EpsCm > 0.f) ? EpsCm : FMath::Max(0.05f * StepCm, 0.25f);
    const FVector pad(FMath::Max(0.5f * Eps, 0.25f));

    float bestT = TNumericLimits<float>::Max();
    int32 bestCi = -1;
    int32 bestClone = -1;

    auto TryOneClone = [&](int32 cloneIndex, const FVTUGridClone& C)
        {
            // Clip rayon sur la bbox du clone
            float ta = 0.f, tb = 0.f;
            if (!IntersectRayAABB_Slab(oLS, dLS, C.BoundsLS, ta, tb)) return;
            ta = FMath::Max(ta, tNear);
            tb = FMath::Min(tb, tFar);
            if (ta >= tb) return;

            // seg AABB -> query octree du clone
            const FVector p0 = oLS + dLS * ta;
            const FVector p1 = oLS + dLS * tb;
            const FVector mn(FMath::Min(p0.X, p1.X), FMath::Min(p0.Y, p1.Y), FMath::Min(p0.Z, p1.Z));
            const FVector mx(FMath::Max(p0.X, p1.X), FMath::Max(p0.Y, p1.Y), FMath::Max(p0.Z, p1.Z));
            const FBox segBox(mn - pad, mx + pad);

            TArray<int32> candidates;
            C.Octree->QueryAABB(segBox, candidates);
            if (candidates.Num() == 0) return;

            // filtre AABB cellule (utile mais souvent redondant) + test exact
            for (int32 ci : candidates)
            {
                const FBox* cb = C.Octree->BoundsOf(ci);
                if (cb)
                {
                    float t0 = 0.f, t1 = 0.f;
                    if (!IntersectRayAABB_Slab(oLS, dLS, *cb, t0, t1)) continue;
                    if (t1 < ta) continue;
                }

                float tEnt = 0.f;
                if (RayIntersectConvexCell_FirstT_WithPoints(G, C.PointsLS, ci, oLS, dLS, Eps, tEnt))
                {
                    if (tEnt >= ta && tEnt < bestT)
                    {
                        bestT = tEnt;
                        bestCi = ci;
                        bestClone = cloneIndex;
                    }
                }
            }
        };

    // now iterate the clones with their index:
    if (bUseClonedOctrees && GridClones.Num() > 0)
    {
        for (int32 i = 0; i < GridClones.Num(); ++i)
        {
            const FVTUGridClone& C = GridClones[i];
            if (!C.Octree.IsValid()) continue;
            TryOneClone(i, C);
        }
    }
    else
    {
        // Fallback : 1/8 d'origine (si tu veux)
        const FBox BaseBoxLS(G.Bounds.Min * ScaleCm, G.Bounds.Max * ScaleCm);
        float ta = 0.f, tb = 0.f;
        if (IntersectRayAABB_Slab(oLS, dLS, BaseBoxLS, ta, tb))
        {
            ta = FMath::Max(ta, tNear);
            tb = FMath::Min(tb, tFar);
            if (ta < tb)
            {
                const FVector p0 = oLS + dLS * ta;
                const FVector p1 = oLS + dLS * tb;
                const FVector mn(FMath::Min(p0.X, p1.X), FMath::Min(p0.Y, p1.Y), FMath::Min(p0.Z, p1.Z));
                const FVector mx(FMath::Max(p0.X, p1.X), FMath::Max(p0.Y, p1.Y), FMath::Max(p0.Z, p1.Z));
                const FBox segBox(mn - pad, mx + pad);

                TArray<int32> candidates;
                CellOctree->QueryAABB(segBox, candidates);
                for (int32 ci : candidates)
                {
                    const FBox* cb = CellOctree->BoundsOf(ci);
                    if (cb)
                    {
                        float t0 = 0.f, t1 = 0.f;
                        if (!IntersectRayAABB_Slab(oLS, dLS, *cb, t0, t1)) continue;
                        if (t1 < ta) continue;
                    }

                    float tEnt = 0.f;
                    if (RayIntersectConvexCell_FirstT(GridCache, ci, ScaleCm, oLS, dLS, Eps, tEnt))
                    {
                        if (tEnt >= ta && tEnt < bestT)
                        {
                            bestT = tEnt;
                            bestCi = ci;
                            bestClone = -1;
                        }
                    }
                }
            }
        }
    }

    if (bestCi < 0) return FLinearColor(0, 0, 0, 0);

    OutCellIndex = bestCi;
    const float val = (*A)[bestCi];
    LastHitCloneIndex = bestClone;
    LastHitCellIndex = bestCi;
    LastHitPointWS = GetActorTransform().TransformPosition(oLS + dLS * bestT);
    return ScalarToColor(val, VMin, VMax);
}

// dessine les arêtes d’une FBox locale transformée par l’actor transform
static void DrawOBBFromLocalBox(UWorld* W, const FTransform& Xf, const FBox& LB,
    const FColor& Col, float Duration, float Thk)
{
    if (!LB.IsValid || !W) return;
    const FVector mn = LB.Min, mx = LB.Max;
    const FVector C[8] = {
        {mn.X,mn.Y,mn.Z},{mx.X,mn.Y,mn.Z},{mn.X,mx.Y,mn.Z},{mx.X,mx.Y,mn.Z},
        {mn.X,mn.Y,mx.Z},{mx.X,mn.Y,mx.Z},{mn.X,mx.Y,mx.Z},{mx.X,mx.Y,mx.Z}
    };
    FVector P[8]; for (int i = 0; i < 8; ++i) P[i] = Xf.TransformPosition(C[i]);
    auto L = [&](int a, int b) { DrawDebugLine(W, P[a], P[b], Col, false, Duration, 0, Thk); };
    L(0, 1); L(1, 3); L(3, 2); L(2, 0); L(4, 5); L(5, 7); L(7, 6); L(6, 4); L(0, 4); L(1, 5); L(2, 6); L(3, 7);
}

FBox AVTUViewer::GetActiveVolumeBoundsLS() const
{
    if (bUseClonedOctrees && GridClones.Num() > 0)
    {
        FBox U(ForceInitToZero);
        for (const auto& C : GridClones)
            if (C.BoundsLS.IsValid) U += C.BoundsLS;
        if (U.IsValid) return U;
    }
    // fallback = 1/8
    return FBox(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
}

void AVTUViewer::DrawHitWorldBounds(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;
    const FTransform Xf = GetActorTransform();

    if (LastHitCloneIndex >= 0 && GridClones.IsValidIndex(LastHitCloneIndex))
    {
        const auto& C = GridClones[LastHitCloneIndex];
        if (C.BoundsLS.IsValid)
            DrawOBBFromLocalBox(GetWorld(), Xf, C.BoundsLS, Color.ToFColor(true), Duration, Thickness);
        return;
    }
    if (LastHitCloneIndex == -1)
    {
        const FBox LB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
        DrawOBBFromLocalBox(GetWorld(), Xf, LB, Color.ToFColor(true), Duration, Thickness);
    }
}


void AVTUViewer::DrawHitCellBounds(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld() || LastHitCellIndex < 0) return;
    const FTransform Xf = GetActorTransform();

    if (LastHitCloneIndex >= 0 && GridClones.IsValidIndex(LastHitCloneIndex))
    {
        const auto& C = GridClones[LastHitCloneIndex];
        if (C.Octree.IsValid())
        {
            if (const FBox* B = C.Octree->BoundsOf(LastHitCellIndex))
                DrawOBBFromLocalBox(GetWorld(), Xf, *B, Color.ToFColor(true), Duration, Thickness);
        }
        return;
    }
    if (LastHitCloneIndex == -1 && CellOctree.IsValid())
    {
        if (const FBox* B = CellOctree->BoundsOf(LastHitCellIndex))
            DrawOBBFromLocalBox(GetWorld(), Xf, *B, Color.ToFColor(true), Duration, Thickness);
    }
}

void AVTUViewer::DrawWorldBoundsAll(bool bIncludeBase, float Duration, float Thickness)
{
    if (!GetWorld()) return;
    const FTransform Xf = GetActorTransform();

    // base (1/8)
    if (bIncludeBase && GridCache.Points.Num() > 0)
    {
        const FBox LB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
        DrawOBBFromLocalBox(GetWorld(), Xf, LB, FColor::White, Duration, Thickness);
    }

    // clones
    for (int32 i = 0; i < GridClones.Num(); ++i)
    {
        const auto& C = GridClones[i];
        if (!C.Octree.IsValid() || !C.BoundsLS.IsValid) continue;

        static const FColor cols[8] = {
            FColor::Red, FColor::Green, FColor::Blue, FColor::Yellow,
            FColor::Cyan, FColor::Magenta, FColor(255,128,0), FColor(128,255,0)
        };
        DrawOBBFromLocalBox(GetWorld(), Xf, C.BoundsLS, cols[i % 8], Duration, Thickness);
    }
}

void AVTUViewer::DrawCellBoundsAny(int32 CellIndex, bool bIncludeBase, float Duration, float Thickness)
{
    if (!GetWorld()) return;
    const FTransform Xf = GetActorTransform();

    // base
    if (bIncludeBase && CellOctree.IsValid())
    {
        if (const FBox* B = CellOctree->BoundsOf(CellIndex))
            DrawOBBFromLocalBox(GetWorld(), Xf, *B, FColor::White, Duration, Thickness);
    }

    // clones
    for (int32 i = 0; i < GridClones.Num(); ++i)
    {
        const auto& C = GridClones[i];
        if (!C.Octree.IsValid()) continue;
        if (const FBox* B = C.Octree->BoundsOf(CellIndex))
        {
            static const FColor cols[8] = {
                FColor::Red, FColor::Green, FColor::Blue, FColor::Yellow,
                FColor::Cyan, FColor::Magenta, FColor(255,128,0), FColor(128,255,0)
            };
            DrawOBBFromLocalBox(GetWorld(), Xf, *B, cols[i % 8], Duration, Thickness);
        }
    }
}

void AVTUViewer::DrawCellBoundsInClone(int32 CloneIndex, int32 CellIndex, float Duration, float Thickness)
{
    if (!GetWorld()) return;
    if (!GridClones.IsValidIndex(CloneIndex)) return;

    const auto& C = GridClones[CloneIndex];
    if (!C.Octree.IsValid()) return;

    if (const FBox* B = C.Octree->BoundsOf(CellIndex))
        DrawOBBFromLocalBox(GetWorld(), GetActorTransform(), *B, FColor::Orange, Duration, Thickness);
}

// petit helper interne (pas exposé BP)
static void DrawOBBFromLocalBox_Internal(UWorld* W, const FTransform& Xf, const FBox& LB,
    const FColor& Col, float Duration, float Thk)
{
    if (!W || !LB.IsValid) return;

    const FVector mn = LB.Min, mx = LB.Max;
    const FVector C[8] = {
        {mn.X,mn.Y,mn.Z},{mx.X,mn.Y,mn.Z},{mn.X,mx.Y,mn.Z},{mx.X,mx.Y,mn.Z},
        {mn.X,mn.Y,mx.Z},{mx.X,mn.Y,mx.Z},{mn.X,mx.Y,mx.Z},{mx.X,mx.Y,mx.Z}
    };
    FVector P[8]; for (int i = 0; i < 8; ++i) P[i] = Xf.TransformPosition(C[i]);

    auto L = [&](int a, int b) { DrawDebugLine(W, P[a], P[b], Col, false, Duration, 0, Thk); };
    L(0, 1); L(1, 3); L(3, 2); L(2, 0);
    L(4, 5); L(5, 7); L(7, 6); L(6, 4);
    L(0, 4); L(1, 5); L(2, 6); L(3, 7);
}

void AVTUViewer::DrawActiveVolumeOBB(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;

    // Union des clones si actifs, sinon bounding du 1/8
    FBox Active = GetActiveVolumeBoundsLS();
    if (!Active.IsValid) return;

    DrawOBBFromLocalBox_Internal(GetWorld(), GetActorTransform(),
        Active, Color.ToFColor(true), Duration, Thickness);
}

void AVTUViewer::DrawCloneBoundsOBB(int32 CloneIndex, float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;
    if (!GridClones.IsValidIndex(CloneIndex)) return;

    const auto& C = GridClones[CloneIndex];
    if (!C.BoundsLS.IsValid) return;

    DrawOBBFromLocalBox_Internal(GetWorld(), GetActorTransform(),
        C.BoundsLS, Color.ToFColor(true), Duration, Thickness);
}

void AVTUViewer::DrawBaseBoundsOBB(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;
    const FBox LB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
    if (!LB.IsValid) return;

    DrawOBBFromLocalBox_Internal(GetWorld(), GetActorTransform(),
        LB, Color.ToFColor(true), Duration, Thickness);
}

//------------------------------------------------------------Cube
void AVTUViewer::SetRayVolumeBaseMaterial(UMaterialInterface* Mat)
{
    UE_LOG(LogTemp, Warning, TEXT("I'm in : SetRayVolumeBaseMaterial !"));
    RayVolumeMaterial = Mat;
    RayVolumeMID = nullptr; // force recréation

    UE_LOG(LogTemp, Warning, TEXT("RayVolumePMC : %s"), *FString(RayVolumePMC ? "True" : "False"));
    UE_LOG(LogTemp, Warning, TEXT("RayVolumeMaterial : %s"), *FString(RayVolumeMaterial ? "True" : "False"));
    if (RayVolumePMC && RayVolumeMaterial)
    {
        RayVolumeMID = UMaterialInstanceDynamic::Create(RayVolumeMaterial, this);
        UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : %s"), *FString(RayVolumeMID ? "True" : "False"));
        if (RayVolumeMID)
            RayVolumePMC->SetMaterial(0, RayVolumeMID);
    }
}

//void AVTUViewer::SetRayVolumeBaseMaterial(UMaterialInterface* Mat)
//{
//    UE_LOG(LogTemp, Warning, TEXT("I'm in : SetRayVolumeBaseMaterial !"));
//    RayVolumeMaterial = Mat;
//    RayVolumeMID = nullptr;
//
//    UE_LOG(LogTemp, Warning, TEXT("RayVolumePMC : %s"), *FString(RayVolumePMC ? "True" : "False"));
//    UE_LOG(LogTemp, Warning, TEXT("RayVolumeMaterial : %s"), *FString(RayVolumeMaterial ? "True" : "False"));
//    if (RayVolumePMC && RayVolumeMaterial)
//    {
//        if (UMaterialInstanceDynamic* AsMID = Cast<UMaterialInstanceDynamic>(RayVolumeMaterial))
//        {
//            UE_LOG(LogTemp, Warning, TEXT("Cast : Success !"));
//            RayVolumeMID = AsMID; // on réutilise tel quel
//        }
//        else
//        {
//            UE_LOG(LogTemp, Warning, TEXT("Cast : failed !"));
//            RayVolumeMID = UMaterialInstanceDynamic::Create(RayVolumeMaterial, this);
//        }
//
//        UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : %s"), *FString(RayVolumeMID ? "True" : "False"));
//        if (RayVolumeMID)
//        {
//            RayVolumePMC->SetMaterial(0, RayVolumeMID);
//        }
//    }
//}

//void AVTUViewer::SpawnRayVolumeProxy(bool bTwoSided)
//{
//    UE_LOG(LogTemp, Warning, TEXT("I'm in : SpawnRayVolumeProxy !"));
//    // (Re)création du PMC si nécessaire
//    if (!RayVolumePMC)
//    {
//        RayVolumePMC = NewObject<UProceduralMeshComponent>(this, TEXT("RayVolumePMC"));
//        RayVolumePMC->SetupAttachment(RootComponent);
//        RayVolumePMC->RegisterComponent();
//        RayVolumePMC->SetCollisionEnabled(ECollisionEnabled::NoCollision);
//        RayVolumePMC->SetCastShadow(false);
//        RayVolumePMC->bUseAsyncCooking = true;
//    }
//
//    // Boîte locale active = union des clones si activés, sinon 1/8
//    const FBox ActiveLS = GetActiveVolumeBoundsLS();
//    if (!ActiveLS.IsValid) return;
//
//    const FVector Center = ActiveLS.GetCenter();     // cm (local à l’actor)
//    const FVector Extent = ActiveLS.GetExtent(); // demi-tailles
//
//    // Génère la géométrie d’un box unit (autour de l’origine) avec ces demi-tailles
//    TArray<FVector> V; TArray<int32> I; TArray<FVector> N; TArray<FVector2D> UV; TArray<FProcMeshTangent> T;
//    UKismetProceduralMeshLibrary::GenerateBoxMesh(Extent, V, I, N, UV, T);
//
//    // Positionne le PMC au centre de la boîte (en local acteur)
//    RayVolumePMC->SetRelativeLocation(Center);
//    RayVolumePMC->SetRelativeRotation(FRotator::ZeroRotator);
//    RayVolumePMC->SetRelativeScale3D(FVector(1));
//
//    // Crée/Remplace la section
//    TArray<FLinearColor> DummyC; DummyC.SetNumZeroed(V.Num());
//    RayVolumePMC->CreateMeshSection_LinearColor(
//        /*SectionIndex*/0, V, I, N, UV, DummyC, T, /*bCreateCollision*/false);
//
//    // Matériau
//    UE_LOG(LogTemp, Warning, TEXT("RayVolumeMaterial : %s"), *FString(RayVolumeMaterial ? "True" : "False"));
//    if (RayVolumeMaterial)
//    {
//        RayVolumeMID = UMaterialInstanceDynamic::Create(RayVolumeMaterial, this);
//        UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : %s"), *FString(RayVolumeMID ? "True" : "False"));
//        if (RayVolumeMID)
//        {
//            UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : I'm set !"));
//            // Deux faces conseillé pour voir "l’intérieur" du cube
//            RayVolumeMID->SetScalarParameterValue(TEXT("TwoSidedHack"), bTwoSided ? 1.f : 0.f);
//            RayVolumePMC->SetMaterial(0, RayVolumeMID);
//        }
//    }
//}

void AVTUViewer::SpawnRayVolumeProxy(bool bTwoSided)
{
    UE_LOG(LogTemp, Warning, TEXT("I'm in : SpawnRayVolumeProxy !"));
    if (!RayVolumePMC)
    {
        RayVolumePMC = NewObject<UProceduralMeshComponent>(this, TEXT("RayVolumePMC"));
        RayVolumePMC->SetupAttachment(RootComponent);
        RayVolumePMC->RegisterComponent();
        RayVolumePMC->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        RayVolumePMC->SetCastShadow(false);
        RayVolumePMC->bUseAsyncCooking = true;
    }

    const FBox ActiveLS = GetActiveVolumeBoundsLS();
    if (!ActiveLS.IsValid) return;

    const FVector Center = ActiveLS.GetCenter();
    const FVector Extent = ActiveLS.GetExtent();

    TArray<FVector> V; TArray<int32> I; TArray<FVector> N; TArray<FVector2D> UV; TArray<FProcMeshTangent> T;
    UKismetProceduralMeshLibrary::GenerateBoxMesh(Extent, V, I, N, UV, T);

    RayVolumePMC->SetRelativeLocation(Center);
    RayVolumePMC->SetRelativeRotation(FRotator::ZeroRotator);
    RayVolumePMC->SetRelativeScale3D(FVector(1));

    TArray<FLinearColor> DummyC; DummyC.SetNumZeroed(V.Num());
    RayVolumePMC->CreateMeshSection_LinearColor(0, V, I, N, UV, DummyC, T, false);

    //UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : %s"), *FString(RayVolumeMID ? "True" : "False"));
    //if (RayVolumeMID)
    //{
    //    UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : I'm set !"));
    //    //RayVolumeMID->SetScalarParameterValue(TEXT("TwoSidedHack"), bTwoSided ? 1.f : 0.f);
    //    RayVolumePMC->SetMaterial(0, RayVolumeMID);
    //}
}


void AVTUViewer::DestroyRayVolumeProxy()
{
    if (RayVolumePMC)
    {
        RayVolumePMC->DestroyComponent();
        RayVolumePMC = nullptr;
    }
    RayVolumeMID = nullptr;
}

void AVTUViewer::SetRayVolumeTexture(UTexture2D* RayTex, float Opacity, float Gain)
{
    if (!RayVolumeMID)
    {
        UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID nul : appelle d'abord SetRayVolumeBaseMaterial() puis SpawnRayVolumeProxy()"));
        return;
    }
    RayVolumeMID->SetTextureParameterValue(TEXT("RayTex"), RayTex);
    RayVolumeMID->SetScalarParameterValue(TEXT("Opacity"), Opacity);
    RayVolumeMID->SetScalarParameterValue(TEXT("Gain"), Gain);
}

void AVTUViewer::SetRayVolumeParams(/*float InMaxDepthCm, float InDepthEpsCm,*/ float InOpacity, float InGain)
{
    if (!RayVolumeMID) return;
    RayVolumeMID->SetScalarParameterValue(TEXT("Opacity"), InOpacity);
    RayVolumeMID->SetScalarParameterValue(TEXT("Gain"), InGain);
}

//UTextureRenderTarget2D* AVTUViewer::RaymarchGPU_ToRT(int32 Width, int32 Height)
//{
//    RayRT = VTU::RaymarchGPU_CreateOrUpdateRT(RayRT, Width, Height);
//    return RayRT;
//}

//float AVTUViewer::ComputeActiveMaxDepthCm() const
//{
//    const FBox LS = GetActiveVolumeBoundsLS();
//    if (!LS.IsValid) return 0.f;
//    const FTransform Xf = GetActorTransform();
//    const FVector WMin = Xf.TransformPosition(LS.Min);
//    const FVector WMax = Xf.TransformPosition(LS.Max);
//    return FVector::Distance(WMin, WMax);
//}

void AVTUViewer::PushProxyBoundsToGPU_FromPMC()
{
    UE_LOG(LogTemp, Warning, TEXT("I'm in : PushProxyBoundsToGPU_FromPMC"));

    if (!PMC) { UE_LOG(LogTemp, Warning, TEXT("PMC is null")); return; }

    const FTransform ATW = PMC->GetComponentTransform();

    // Bounds monde du PMC (FBoxSphereBounds contient *aussi* la AABB)
    const FBoxSphereBounds WS = PMC->CalcBounds(ATW);
    const FBox BoxWS = WS.GetBox();
    UE_LOG(LogTemp, Warning, TEXT("[Push] PMC BoundsWS Min=(%.2f,%.2f,%.2f) Max=(%.2f,%.2f,%.2f)"), BoxWS.Min.X, BoxWS.Min.Y, BoxWS.Min.Z, BoxWS.Max.X, BoxWS.Max.Y, BoxWS.Max.Z);

    UGameInstance* GI = GetGameInstance();
    if (!GI) { UE_LOG(LogTemp, Warning, TEXT("No GameInstance")); return; }

    UCustomShaderSubsystem* Sub = GI->GetSubsystem<UCustomShaderSubsystem>();
    if (!Sub) { UE_LOG(LogTemp, Warning, TEXT("CustomShaderSubsystem missing")); return; }

    Sub->SetRaymarchVolumeTransform(ATW);
    Sub->SetRaymarchBoundsWS(BoxWS.Min, BoxWS.Max);
}

void AVTUViewer::BindRaymarchRTToMID()
{
    if (!RayVolumeMID) return;

    if (UGameInstance* GI = GetGameInstance())
    {
        if (UCustomShaderSubsystem* Sub = GI->GetSubsystem<UCustomShaderSubsystem>())
        {
            if (UTextureRenderTarget2D* RT = Sub->GetRaymarchRT())
            {
                RayVolumeMID->SetTextureParameterValue(TEXT("RaymarchTex"), RT);
            }
        }
    }
}

void AVTUViewer::PushGridToGPU()
{
    // Supposons que ta grille CPU s’appelle RawGrid / Grid / CurrentGrid (adapte le nom)
    const FVTUGrid& Grid = GridCache;

    UE_LOG(LogTemp, Warning, TEXT("[Viewer] PushGridToGPU: Points=%d Cells=%d"),
        Grid.Points.Num(), Grid.NumCells());

    if (UGameInstance* GI = GetGameInstance())
    {
        if (UCustomShaderSubsystem* Sub = GI->GetSubsystem<UCustomShaderSubsystem>())
        {
            const bool bOK = Sub->UploadVTUGrid(Grid, ScaleCm);
            UE_LOG(LogTemp, Warning, TEXT("[Viewer] UploadVTUGrid -> %s"), bOK ? TEXT("OK") : TEXT("FAILED"));
        }
    }
}