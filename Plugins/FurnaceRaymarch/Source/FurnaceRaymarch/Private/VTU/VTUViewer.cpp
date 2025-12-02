#include "VTU/VTUViewer.h"
#include "Misc/Paths.h"
#include "Misc/MessageDialog.h"
#include "Containers/Array.h"
#include "Misc/AssertionMacros.h"
#include "KismetProceduralMeshLibrary.h"
#include "ProceduralMeshComponent.h"
#include "CustomShaderSubsystem.h" // plugin
#include "Engine/GameInstance.h"

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
//void AVTUViewer::Probe_FileInfo(const FString& Path)
//{
//    int64 Size = 0; bool bUG = false, bAscii = false; FString FirstKB;
//    const bool ok = VTUCore::Probe_FileInfo(Path, Size, bUG, bAscii, FirstKB);
//    UE_LOG(LogTemp, Display, TEXT("[VTU:BP A] ok=%d size=%lld UG=%d ascii=%d"), ok, Size, bUG, bAscii);
//}
//
//void AVTUViewer::Probe_ParseHeaders(const FString& Path)
//{
//    int32 nPts = 0, nCells = 0, nConn = 0, nFaces = 0, nFoffs = 0; bool bPoly = false; float sec = 0;
//    const bool ok = VTUCore::Probe_ParseHeaders_ASCII(Path, nPts, nCells, nConn, nFaces, nFoffs, bPoly, sec);
//    UE_LOG(LogTemp, Display, TEXT("[VTU:BP B] ok=%d Pts=%d Cells=%d Conn=%d Faces=%d FOffs=%d Poly=%d (%.2fs)"),
//        ok, nPts, nCells, nConn, nFaces, nFoffs, bPoly, sec);
//}
//
//void AVTUViewer::Probe_ParseHeaders_Light(const FString& Path)
//{
//    int32 nPts = 0, nCells = 0; bool hasC = false, hasO = false, hasT = false, hasF = false, hasFo = false; float sec = 0;
//    const bool ok = VTUCore::Probe_ParseHeaders_Light(Path, nPts, nCells, hasC, hasO, hasT, hasF, hasFo, sec);
//    UE_LOG(LogTemp, Display, TEXT("[VTU:BP B] ok=%d NumPoints=%d NumCells=%d | conn=%d offs=%d types=%d faces=%d faceoffs=%d (%.2fs)"),
//        ok, nPts, nCells, hasC, hasO, hasT, hasF, hasFo, sec);
//}
//
//void AVTUViewer::Probe_ParseSamples(const FString& Path, int32 MaxPointTokens, int32 MaxConnTokens)
//{
//    int32 pTok = 0, cTok = 0; float sP = 0, sC = 0;
//    const bool ok = VTUCore::Probe_ParseSamples_ASCII(Path, MaxPointTokens, MaxConnTokens, pTok, cTok, sP, sC);
//    UE_LOG(LogTemp, Display, TEXT("[VTU:BP C] ok=%d Ptokens=%d (%.2fs) Ctokens=%d (%.2fs)"),
//        ok, pTok, sP, cTok, sC);
//}
//
//void AVTUViewer::Probe_ReadOffsetsLast(const FString& Path, int32 NumCells)
//{
//    int64 last = 0; float sec = 0;
//    const bool ok = VTUCore::ReadOffsetsLast_ASCII(Path, NumCells, last, sec);
//    UE_LOG(LogTemp, Display, TEXT("[VTU:BP Offs] ok=%d lastOffset=%lld (%.2fs)"), ok, last, sec);
//}
//
//void AVTUViewer::Probe_ScanCountsAndOffsets(const FString& Path)
//{
//    int32 NumPoints = 0, NumCells = 0;
//    bool  bHasConn = false, bHasOffs = false, bHasTypes = false, bHasFaces = false, bHasFaceOffs = false;
//    int64 ConnLen = 0;
//    float SecsHeaders = 0.f, SecsOffsets = 0.f;
//
//    const bool ok = VTUCore::Probe_ScanCountsAndOffsets(
//        Path,
//        NumPoints, NumCells,
//        bHasConn, bHasOffs, bHasTypes, bHasFaces, bHasFaceOffs,
//        ConnLen,
//        SecsHeaders, SecsOffsets
//    );
//
//    UE_LOG(LogTemp, Display,
//        TEXT("[VTU:BP ALL] ok=%d | Points=%d Cells=%d | conn=%d offs=%d types=%d faces=%d faceoffs=%d | ConnLen=%lld | headers=%.2fs offsets=%.2fs"),
//        ok ? 1 : 0,
//        NumPoints, NumCells,
//        bHasConn ? 1 : 0, bHasOffs ? 1 : 0, bHasTypes ? 1 : 0, bHasFaces ? 1 : 0, bHasFaceOffs ? 1 : 0,
//        ConnLen, SecsHeaders, SecsOffsets);
//
//    if (GEngine)
//    {
//        GEngine->AddOnScreenDebugMessage(
//            -1, 6.f, ok ? FColor::Green : FColor::Red,
//            FString::Printf(TEXT("VTU: ok=%d Pts=%d Cells=%d ConnLen=%lld (%.2fs + %.2fs)"),
//                ok ? 1 : 0, NumPoints, NumCells, ConnLen, SecsHeaders, SecsOffsets));
//    }
//}

//----------------------------------------------------------------Load VTU and Predicted Vector------------------------------------------------------
// Charge un VTU depuis un cache binaire si disponible, sinon parse le .vtu,
// met à jour le cache, puis "affiche" la peau (surface) dans un ProceduralMeshComponent (PMC).
// Retourne true si la construction/affichage de la surface a réussi.
bool AVTUViewer::LoadOrParseVTUAndShow(const FString& VtuPath, const FString& CacheFilePath)
{
    // 1) Résolution du chemin de cache (si non fourni, on utilise Saved/VTUCache/furnace.vbin)
    FString CachePath = CacheFilePath;
    if (CachePath.IsEmpty()) {
        CachePath = FPaths::ProjectSavedDir() / TEXT("VTUCache/furnace.vbin");
    }

    bool bLoadedFromCache = false;
    // 2) Si le cache existe, on tente de charger la grille (points, connectivité, etc.) depuis le binaire
    if (FPaths::FileExists(CachePath)) {
        UE_LOG(LogTemp, Display, TEXT("VTU: trying cache '%s'"), *CachePath);
        bLoadedFromCache = VTUCore::LoadGridBinary(CachePath, GridCache);
        if (!bLoadedFromCache) {
            // Le fichier de cache est présent mais invalide / incompatible : on retombe sur le parsing du .vtu
            UE_LOG(LogTemp, Warning, TEXT("VTU: cache invalid, will re-parse VTU"));
        }
    }

    // 3) Si pas de cache valide, on parse le VTU (lecture ASCII en streaming pour limiter la mémoire)
    if (!bLoadedFromCache) {
        UE_LOG(LogTemp, Display, TEXT("VTU: parsing '%s' (streaming)"), *VtuPath);
        if (!VTUCore::LoadVTU_ASCII_Streaming(VtuPath, GridCache, /*bLoadFacesIfPoly=*/true)) {
            // Parsing échoué : on log et on sort en échec
            UE_LOG(LogTemp, Error, TEXT("VTU: parse failed"));
            return false;
        }

        // 4) Si parsing OK, on (ré)écrit un cache binaire pour les prochaines exécutions
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

    // 5) "Show" : on construit la surface (section 0) dans le ProceduralMeshComponent cible.
   //    - PMC doit être valide
   //    - BuildSurfaceToPMC transforme les données du VTU (faces/triangles) en mesh procédural visible.
    if (!PMC) return false;
    const bool okSurf = VTUCore::BuildSurfaceToPMC(GridCache, PMC, ScaleCm);
    UE_LOG(LogTemp, Display, TEXT("VTU: surface %s (Pts=%d Cells=%d Conn=%d)"),
        okSurf ? TEXT("OK") : TEXT("FAIL"),
        GridCache.Points.Num(), GridCache.NumCells(), GridCache.Connectivity.Num());
    return okSurf;
}

// Variante "chargement uniquement" : charge depuis le cache si possible,
// sinon parse le .vtu et réécrit le cache. Ne fait aucun affichage/PMC.
// Retourne true si la grille (GridCache) est disponible en sortie.
bool AVTUViewer::LoadOrParseVTU(const FString& VtuPath, const FString& CacheFilePath)
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

    return true;
}

// Charge un vecteur de prédictions (par cellule) au format .npy et le stocke dans CellFeatures.
// Le nombre d’éléments attendus doit correspondre au nombre de cellules de la grille déjà chargée.
bool AVTUViewer::LoadPredictedVectorNPY(const FString& NpyPath)
{
    const double T0 = FPlatformTime::Seconds();

    // 1) Vérifie qu’une grille est déjà chargée
    if (GridCache.NumCells() <= 0) {
        UE_LOG(LogTemp, Error, TEXT("LoadPredictedVectorNPY: grid not loaded"));
        return false;
    }

	// 2) Charge le .npy dans CellFeatures
	const bool bOk = VTUCore::LoadNPY_CellFeatures(NpyPath, GridCache.NumCells(), CellFeatures);

    const double Ms = (FPlatformTime::Seconds() - T0) * 1000.0;
    UE_LOG(LogTemp, Display, TEXT("LoadPredictedVectorNPY: %s in %.2f ms (cells=%d)"), bOk ? TEXT("OK") : TEXT("FAIL"), Ms, GridCache.NumCells());

    return bOk;
}
//----------------------------------------------------------------Basic Color Display & Set Material for PMC------------------------------------------------------
// Retourne un pointeur vers le tableau de scalaires (par cellule) correspondant
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

// Applique une "feature" scalaire (par cellule) sur la surface en colorisant le mesh procédural.
// - Feature : scalaire utilisé
// - VMin/VMax : bornes de la plage utilisé
// - SectionIndex : section PMC à mettre à jour (par ex. 0 pour la peau). 
//   Si la construction colorée échoue, on retombe sur un mesh non coloré.
//
// Retourne true si la construction (colorée ou fallback) a réussi.
bool AVTUViewer::ApplyFeatureToSurface(EVTUCellFeature Feature, float VMin, float VMax, int32 SectionIndex)
{
    // 1) Préconditions : il faut un PMC valide et une grille déjà chargée
    if (!PMC || GridCache.NumCells() <= 0) {
        UE_LOG(LogTemp, Error, TEXT("ApplyFeatureToSurface: grid not loaded"));
        return false;
    }
    // 2) Récupération du tableau de scalaires choisi (un float par cellule)
    const TArray<float>* CellS = FeatureArrayRef(CellFeatures, Feature);
    if (!CellS || CellS->Num() != GridCache.NumCells()) {
        // Cohérence stricte : on attend exactement un scalaire par cellule
        UE_LOG(LogTemp, Error, TEXT("ApplyFeatureToSurface: feature array invalid (got %d, need %d)"),
            CellS ? CellS->Num() : -1, GridCache.NumCells());
        return false;
    }

    // 3) Tentative de construction colorisée :  - Utilise la connectivité en faces (polyèdres) si disponible (faces/faceoffsets)
    bool ok = VTUCore::BuildSurfaceToPMC_WithCellScalars(GridCache, PMC, ScaleCm, *CellS, VMin, VMax, SectionIndex);

    //  4) Fallback : si échec (pas de faces/faceoffsets), on affiche la surface “blanche” existante.
    if (!ok) {
        UE_LOG(LogTemp, Warning, TEXT("ApplyFeatureToSurface: falling back to non-colored surface (no poly faces?)"));
        ok = VTUCore::BuildSurfaceToPMC(GridCache, PMC, ScaleCm);
    }
    return ok;
}

// Assigne un matériau à une section de mesh procédural (PMC).
// - Material : matériau UE à appliquer
// - SectionIndex : index de section. 
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
        // -1 => on couvre les sections utiliser : 0 (peau), 1 (slice/alt surface)
        Apply(0);
        Apply(1);
    }
    return true;
}

//-----------------------------------------------------Octree---------------------------------------------------------
// Construit un octree de cellules (sur CPU) à partir de la grille chargée.
// L’octree stocke des "proxies" par cellule : un AABB + l’index de cellule.
// Les AABB sont calculés en unités monde (cm) puis l’octree est bâti sur ces boîtes.
bool AVTUViewer::BuildCellOctree()
{
    if (GridCache.NumCells() <= 0) {
        UE_LOG(LogTemp, Warning, TEXT("BuildCellOctree: grid is empty"));
        return false;
    }

    // 1) Génération des proxies de cellule
    //    - Pour chaque cellule, on reconstruit son AABB en parcourant sa connectivité.
    //    - On travaille en centimètres (ScaleCm) pour rester cohérent avec l’espace monde UE.
    TArray<FVTUCellProxy> Proxies;
    Proxies.Reserve(GridCache.NumCells());

    int32 start = 0;
    for (int32 ci = 0; ci < GridCache.NumCells(); ++ci)
    {
        const int32 end = GridCache.Offsets[ci]; 
        FBox b(ForceInitToZero);
        // Parcours des indices de sommets de la cellule dans le tableau Connectivity
        for (int32 k = start; k < end; ++k) {
            const int32 vi = GridCache.Connectivity[k];
            if (!GridCache.Points.IsValidIndex(vi)) continue;
            // Étend la boîte avec la position du point, mise à l’échelle en cm
            b += FVector(GridCache.Points[vi]) * ScaleCm;
        }
        start = end; // met à jour le début pour la cellule suivante

        // Si la boîte est valide (au moins un point accumulé), on crée le proxy
        if (b.IsValid) {
            FVTUCellProxy P; P.Bounds = b; P.Cell = ci;
            Proxies.Add(P);
        }
    }

    // 2) Détermine les bornes monde globales (AABB de la grille), également en cm
    const FBox WorldB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);

    // 3) Construction de l’octree
    //    - MaxPerLeaf = 32 : nombre max d’éléments par feuille avant subdivision
    //    - MaxDepth   = 10 : profondeur max pour éviter une subdivision excessive
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

// Libère l’octree des cellules (UniquePtr -> Reset), utile lors d’un rechargement ou d’un clear.
void AVTUViewer::ClearCellOctree()
{
    if (CellOctree.IsValid()) {
        CellOctree.Reset();
        UE_LOG(LogTemp, Display, TEXT("VTU Octree: cleared"));
    }
}

//-----------------------------------------------------Clone Octree---------------------------------------------------------

// Définit le pivot local (en centimètres, dans l'espace LOCAL de l'acteur) autour duquel
// on appliquera les flips X/Y et l'éventuel échange X<->Y lors de la création des clones.
// Ce pivot sert de centre de symétrie/rotation pour les miroirs 2D.
void AVTUViewer::SetClonePivotLocal(const FVector& PivotLS)
{
    ClonePivotLS = PivotLS; // typiquement (0,0,0)
}

// Supprime tous les clones construits (points transformés + octrees).
void AVTUViewer::ClearClonedOctrees()
{
    GridClones.Empty();
}

/// Applique une transformation XY autour d'un pivot local :
//  - bSwapXY : échange les axes X et Y (rotation de 90° si on le combine à des flips)
//  - bFlipX  : symétrie par rapport à l'axe Y (inversion du signe de X)
//  - bFlipY  : symétrie par rapport à l'axe X (inversion du signe de Y)
// La transformation se fait en coordonnées locales (LS) et est centrée sur PivotLS.
static FORCEINLINE FVector ApplyFlipSwapXY(const FVector& P, const FVector& PivotLS,
    bool bFlipX, bool bFlipY, bool bSwapXY)
{
    FVector d = P - PivotLS; // vecteur relatif au pivot
    if (bSwapXY) Swap(d.X, d.Y);
    if (bFlipX)  d.X = -d.X;
    if (bFlipY)  d.Y = -d.Y;
    return PivotLS + d;
}

// Calcule la boîte englobante axis-aligned (AABB) d'une cellule 'ci', mais en utilisant
// un tableau de points fourni (PtsLS), déjà transformés en local-space cm.
static FBox CellAABB_FromPoints(const FVTUGrid& G, int32 ci, const TArray<FVector3f>& PtsLS)
{
    // Une cellule agrège les sommets dans Connectivity entre[start, end
    const int32 start = (ci == 0 ? 0 : G.Offsets[ci - 1]);
    const int32 end = G.Offsets[ci];
    FBox B(ForceInitToZero); // AABB "vide" au départ
    for (int32 k = start; k < end; ++k)
    {
        const int32 vi = G.Connectivity[k]; // index du sommet global
        if (!PtsLS.IsValidIndex(vi)) continue;
        B += FVector(PtsLS[vi]); // étend la bbox avec le point
    }
	return B; // retourne l'AABB finale
}

// Construit tous les "clones" XY : identité (optionnel), flips X/Y/XY, swap XY et
// leurs combinaisons. Chaque clone contient :
//  - un nuage de points local transformé (OutClone.PointsLS),
//  - sa bbox globale (OutClone.BoundsLS),
//  - un octree de cellules indexant les AABBs par cellule dans ce repère.
// MaxPerLeaf et MaxDepth paramètrent la granularité de l’octree.
bool AVTUViewer::BuildClonedOctrees_XY(bool bIncludeIdentity, int32 MaxPerLeaf, int32 MaxDepth)
{
    GridClones.Empty();

    // instancie un clone avec (fx,fy,sw) et ajoute au tableau si OK.
    auto Add = [&](bool fx, bool fy, bool sw, const TCHAR* tag)->bool
        {
            FVTUGridClone C;
            if (!BuildOneClone_XY(fx, fy, sw, tag, MaxPerLeaf, MaxDepth, C)) return false;
            GridClones.Add(MoveTemp(C));
            return true;
        };

    bool ok = true;
    // Clone "identité" (sans flip/swap)
    if (bIncludeIdentity) ok &= Add(false, false, false, TEXT("Id"));

    // Flips (X, Y, XY) sans swap
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
        // Point d'origine du VTU (mètres ou unités VTU) -> converti en cm (ScaleCm)
        const FVector pLS = FVector(G.Points[i]) * ScaleCm;
        // On applique flip/swap autour du pivot local choisi
        const FVector qLS = ApplyFlipSwapXY(pLS, ClonePivotLS, bFlipX, bFlipY, bSwapXY);
        // Stockage en float32 (FVector3f) côté clone
        OutClone.PointsLS[i] = FVector3f(qLS);
        // Étend la bbox globale du clone
        BB += qLS;
    }
    OutClone.BoundsLS = BB;
    OutClone.Label = Label;

    // 2) Création des proxies de cellules : une AABB par cellule basée sur PointsLS transformés.
    TArray<FVTUCellProxy> Proxies;
    Proxies.Reserve(G.NumCells());
    for (int32 ci = 0; ci < G.NumCells(); ++ci)
    {
        const FBox CB = CellAABB_FromPoints(G, ci, OutClone.PointsLS);
        if (!CB.IsValid) continue; // saute les cellules dégénérées/vides
        FVTUCellProxy pr;
        pr.Cell = ci; // index de la cellule d'origine (1/8)
        pr.Bounds = CB;
        Proxies.Add(pr);
    }

    // 3) Construction de l'octree à partir des proxies + bbox globale du clone.
    OutClone.Octree = MakeShared<FVTUCellOctree>();
    const bool ok = OutClone.Octree->BuildFromProxies(Proxies, OutClone.BoundsLS, MaxPerLeaf, MaxDepth);
    if (!ok)
    {
        OutClone.Octree.Reset();
        return false;
    }

    return true;
}

//-----------------------------------------------------Raymarch---------------------------------------------------------
// Ray <=> AABB (slab test) en LOCAL
// Test d'intersection rapide rayon/boîte axis-aligned (AABB).
// - Ro : origine du rayon (Local Space)
// - Rd : direction du rayon (Local Space)
// - Box : AABB testée (Local Space)
// - OutTMin/OutTMax : paramètres d'entrée/sortie le long du rayon si intersection
// Retourne true si le segment intersecte la box.
static bool IntersectRayAABB_Slab(const FVector& Ro, const FVector& Rd, const FBox& Box, float& OutTMin, float& OutTMax)
{
    // Inverses des composantes direction (évite les divisions par 0)
    const FVector Inv((Rd.X != 0.f) ? 1.f / Rd.X : BIG_NUMBER,
        (Rd.Y != 0.f) ? 1.f / Rd.Y : BIG_NUMBER,
        (Rd.Z != 0.f) ? 1.f / Rd.Z : BIG_NUMBER);

    // T d'intersection avec chaque paire de plans (min/max) selon les axes
    const FVector t1 = (Box.Min - Ro) * Inv;
    const FVector t2 = (Box.Max - Ro) * Inv;

    // Entrée = max des min par axe, Sortie = min des max par axe
    const float tmin = FMath::Max3(FMath::Min(t1.X, t2.X), FMath::Min(t1.Y, t2.Y), FMath::Min(t1.Z, t2.Z));
    const float tmax = FMath::Min3(FMath::Max(t1.X, t2.X), FMath::Max(t1.Y, t2.Y), FMath::Max(t1.Z, t2.Z));
    OutTMin = tmin; OutTMax = tmax;

    // Intersection si la fenêtre [tmin,tmax] existe et coupe t>=0
    return tmax >= FMath::Max(tmin, 0.f);
}

// Colormap Viridis (stops sRGB -> convertis en LINÉAIRE, puis interpolation)
// Source stops (approx.): [ (68,1,84), (59,82,139), (33,145,140), (94,201,98), (253,231,37) ]
// Mappe un scalaire v dans [vmin,vmax] vers une couleur (linéaire) en interpolant 6 stops.
static FLinearColor ScalarToColor(float v, float vmin, float vmax)
{
    if (!(vmax > vmin)) return FLinearColor::Gray;

    // Normalisation 0..1
    float t = (v - vmin) / (vmax - vmin);
    t = FMath::Clamp(t, 0.f, 1.f);

    // Stops sRGB -> stockés en linéaire une fois (static)
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

    // 5 segments (6 stops) -> indice de segment i et interpolation u
    const float f = t * 5.f;                
    const int   i = FMath::Clamp((int)FMath::FloorToFloat(f), 0, 4);
    const float u = f - (float)i;

    // Interpolation linéaire entre les stops i et i+1
    return FMath::Lerp(StopsLin[i], StopsLin[i + 1], u); // A=1 par défaut
}

// Itère les faces d'une cellule (tet/hex/wedge/pyramid/polyhedron) et appelle Fn(fv,nv)
// - fv : pointeur vers la liste d'indices de sommets de la face
// - nv : nombre de sommets de la face
static void ForEachCellFace_VTK(const FVTUGrid& G, int32 ci, TFunctionRef<void(const int32* fv, int32 nv)> Fn)
{
    // Récupère la fenêtre [start,end[ dans Connectivity pour la cellule ci
    const int32 start = (ci == 0 ? 0 : G.Offsets[ci - 1]);
    const int32 end = G.Offsets[ci];
    if (end <= start || !G.Types.IsValidIndex(ci)) return;

    const uint8 ty = G.Types[ci];
    const int32* v = G.Connectivity.GetData() + start;

    // N'appelle le callback que pour des faces valides (>=3 sommets)
    auto FaceCall = [&](const int32* fv, int32 nv) { if (nv >= 3) Fn(fv, nv); };

    switch ((EVTKCellType)ty)
    {
    case EVTKCellType::VTK_TETRA: {
        // 4 faces triangulaires
        int32 f0[3] = { v[0],v[1],v[2] }; FaceCall(f0, 3);
        int32 f1[3] = { v[0],v[3],v[1] }; FaceCall(f1, 3);
        int32 f2[3] = { v[1],v[3],v[2] }; FaceCall(f2, 3);
        int32 f3[3] = { v[0],v[2],v[3] }; FaceCall(f3, 3);
        break;
    }
    case EVTKCellType::VTK_HEXAHEDRON: {
        // 6 faces quad
        int32 f0[4] = { v[0],v[1],v[2],v[3] }; FaceCall(f0, 4);
        int32 f1[4] = { v[4],v[5],v[6],v[7] }; FaceCall(f1, 4);
        int32 f2[4] = { v[0],v[4],v[5],v[1] }; FaceCall(f2, 4);
        int32 f3[4] = { v[1],v[5],v[6],v[2] }; FaceCall(f3, 4);
        int32 f4[4] = { v[2],v[6],v[7],v[3] }; FaceCall(f4, 4);
        int32 f5[4] = { v[3],v[7],v[4],v[0] }; FaceCall(f5, 4);
        break;
    }
    case EVTKCellType::VTK_WEDGE: {
        // 2 faces tri + 3 faces quad
        int32 f0[3] = { v[0],v[1],v[2] }; FaceCall(f0, 3);
        int32 f1[3] = { v[3],v[4],v[5] }; FaceCall(f1, 3);
        int32 f2[4] = { v[0],v[3],v[4],v[1] }; FaceCall(f2, 4);
        int32 f3[4] = { v[1],v[4],v[5],v[2] }; FaceCall(f3, 4);
        int32 f4[4] = { v[2],v[5],v[3],v[0] }; FaceCall(f4, 4);
        break;
    }
    case EVTKCellType::VTK_PYRAMID: {
        // 1 base quad + 4 faces tri
        int32 f0[4] = { v[0],v[1],v[2],v[3] }; FaceCall(f0, 4);
        int32 f1[3] = { v[0],v[4],v[1] };     FaceCall(f1, 3);
        int32 f2[3] = { v[1],v[4],v[2] };     FaceCall(f2, 3);
        int32 f3[3]{ v[2],v[4],v[3] };      FaceCall(f3, 3);
        int32 f4[3]{ v[3],v[4],v[0] };      FaceCall(f4, 3);
        break;
    }
    case EVTKCellType::VTK_POLYHEDRON: {
        // Format polyhedron : G.Faces contient [nFaces,(nv f1, indices...), (nv f2, indices...), ...]
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

            // Copie temporaire de la face (pile/inline allocator)
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
// Méthode "plane clipping" : accumule tEnter/tExit via les demi-espaces des faces.
// - Eps : tolérance inside/outside pour la robustesse.
// - outTEntry : premier t d'entrée si intersection.
static bool RayIntersectConvexCell_FirstT(const FVTUGrid& G, int32 ci, float ScaleCm, const FVector& oLS, const FVector& dLS, float Eps, float& outTEntry)
{
    outTEntry = TNumericLimits<float>::Max();
    if (!G.Types.IsValidIndex(ci)) return false;

    // Centroïde approx. pour orienter les normales (vers l'extérieur)
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

    // Parcours des faces : mise à jour de [tEnter,tExit]
    ForEachCellFace_VTK(G, ci, [&](const int32* fv, int32 nv)
        {
            if (nv < 3) return;
            hadFace = true;

            const FVector p0 = FVector(G.Points[fv[0]]) * ScaleCm;
            const FVector p1 = FVector(G.Points[fv[1]]) * ScaleCm;
            const FVector p2 = FVector(G.Points[fv[2]]) * ScaleCm;

            // Normale géométrique, orientée vers l'extérieur (via le centroïde)
            FVector n = FVector::CrossProduct(p1 - p0, p2 - p0);
            if (!n.Normalize()) return;
            if (FVector::DotProduct(n, C - p0) > 0) n *= -1.f;

            // Intersection rayon/plan : Nd = n·d, No = n·(o-p0)
            const float Nd = FVector::DotProduct(n, dLS);
            const float No = FVector::DotProduct(n, oLS - p0);

            if (FMath::Abs(Nd) < 1e-8f)
            {
                // Rayon parallèle : si côté extérieur -> rejet immédiat
                if (No > Eps) { tEnter = 1.f; tExit = 0.f; return; }
                // Sinon contrainte inactive (plan tangent ou "regardant l'intérieur")
                return;
            }
            // t d'intersection avec le plan
            const float tHit = -No / Nd; 

            // Plan "entrant" (Nd<0) resserre tEnter, sinon resserre tExit
            if (Nd < 0.f) tEnter = FMath::Max(tEnter, tHit); // entrant
            else          tExit = FMath::Min(tExit, tHit); // sortant
        });

    if (!hadFace) return false;
    if (tEnter > tExit) return false; // intervalle vide
    if (tExit < 0.f)  return false; // tout derrière l'origine

    // Vérification finale : le point d'entrée est bien dans tous les demi-espaces
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

            // dist > 0 => côté extérieur -> pas inside
            const float dist = FVector::DotProduct(n, pEnt - p0);
            if (dist > Eps) inside = false;
        });
    if (!inside) return false;

    outTEntry = FMath::Max(0.f, tEnter);
    return true;
}

// Variante "clone" : identique à ci-dessus mais consomme un tableau de points LOCAL (PtsLS)
// déjà transformés (flip/swap/pivot), donc pas de ScaleCm ni accès à G.Points. 
static bool RayIntersectConvexCell_FirstT_WithPoints(const FVTUGrid& G, const TArray<FVector3f>& PtsLS, int32 ci, const FVector& oLS, const FVector& dLS, float Eps, float& outTEntry)
{
    outTEntry = TNumericLimits<float>::Max();
    if (!G.Types.IsValidIndex(ci)) return false;

    const int32 start = (ci == 0 ? 0 : G.Offsets[ci - 1]);
    const int32 end = G.Offsets[ci];
    if (end <= start) return false;

    // Centroïde dans le repère du clone
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
            // oriente la normale vers l'extérieur via le centroïde
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

    // Vérif inside sur le point d'entrée
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

// Lance un rayon en World Space et renvoie la première cellule touchée + couleur du "Feature".
// - Transforme le rayon en Local Space.
// - Parcourt soit les clones (octrees clonés), soit l'octree de base.
// - Pour chaque candidat renvoyé par l'octree (via AABB), fait un test exact cellulaire.
// - Retour : OutCellIndex, OutHitWS (point monde), OutColor (colormap), bool hit.
bool AVTUViewer::RaycastFirstCell(EVTUCellFeature Feature, const FVector& RayOriginWS, const FVector& RayDirWS, int32& OutCellIndex, FVector& OutHitWS, FLinearColor& OutColor, float StepCm, float VMin, float VMax, float EpsCm, bool  bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS, float  ClipThicknessCm, bool bKeepFront)
{
    OutCellIndex = -1; OutHitWS = FVector::ZeroVector; OutColor = FLinearColor(0, 0, 0, 0);

    // Monde -> Local (cm)
    const FTransform Xf = GetActorTransform();
    const FTransform InvXf = Xf.Inverse();
    const FVector oLS = InvXf.TransformPosition(RayOriginWS);
    const FVector dLS = InvXf.TransformVectorNoScale(RayDirWS).GetSafeNormal();

    // Paramètres de robustesse
    int32 ci = -1;
    float eps = (EpsCm > 0.f) ? EpsCm : FMath::Max(0.05f * StepCm, 0.25f);

    // Résout "meilleure" intersection (t minimal) en prenant en compte clipping + octrees
    auto Solve = [&](int32& OutCi, float& OutT)->bool
        {
            const FVTUGrid& G = GridCache;
            const float   Eps = eps;
            const FVector pad(FMath::Max(0.5f * Eps, 0.25f)); // marge pour la query AABB

            // Fenêtre tNear/tFar induite par le plan de coupe optionnel (en local)
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
                    // Rayon parallèle : on vérifie de quel côté on se trouve
                    const float s = b;
                    const bool keep = bKeepFront ? (s >= +half) : (s <= -half);
                    if (!keep) return false;
                }
                else
                {
                    // Cas demi-espace épais (front/back selon bKeepFront)
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
            // Parcours d'un clone (bbox du clone -> query octree -> tests précis)
            auto TryClone = [&](const FVTUGridClone& C)
                {
                    float ta = 0.f, tb = 0.f;
                    if (!IntersectRayAABB_Slab(oLS, dLS, C.BoundsLS, ta, tb)) return;
                    ta = FMath::Max(ta, tNear);
                    tb = FMath::Min(tb, tFar);
                    if (ta >= tb) return;

                    // AABB du segment de rayon dans la fenêtre [ta,tb] (avec padding)
                    const FVector p0 = oLS + dLS * ta;
                    const FVector p1 = oLS + dLS * tb;
                    const FVector mn(FMath::Min(p0.X, p1.X), FMath::Min(p0.Y, p1.Y), FMath::Min(p0.Z, p1.Z));
                    const FVector mx(FMath::Max(p0.X, p1.X), FMath::Max(p0.Y, p1.Y), FMath::Max(p0.Z, p1.Z));
                    const FBox segBox(mn - pad, mx + pad);

                    // Candidats depuis l'octree cloné
                    TArray<int32> candidates;
                    C.Octree->QueryAABB(segBox, candidates);
                    for (int32 cidx : candidates)
                    {
                        // Filtre optionnel AABB cellule->rayon
                        const FBox* cb = C.Octree->BoundsOf(cidx);
                        if (cb)
                        {
                            float t0 = 0.f, t1 = 0.f;
                            if (!IntersectRayAABB_Slab(oLS, dLS, *cb, t0, t1)) continue;
                            if (t1 < ta) continue;
                        }

                        // Test exact face-par-face avec points du clone
                        float tEnt = 0.f;
                        if (RayIntersectConvexCell_FirstT_WithPoints(G, C.PointsLS, cidx, oLS, dLS, Eps, tEnt))
                        {
                            if (tEnt >= ta && tEnt < bestT) { bestT = tEnt; bestCi = cidx; }
                        }
                    }
                };

            // Choix : clones si disponibles, sinon 1/8 d'origine
            if (bUseClonedOctrees && GridClones.Num() > 0)
            {
                for (const auto& C : GridClones) if (C.Octree.IsValid()) TryClone(C);
            }
            else
            {
                // Fallback (1/8) : mêmes étapes avec l'octree de base
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

    // Résout la première cellule touchée et son t local
    float tHit = 0.f;
    if (!Solve(OutCellIndex, tHit)) return false;

    // Récupère la valeur scalaire et la convertit en couleur
    const TArray<float>* Arr = FeatureArrayRef(CellFeatures, Feature);
    const float v = (*Arr)[OutCellIndex];
    OutColor = ScalarToColor(v, VMin, VMax);

    // Convertit le hit en World Space
    const FVector hitLS = oLS + dLS * tHit;
    OutHitWS = Xf.TransformPosition(hitLS);
    return true;
}

// Fait un "raymarch" CPU par pixel et remplit une texture 2D (BGRA8):
// - Pour chaque pixel, projette en rayon monde (depuis la caméra).
// - Utilise RaycastFirstCell pour récupérer la première cellule touchée.
// - Colorise avec la colormap. L'ALPHA encode la profondeur normalisée (caméra->hit).
// - Option "bCropToOBB" : restreint la zone d'échantillonnage aux projections de l'OBB actif.
UTexture2D* AVTUViewer::RaymarchToTexture2D(int32 Width, int32 Height, EVTUCellFeature Feature, float StepCm, float VMin, float VMax, float EpsCm, bool bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS, float ClipThicknessCm, bool bKeepFront, bool bCropToOBB, bool bClearTransparent)
{
    if (Width <= 0 || Height <= 0) return nullptr;
    if (!GetWorld()) return nullptr;

    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC || !PC->PlayerCameraManager) return nullptr;

    // Récupère la taille du viewport (pour mapper indices pixels -> coords écran)
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

    // Création de la texture cible (BGRA8, sRGB)
    UTexture2D* Tex = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8);
    Tex->SRGB = true;

    // Buffer CPU des pixels
    TArray<FColor> Pixels;
    Pixels.SetNumUninitialized(Width * Height);
    const FColor Clear = bClearTransparent ? FColor(0, 0, 0, 0) : FColor(0, 0, 0, 255);
    for (int i = 0; i < Pixels.Num(); ++i) Pixels[i] = Clear;

    // plein écran par défaut
    int32 sx0 = 0, sy0 = 0, sx1 = ViewX - 1, sy1 = ViewY - 1;
    int32 tx0 = 0, ty0 = 0, tx1 = Width - 1, ty1 = Height - 1;

    if (bCropToOBB)
    {
        // Projette sur l'écran les 8 coins de la boîte active (Local->World->Screen)
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
        // Si projection partielle : clamp dans le viewport + mapping vers la texture
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

    // Taille de la fenêtre de rendu (cadrage) dans la tex
    const int32 pxCount = (tx1 - tx0 + 1);
    const int32 pyCount = (ty1 - ty0 + 1);
    const float sxStart = float(sx0), sxEnd = float(sx1);
    const float syStart = float(sy0), syEnd = float(sy1);

    // Interpolation linéaire d'une coordonnée écran entre [start,end]
    auto SampleLerp = [](int32 idx, int32 count, float a, float b)->float
    {
        if (count <= 1) return 0.5f * (a + b);
        const float t = float(idx) / float(count - 1);
        return FMath::Lerp(a, b, t);
    };

    // Boucle par pixel : déprojection -> rayon monde -> RaycastFirstCell -> couleur/alpha
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

            // --- Lancement du rayon -> récup du hit et de la couleur
            int32 cellIdx = -1;
            FVector HitWS = FVector::ZeroVector;
            FLinearColor ColLin = FLinearColor::Transparent;

            FLinearColor tmpColor;
            if (RaycastFirstCell(Feature, CamWS, WorldD, cellIdx, HitWS, tmpColor, StepCm, VMin, VMax, EpsCm, bEnableClip, ClipOriginWS, ClipNormalWS, ClipThicknessCm, bKeepFront))
            {
                ColLin = tmpColor;

                // DEPTH en alpha : distance caméra->hit normalisée par la diagonale active
                const float hitDistCm = FVector::Distance(CamWS, HitWS);
                const float aNorm = FMath::Clamp(hitDistCm / FMath::Max(1.f, MaxDepthCm), 0.f, 1.f);
                const uint8 A = (uint8)FMath::RoundToInt(aNorm * 255.f);

                const FColor C = ColLin.ToFColorSRGB();
                Pixels[ty * Width + tx] = FColor(C.R, C.G, C.B, A);
            }
            else
            {
                // Pas de hit -> pixel transparent
                Pixels[ty * Width + tx] = Clear;
            }
        }
    }

    // Upload CPU -> Texture2D
    FTexture2DMipMap& Mip = Tex->GetPlatformData()->Mips[0];
    void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
    FMemory::Memcpy(Data, Pixels.GetData(), Pixels.Num() * sizeof(FColor));
    Mip.BulkData.Unlock();
    Tex->UpdateResource();

    return Tex;
}

// Version "debug" : renvoie uniquement la couleur au premier hit (et met à jour des infos debug).
// - Gère clipping, clones vs 1/8, et retient bestT / bestCi / bestClone.
// - Remplit LastHitCloneIndex / LastHitCellIndex / LastHitPointWS pour debug/test.
FLinearColor AVTUViewer::RaymarchDebug(EVTUCellFeature Feature, const FVector& RayOriginWS, const FVector& RayDirWS, int32& OutCellIndex, float StepCm, float VMin, float VMax, float EpsCm, bool  bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS, float  ClipThicknessCm, bool   bKeepFront)
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

    // Fenêtre tNear/tFar induite par le plan de coupe optionnel
    float tNear = 0.f, tFar = 0.f;
    {
        // Valeurs initiales (pas de limite avant l'application des clones)
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
                // Rayon parallèle : teste de quel côté on est
                const float s = b;
                const bool keep = bKeepFront ? (s >= +half) : (s <= -half);
                if (!keep) return FLinearColor(0, 0, 0, 0);
            }
            else
            {
                // Demi-espace
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

    // Paramètres de robustesse
    const float   Eps = (EpsCm > 0.f) ? EpsCm : FMath::Max(0.05f * StepCm, 0.25f);
    const FVector pad(FMath::Max(0.5f * Eps, 0.25f));

    float bestT = TNumericLimits<float>::Max();
    int32 bestCi = -1;
    int32 bestClone = -1;

    // Teste un clone : bbox -> query octree -> test exact par cellule
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

        // filtre AABB cellule + test exact
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

    // Parcours clones ou fallback 1/8
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
        // Fallback : 1/8 d'origine
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

    // Sorties debug + couleur
    OutCellIndex = bestCi;
    const float val = (*A)[bestCi];
    LastHitCloneIndex = bestClone;
    LastHitCellIndex = bestCi;
    LastHitPointWS = GetActorTransform().TransformPosition(oLS + dLS * bestT);
    return ScalarToColor(val, VMin, VMax);
}

//--------------------------------DebugDraw---------------------------------
// Dessine les 12 arêtes d’un cube à partir d’un tableau de 8 sommets P (ordre standard).
// - P : sommets mondes (8 points, indices 0..7)
static void DrawBoxEdges(UWorld* world, const TArray<FVector>& P, const FColor& color, bool bPersistent, float Duration, float thickness)
{
    // tracer une arête entre deux indices
    auto L = [&](int a, int b) { DrawDebugLine(world, P[a], P[b], color, bPersistent, Duration, 0, thickness); };
    // indices des 12 arêtes d’un cube
    L(0, 1); L(1, 3); L(3, 2); L(2, 0); // bas
    L(4, 5); L(5, 7); L(7, 6); L(6, 4); // haut
    L(0, 4); L(1, 5); L(2, 6); L(3, 7); // montantes
}

// Dessine les arêtes d’une FBox locale (LB) après transformation par Xf (OBB visuellement).
// Convertit les 8 coins de la box locale en monde, puis trace les 12 arêtes.
static void DrawOBBFromLocalBox(UWorld* world, const FTransform& Xf, const FBox& LB, const FColor& Col, float Duration, float thickness)
{
    if (!LB.IsValid || !world) return;
    // 8 coins en Local Space
    const FVector mn = LB.Min, mx = LB.Max;
    const FVector C[8] = {
        {mn.X,mn.Y,mn.Z},{mx.X,mn.Y,mn.Z},{mn.X,mx.Y,mn.Z},{mx.X,mx.Y,mn.Z},
        {mn.X,mn.Y,mx.Z},{mx.X,mn.Y,mx.Z},{mn.X,mx.Y,mx.Z},{mx.X,mx.Y,mx.Z}
    };
    // Transformation en World Space
    FVector P[8]; for (int i = 0; i < 8; ++i) P[i] = Xf.TransformPosition(C[i]);
    // Tracé des 12 arêtes
    auto L = [&](int a, int b) { DrawDebugLine(world, P[a], P[b], Col, false, Duration, 0, thickness); };
    L(0, 1); L(1, 3); L(3, 2); L(2, 0); L(4, 5); L(5, 7); L(7, 6); L(6, 4); L(0, 4); L(1, 5); L(2, 6); L(3, 7);
}

// Retourne les bounds actifs en Local Space :
// - si clones actifs : union des BoundsLS de chaque clone valide,
// - sinon : bounding box du 1/8 (GridCache.Bounds * ScaleCm).
FBox AVTUViewer::GetActiveVolumeBoundsLS() const
{
    if (bUseClonedOctrees && GridClones.Num() > 0)
    {
        FBox U(ForceInitToZero);
        for (const auto& C : GridClones)
            if (C.BoundsLS.IsValid) U += C.BoundsLS;
        if (U.IsValid) return U;
    }
    // fallback = 1/8 (box locale issue du VTU, mise à l’échelle en cm)
    return FBox(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
}

// Dessine la boîte monde correspondant au dernier hit (LastHitCloneIndex) :
// - si on a touché un clone : OBB de ce clone,
// - si on a touché la base (index -1) : OBB du 1/8.
// Sert à visualiser le volume global dans lequel le rayon a intersecté.
void AVTUViewer::DrawHitWorldBounds(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;
    const FTransform Xf = GetActorTransform();

    // Hit dans un clone spécifique
    if (LastHitCloneIndex >= 0 && GridClones.IsValidIndex(LastHitCloneIndex))
    {
        const auto& C = GridClones[LastHitCloneIndex];
        if (C.BoundsLS.IsValid)
            DrawOBBFromLocalBox(GetWorld(), Xf, C.BoundsLS, Color.ToFColor(true), Duration, Thickness);
        return;
    }
    // Hit dans la base (1/8, index -1)
    if (LastHitCloneIndex == -1)
    {
        const FBox LB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
        DrawOBBFromLocalBox(GetWorld(), Xf, LB, Color.ToFColor(true), Duration, Thickness);
    }
}

// Dessine la boîte de la cellule touchée (celle renvoyée par le dernier raycast) :
// - si hit d’un clone : récupère la box de la cellule via l’octree du clone,
// - sinon via l’octree de base.
// Utile pour vérifier la précision de l’intersection.
void AVTUViewer::DrawHitCellBounds(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld() || LastHitCellIndex < 0) return;
    const FTransform Xf = GetActorTransform();

    // Cellule issue d’un clone
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
    // Cellule issue de l’octree de base (1/8)
    if (LastHitCloneIndex == -1 && CellOctree.IsValid())
    {
        if (const FBox* B = CellOctree->BoundsOf(LastHitCellIndex))
            DrawOBBFromLocalBox(GetWorld(), Xf, *B, Color.ToFColor(true), Duration, Thickness);
    }
}

// Dessine l’OBB du volume actif (union des clones si activés, sinon 1/8).
// Pratique pour vérifier le cadrage et le crop côté rendu CPU
void AVTUViewer::DrawActiveVolumeOBB(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;
    FBox Active = GetActiveVolumeBoundsLS();
    if (!Active.IsValid) return;

    DrawOBBFromLocalBox(GetWorld(), GetActorTransform(),
        Active, Color.ToFColor(true), Duration, Thickness);
}

// Dessine l’OBB d’un **clone** particulier (par son index dans GridClones).
// Ignore les indices invalides ou bounds non valides.
void AVTUViewer::DrawCloneBoundsOBB(int32 CloneIndex, float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;
    if (!GridClones.IsValidIndex(CloneIndex)) return;

    const auto& C = GridClones[CloneIndex];
    if (!C.BoundsLS.IsValid) return;

    DrawOBBFromLocalBox(GetWorld(), GetActorTransform(),
        C.BoundsLS, Color.ToFColor(true), Duration, Thickness);
}

// Dessine l’OBB de la box de base (1/8, non clonée).
void AVTUViewer::DrawBaseBoundsOBB(float Duration, float Thickness, FLinearColor Color) const
{
    if (!GetWorld()) return;

    // Bounds locaux du 1/8 (mise à l’échelle cm)
    const FBox LB(GridCache.Bounds.Min * ScaleCm, GridCache.Bounds.Max * ScaleCm);
    if (!LB.IsValid) return;

    DrawOBBFromLocalBox(GetWorld(), GetActorTransform(),
        LB, Color.ToFColor(true), Duration, Thickness);
}

//------------------------------------------------------------Rendering Proxy------------------------------------------------------------
// Définit le matériau de base pour le proxy volumique (cube) et crée une MID si possible.
// - Mat : matériau maître (UMaterialInterface) : sera transformer en une instance dynamique (MID) pour piloter des paramètres.
void AVTUViewer::SetRayVolumeBaseMaterial(UMaterialInterface* Mat)
{
    UE_LOG(LogTemp, Warning, TEXT("I'm in : SetRayVolumeBaseMaterial !"));
    // Stocke le matériau maître, et invalide toute MID existante (sera recréée si possible)
    RayVolumeMaterial = Mat;
    RayVolumeMID = nullptr;

    UE_LOG(LogTemp, Warning, TEXT("RayVolumePMC : %s"), *FString(RayVolumePMC ? "True" : "False"));
    UE_LOG(LogTemp, Warning, TEXT("RayVolumeMaterial : %s"), *FString(RayVolumeMaterial ? "True" : "False"));
    // Si on a déjà un PMC de proxy et un matériau maître, on fabrique une MID et on l’assigne à la section 0
    if (RayVolumePMC && RayVolumeMaterial)
    {
        // La MID permet de changer des paramètres (textures, scalaires…) à l’exécution par instance
        RayVolumeMID = UMaterialInstanceDynamic::Create(RayVolumeMaterial, this);
        UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : %s"), *FString(RayVolumeMID ? "True" : "False"));
        if (RayVolumeMID)
            RayVolumePMC->SetMaterial(0, RayVolumeMID);
    }
}

// Crée (ou recrée) le proxy volumique sous forme d’un cube (ProceduralMeshComponent),
// aligné sur la bounding box **active** (union des clones ou 1/8).
// - bTwoSided : pilote un paramètre du matériau ("TwoSidedHack") pour rendre visibles les faces internes.
void AVTUViewer::SpawnRayVolumeProxy(bool bTwoSided)
{
    UE_LOG(LogTemp, Warning, TEXT("I'm in : SpawnRayVolumeProxy !"));
    // (Re)création du PMC si nécessaire
    if (!RayVolumePMC)
    {
        RayVolumePMC = NewObject<UProceduralMeshComponent>(this, TEXT("RayVolumePMC"));
        RayVolumePMC->SetupAttachment(RootComponent);
        RayVolumePMC->RegisterComponent();
        RayVolumePMC->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        RayVolumePMC->SetCastShadow(false);
        RayVolumePMC->bUseAsyncCooking = true;
    }

    /// Récupère la boîte locale active (LS) : union des clones si actifs, sinon 1/8 (GridCache.Bounds * ScaleCm)
    const FBox ActiveLS = GetActiveVolumeBoundsLS();
    if (!ActiveLS.IsValid) return;

    const FVector Center = ActiveLS.GetCenter(); // centre du volume en local acteur (cm)
    const FVector Extent = ActiveLS.GetExtent(); // demi-tailles 

    // Génère la géométrie d’une box centrée à l’origine (en LS), de demi-tailles "Extent".
    // V, I, N, UV, T sont remplis pour fabriquer une section de mesh
    TArray<FVector> V; TArray<int32> I; TArray<FVector> N; TArray<FVector2D> UV; TArray<FProcMeshTangent> T;
    UKismetProceduralMeshLibrary::GenerateBoxMesh(Extent, V, I, N, UV, T);

    // Positionne le PMC au centre de la boîte (en local acteur)
    RayVolumePMC->SetRelativeLocation(Center);
    RayVolumePMC->SetRelativeRotation(FRotator::ZeroRotator);
    RayVolumePMC->SetRelativeScale3D(FVector(1));

    // Crée/Remplace la section la section 0 
    TArray<FLinearColor> DummyC; DummyC.SetNumZeroed(V.Num());
    RayVolumePMC->CreateMeshSection_LinearColor(
        /*SectionIndex*/0, V, I, N, UV, DummyC, T, /*bCreateCollision*/false);

    // Assigne un matériau si disponible, via une MID (pour exposer des paramètres dynamiques)
    UE_LOG(LogTemp, Warning, TEXT("RayVolumeMaterial : %s"), *FString(RayVolumeMaterial ? "True" : "False"));
    if (RayVolumeMaterial)
    {
        RayVolumeMID = UMaterialInstanceDynamic::Create(RayVolumeMaterial, this);
        UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : %s"), *FString(RayVolumeMID ? "True" : "False"));
        if (RayVolumeMID)
        {
            UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID : I'm set !"));
            // Deux faces conseillé pour voir "l’intérieur" du cube // "TwoSidedHack" est un paramètre matériau (plus utilisé de mémoire)
            RayVolumeMID->SetScalarParameterValue(TEXT("TwoSidedHack"), bTwoSided ? 1.f : 0.f);
            RayVolumePMC->SetMaterial(0, RayVolumeMID);
        }
    }
}

// Détruit le PMC et libère la MID associée (réinitialisation clean).
void AVTUViewer::DestroyRayVolumeProxy()
{
    if (RayVolumePMC)
    {
        RayVolumePMC->DestroyComponent();
        RayVolumePMC = nullptr;
    }
    RayVolumeMID = nullptr;
}

// (CPU) Pousse dans la MID la texture de raymarching (par ex. sortie CPU) pour l’affichage dans le matériau.
// - RayTex : texture 2D (BGRA/RGBA, etc.) que le shader échantillonne via le paramètre "RaymarchTex".
void AVTUViewer::SetRayVolumeTexture(UTexture2D* RayTex)
{
    // Nécessite une MID déjà créée (SetRayVolumeBaseMaterial + SpawnRayVolumeProxy)
    if (!RayVolumeMID)
    {
        UE_LOG(LogTemp, Warning, TEXT("RayVolumeMID nul : appelle d'abord SetRayVolumeBaseMaterial() puis SpawnRayVolumeProxy()"));
        return;
    }
    RayVolumeMID->SetTextureParameterValue(TEXT("RaymarchTex"), RayTex);
}

//--------------------------------Push to GPU Shader---------------------------------
// Transmet au sous-système GPU :
// - la matrice Monde->Local de l’actor 
// - l’AABB du proxy de raymarch (en espace local acteur, cm)
void AVTUViewer::PushProxyBoundsToGPU_FromRayVolumePMC()
{
    UE_LOG(LogTemp, Warning, TEXT("PushProxyBoundsToGPU_FromRayVolumePMC()"));
    if (!RayVolumePMC) { UE_LOG(LogTemp, Warning, TEXT("  RayVolumePMC == null")); return; }

    // Calcule les bounds du PMC dans son espace local propre => AABB local du mesh tel qu’il a été créé 
    const FBoxSphereBounds LBS = RayVolumePMC->CalcBounds(FTransform::Identity);
    const FBox BoxLS = LBS.GetBox();

    // Récupère le transform RELATIF du PMC (position/rotation/échelle par rapport à l’actor)
    // Ici on ne l’utilise que pour translater le box local pour obtenir un box en "actor local".
    const FTransform Rel = RayVolumePMC->GetRelativeTransform();
    const FBox BoxActorLS(
        BoxLS.Min + Rel.GetLocation(),
        BoxLS.Max + Rel.GetLocation());

    /*const FVector MinLS = BoxLS.Min;
    const FVector MaxLS = BoxLS.Max;*/

    // Matrices Local->World et World->Local de l’acteur
    const FMatrix L2W = GetActorTransform().ToMatrixWithScale();
    const FMatrix W2L = L2W.Inverse();
    //const FMatrix W2L = L2W.InverseFast();

    /*const FMatrix CompL2W = RayVolumePMC->GetComponentTransform().ToMatrixWithScale();
    const FMatrix CompW2L = CompL2W.InverseFast();*/

    // Push des constantes vers le sous-système shader (côté GPU)
    if (UGameInstance* GI = GetGameInstance())
        if (UCustomShaderSubsystem* Sub = GI->GetSubsystem<UCustomShaderSubsystem>())
        {
            Sub->SetWorldToLocal(W2L);            // pour transformer rayOrigin/rayDirection -> LS
            //Sub->SetWorldToLocal(CompW2L);
            //Sub->SetRaymarchBoundsLS(MinLS, MaxLS); // AABB LS du proxy

            // Matrice Monde -> Local de l’actor
            //Sub->SetWorldToLocal(GetActorTransform().ToMatrixWithScale().InverseFast());

            // AABB en espace local acteur (cm) du volume dans lequel on raymarche
            Sub->SetRaymarchBoundsLS(BoxActorLS.Min, BoxActorLS.Max);
            UE_LOG(LogTemp, Warning, TEXT("  -> SetWorldToLocal + SetRaymarchBoundsLS (actor space) OK"));
            UE_LOG(LogTemp, Warning, TEXT("  -> LS=ACTOR  Min=(%.2f,%.2f,%.2f)  Max=(%.2f,%.2f,%.2f)"), BoxActorLS.Min.X, BoxActorLS.Min.Y, BoxActorLS.Min.Z, BoxActorLS.Max.X, BoxActorLS.Max.Y, BoxActorLS.Max.Z);
        }
}

// Lie la RenderTarget (écrite par le compute raymarch GPU) au Material Instance Dynamic
// afin que le proxy (PMC) affiche en temps réel la texture produite par le GPU.
void AVTUViewer::BindRaymarchRTToMID()
{
    if (!RayVolumeMID) return;

    if (UGameInstance* GI = GetGameInstance())
    {
        if (UCustomShaderSubsystem* Sub = GI->GetSubsystem<UCustomShaderSubsystem>())
        {
            // Récupère la RT où le compute écrit
            if (UTextureRenderTarget2D* RT = Sub->GetRaymarchRT())
            {
                // Pousse la RT dans le paramètre texture du matériau
                RayVolumeMID->SetTextureParameterValue(TEXT("RaymarchTex"), RT);
            }
        }
    }
}

// Envoie au GPU la géométrie VTU (points/connectivité/offsets/types, échelle cm)
// puis (si disponible) l’octree CPU sérialisé pour l’utiliser comme structure d’accélération côté GPU.
void AVTUViewer::PushGridToGPU()
{
    if (UGameInstance* GI = GetGameInstance())
    {
        if (UCustomShaderSubsystem* Sub = GI->GetSubsystem<UCustomShaderSubsystem>())
        {
            // 1) Upload VTU (une seule fois)
            Sub->UploadVTUGrid(GridCache, ScaleCm);

            // 2) Upload Octree (si construit)
            if (CellOctree.IsValid() && CellOctree->IsBuilt())
            {
                Sub->UploadVTUOctree(*CellOctree);
            }
        }
    }
}

// Met à jour côté GPU :
// - le buffer des valeurs scalaires par cellule (feature choisie)
// - la fenêtre de visualisation (VMin/VMax) pour la colorimétrie/normalisation dans le shader
// - l’epsilon géométrique pour les tests/intersections (tolérances numériques).
void AVTUViewer::SetFeatureSelected(EVTUCellFeature Feat, float InVMin, float InVMax, float InEpsCm)
{
    // Sélectionne le tableau de scalaires correspondant à la feature
    const TArray<float>* Arr = FeatureArrayRef(CellFeatures, Feat);
    if (!Arr || Arr->Num() <= 0) { UE_LOG(LogTemp, Warning, TEXT("Feature array empty")); return; }

    if (UGameInstance* GI = GetGameInstance())
    {
        if (UCustomShaderSubsystem* Sub = GI->GetSubsystem<UCustomShaderSubsystem>())
        {
            // Push des valeurs par cellule (même ordre d’indices que GridCache)
            Sub->UploadFeatureVals(*Arr);
            // Fenêtre de normalisation/affichage
            Sub->SetFeatureWindow(InVMin, InVMax);
            // Tolérance géométrique (en cm)
            Sub->SetRaymarchEpsilon(InEpsCm);
        }
    }
}