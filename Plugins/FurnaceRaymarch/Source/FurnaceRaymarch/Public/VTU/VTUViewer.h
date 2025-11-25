#pragma once

#include "CoreMinimal.h"
#include "VTU/VTUOctree.h"
#include "GameFramework/Actor.h"
#include "VTU/VTUCore.h"
#include "VTUViewer.generated.h"


UENUM(BlueprintType)
enum class EVTUCellFeature : uint8
{
	NOx, CO, OH, H2, H2O, CO2, O2, CH4, T
};

// --- Un clone complet : points transformés (LOCAL cm), bbox, et son octree ---
USTRUCT()
struct FVTUGridClone
{
	GENERATED_BODY()

	// Points transformés (déjà en LOCAL cm)
	TArray<FVector3f> PointsLS;

	// BBox de tout le clone (LOCAL cm)
	FBox BoundsLS = FBox(ForceInitToZero);

	// Octree des cellules du clone (chaque élément = index de cellule du 1/8)
	TSharedPtr<FVTUCellOctree> Octree;

	// Mnémonique (debug)
	FString Label;
};

UCLASS()
class AVTUViewer : public AActor
{
	GENERATED_BODY()
public:
	UPROPERTY(VisibleAnywhere) UProceduralMeshComponent* PMC; // physical mesh component pour afficher la surface
	UPROPERTY(EditAnywhere, Category = "VTU") float ScaleCm = 100.f; // échelle globale (1 unité = X cm)
	UPROPERTY(VisibleAnywhere, Category = "VTU|Features") FVTUCellFeatures CellFeatures;  // 9 tableaux alignés aux cellules

	FVTUGrid GridCache;
	TArray<float> PointScalarsCache; // size = Points.Num()
	AVTUViewer();
	
	//--------------------------------Clone Property---------------------------------
	// Activer/désactiver l’usage des clones
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Clones")
	bool bUseClonedOctrees = false;

	// Tous les clones (Id, FlipX, FlipY, FlipXY, SwapXY * leurs flips) — jusqu’à 8
	TArray<FVTUGridClone> GridClones;

	// Pivot LOCAL (cm) des plans X=0 / Y=0)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Clones")
	FVector ClonePivotLS = FVector::ZeroVector;
	
	//--------------------------------Raymarch Property----------------------------------
	// --- Mémo du dernier hit (alimenté par RaymarchDebug) ---
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Debug")
	int32 LastHitCloneIndex = -999;   // -1 = base (1/8), >=0 = index clone, -999 = aucun

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Debug")
	int32 LastHitCellIndex = -1;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Debug")
	FVector LastHitPointWS = FVector::ZeroVector;

	//--------------------------------Proxy Property----------------------------------
	// == Proxy cube pour afficher la texture de raymarch ==
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "VTU|Proxy")
	UProceduralMeshComponent* RayVolumePMC = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	UMaterialInterface* RayVolumeMaterial = nullptr;

	UPROPERTY(Transient)
	UMaterialInstanceDynamic* RayVolumeMID = nullptr;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	float DefaultDepthEpsCm = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	float DefaultOpacity = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VTU|Proxy")
	float DefaultGain = 1.0f;

	//--------------------------------Probe for import----------------------------------
	/*UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_FileInfo(const FString& Path);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ParseHeaders(const FString& Path);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ParseHeaders_Light(const FString& Path);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ParseSamples(const FString& Path, int32 MaxPointTokens = 30000, int32 MaxConnTokens = 60000);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ReadOffsetsLast(const FString& Path, int32 NumCells);

	UFUNCTION(BlueprintCallable, Category = "VTU|Probe")
	void Probe_ScanCountsAndOffsets(const FString& Path);*/

	//--------------------------------Load for import----------------------------------
	///  Load et parse un VTU (avec cache optionnel) et affiche la surface
	UFUNCTION(BlueprintCallable, Category = "VTU")
	bool LoadOrParseVTUAndShow(const FString& VtuPath, const FString& CacheFilePath);

	/// Load et parse un VTU (avec cache optionnel) sans créer le mesh surface PMC
	UFUNCTION(BlueprintCallable, Category = "VTU")
	bool LoadOrParseVTU(const FString& VtuPath, const FString& CacheFilePath);

	///Load un tableau NPY de scalaires par point (aligné aux points du VTU chargé)
	UFUNCTION(BlueprintCallable, Category = "VTU|Features")
	bool LoadPredictedVectorNPY(const FString& NpyPath);

	//--------------------------------Surface & Materials----------------------------------
	UFUNCTION(BlueprintCallable, Category = "VTU|Features")
	bool ApplyFeatureToSurface(EVTUCellFeature Feature, float VMin, float VMax, int32 SectionIndex = 0);

	UFUNCTION(BlueprintCallable, Category = "VTU|Materials")
	bool SetSurfaceMaterial(UMaterialInterface* Material, int32 SectionIndex = -1); // -1 => toutes les sections utiles
	
	//--------------------------------Octree---------------------------------
	UFUNCTION(BlueprintCallable, Category = "VTU|Octree")
	bool BuildCellOctree();  // à appeler après LoadOrParseVTUAndShow ou LoadOrParseVTU

	UFUNCTION(BlueprintCallable, Category = "VTU|Octree")
	void ClearCellOctree();

	//--------------------------------Clone Octree---------------------------------
	// API
	UFUNCTION(BlueprintCallable, Category = "VTU|Clones")
	void SetClonePivotLocal(const FVector& PivotLS);

	UFUNCTION(BlueprintCallable, Category = "VTU|Clones")
	void ClearClonedOctrees();

	UFUNCTION(BlueprintCallable, Category = "VTU|Clones")
	// Construit 8 clones (Id, FlipX, FlipY, FlipXY, SwapXY + leurs flips)
	bool BuildClonedOctrees_XY(bool bIncludeIdentity = true, int32 MaxPerLeaf = 64, int32 MaxDepth = 12);

	bool BuildOneClone_XY(bool bFlipX, bool bFlipY, bool bSwapXY, const FString& Label, int32 MaxPerLeaf, int32 MaxDepth, FVTUGridClone& OutClone);

	//--------------------------------Raymarch---------------------------------
	// Raymarch CPU de debug (retourne une couleur pour un rayon)
	UFUNCTION(BlueprintCallable, Category = "VTU|Ray")
	FLinearColor RaymarchDebug(EVTUCellFeature Feature, const FVector& RayOriginWS, const FVector& RayDirWS, int32& OutCellIndex, float StepCm, /*int32 MaxSteps, float Density,*/ float VMin, float VMax, float EpsCm, bool bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS, float ClipThicknessCm, bool bKeepFront); // VMin/VMax = 0 => autoscale

	UFUNCTION(BlueprintCallable, Category = "VTU|Ray")
	bool RaycastFirstCell(EVTUCellFeature Feature, const FVector& RayOriginWS, const FVector& RayDirWS, int32& OutCellIndex, FVector& OutHitWS, FLinearColor& OutColor, float StepCm, float VMin, float VMax, float EpsCm, bool  bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS,float  ClipThicknessCm, bool bKeepFront);

	UFUNCTION(BlueprintCallable, Category = "VTU|Ray")
	UTexture2D* RaymarchToTexture2D(int32 Width, int32 Height, EVTUCellFeature Feature, float StepCm, float VMin, float VMax, float EpsCm, bool bEnableClip, FVector ClipOriginWS, FVector ClipNormalWS, float ClipThicknessCm, bool bKeepFront, bool bCropToOBB = true, bool bClearTransparent = true);

	//--------------------------------DebugDraw---------------------------------
	// --- Outils debug ciblés ---
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawHitWorldBounds(float Duration = 5.f, float Thickness = 2.f, FLinearColor Color = FLinearColor::Yellow) const;

	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawHitCellBounds(float Duration = 5.f, float Thickness = 2.f, FLinearColor Color = FLinearColor::Blue) const;

	// Bounds “actifs” du volume pour le rendu (union clones si actifs)
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	FBox GetActiveVolumeBoundsLS() const;

	// Dessine l’OBB du volume actif (union des clones si actifs, sinon le 1/8)
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawActiveVolumeOBB(float Duration = 5.f, float Thickness = 2.f, FLinearColor Color = FLinearColor::Red) const;

	// Dessine l’OBB du clone i (0..GridClones.Num()-1)
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawCloneBoundsOBB(int32 CloneIndex, float Duration = 5.f, float Thickness = 2.f, FLinearColor Color = FLinearColor::Yellow) const;

	// Dessine l’OBB du 1/8 de base
	UFUNCTION(BlueprintCallable, Category = "VTU|Debug")
	void DrawBaseBoundsOBB(float Duration = 5.f, float Thickness = 2.f, FLinearColor Color = FLinearColor::White) const;

	//---------------------Rendering Proxy---------------------------------
	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void SetRayVolumeBaseMaterial(UMaterialInterface* Mat);

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void SpawnRayVolumeProxy(bool bTwoSided = true);

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void DestroyRayVolumeProxy();

	UFUNCTION(BlueprintCallable, Category = "VTU|Proxy")
	void SetRayVolumeTexture(UTexture2D* RayTex);

	//--------------------------------Push to GPU Shader---------------------------------
	UFUNCTION(BlueprintCallable, Category = "VTU|GPURaymarch")
	void PushProxyBoundsToGPU_FromRayVolumePMC();

	UFUNCTION(BlueprintCallable, Category = "VTU|GPURaymarch")
	void BindRaymarchRTToMID();

	UFUNCTION(BlueprintCallable, Category = "VTU|GPURaymarch")
	void PushGridToGPU();

	UFUNCTION(BlueprintCallable, Category = "VTU|GPURaymarch")
	void SetFeatureSelected(EVTUCellFeature Feat, float InVMin, float InVMax, float InEpsCm);

	private:
		TUniquePtr<FVTUCellOctree> CellOctree;
};